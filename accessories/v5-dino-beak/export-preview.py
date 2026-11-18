"""Register the V5 printable beak STLs to Duck Lab's neutral jaw_soft coordinates."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from manifold3d import Manifold, Mesh

HERE = Path(__file__).resolve().parent
CAD = runpy.run_path(str(HERE/"dino-beak.py"))
DTYPE = CAD["DTYPE"]
HEAD_SHA = "dcbe8ae093a842399e94eecc6f41a533575df9e8367f26690d737a9d38f8b327"
PRINT_TO_HEAD = np.array([[-1.,0,0],[0,0,1],[0,1,0]])
OFFSET = {"upper": np.array([-.00325,-17.,-17.05394]), "lower": np.array([0.,-17.5,-41.94842])}
HINGE = (PRINT_TO_HEAD@np.array([0.,24.05,20.30])+OFFSET["lower"]).tolist()


def registered(v, side):
    return v@PRINT_TO_HEAD.T+OFFSET[side]


def surface(solid):
    mesh=solid.to_mesh()
    return mesh.vert_properties[:,:3], mesh.tri_verts


def indexed(tris):
    p, f=np.unique(tris.reshape(-1,3),axis=0,return_inverse=True)
    return p,f.reshape(-1,3)


def print_triangles(file, raw, side, tusks):
    # Recover the seated part from the actual slicer STL with the inverse of
    # print_pose(), and check against every vertex of its source solid.
    v,f=surface(raw)
    rotation=Rotation.from_euler("xyz",CAD["bed_rotation"](side,tusks),degrees=True).as_matrix()
    vp=v@rotation.T
    lo,hi=vp.min(0),vp.max(0)
    shift=np.array([-(lo[0]+hi[0])/2,-(lo[1]+hi[1])/2,-lo[2]])
    t=np.fromfile(file,dtype=DTYPE,offset=84)["vertex"].astype(float)
    pos,idx=indexed(t)
    # The STL writer simplifies the bed-oriented part by 0.008 mm. Validate
    # the printed vertex set there before reversing the bed translation.
    expected,_=surface(CAD["print_pose"](raw,tusks,side=side).simplify(.008))
    if max(cKDTree(pos).query(expected)[0].max(),cKDTree(expected).query(pos)[0].max())>.002:
        raise ValueError(f"Print differs from registered design: {file}")
    pos=(pos-shift)@rotation
    return registered(pos,side),idx


def stock_head(runtime):
    path=runtime/"assets/sim/model/assets/head_roll.stl"
    data=path.read_bytes()
    if hashlib.sha256(data).hexdigest()!=HEAD_SHA:
        raise ValueError("Check connected-component registration against the current walking-model head")
    points,faces=indexed(np.frombuffer(data,DTYPE,offset=84)["vertex"]*1000)
    edges=coo_matrix((np.ones(faces[:,:2].size),(faces[:,:2].ravel(),faces[:,1:].ravel())),shape=(len(points),len(points)))
    count,labels=connected_components(edges,directed=False)
    if count!=9:
        raise ValueError("Unexpected stock head components")
    # Component ids refer to points sorted lexicographically in the pinned asset.
    # 0 is its rigid HEAD_BOTTOM, 1 its JAW, 2 the main exterior shell.
    for name,selection in [("head-shell",[2]),("head-hardware",[3,4,5,6,7,8])]:
        selected=faces[np.isin(labels[faces[:,0]],selection)]
        p,indices=indexed(points[selected])
        yield name,"fixed","all",p,indices


def meshes(runtime):
    yield from stock_head(runtime)
    for side,base,pad in [("upper","HEAD_BOTTOM","MOUTH_UP_TPU"),("lower","JAW","MOUTH_BOTTOM_TPU")]:
        group="moving" if side=="lower" else "fixed"
        stock=CAD["read_stock"](base)
        p,f=surface(stock.simplify(.035))
        yield ("lower-jaw" if side=="lower" else "head-bottom"),group,"all",registered(p,side),f
        p,f=surface(CAD["read_stock"](pad).simplify(.035))
        if side=="upper": p=p*[1,-1,-1]+[0,-43.840,11.0]
        else: p=p+[0,-15.54,2.8]
        yield side+"-pad",group,"all",registered(p,side),f
        for teeth,tusks,variant in ([(True,False,"teeth"),(False,True,"tusks"),(True,True,"teeth-tusks")] if side=="upper" else [(True,False,"teeth")]):
            raw=CAD["build"](stock,side,teeth,tusks)
            p,f=print_triangles(HERE/"print"/f"{side}-{variant}.stl",raw,side,tusks)
            yield side+"-"+variant,group,variant,p,f


def export(runtime, target):
    geometry=bytearray()
    records=[]
    volumes={}
    for name,group,variant,p,f in meshes(runtime):
        # 0.01 mm coordinate precision. Every packed face fits a 16-bit index.
        coordinates=np.round(p*100)
        if coordinates.min()<=-32768 or coordinates.max()>32767 or len(p)>65535:
            raise ValueError("Preview mesh is outside the fixed-point format")
        offset=len(geometry)
        geometry.extend(coordinates.astype("<i2").tobytes())
        vertices_end=len(geometry)
        geometry.extend(f.astype("<u2").tobytes())
        records.append(dict(name=name,group=group,variant=variant,position_offset=offset,vertices=len(p),index_offset=vertices_end,indices=int(f.size)))
        if variant!="all":
            # Signed tetrahedral volume from the unsimplified printable mesh.
            a,b,c=p[f].transpose(1,0,2)
            volumes[name]=abs(np.einsum("ij,ij->",a,np.cross(b,c)))/6000
    combined={"teeth":volumes["upper-teeth"]+volumes["lower-teeth"],"tusks":volumes["upper-tusks"],"combined":volumes["upper-teeth-tusks"]+volumes["lower-teeth"]}
    mass={k:{n:round(v*d,4) for n,d in [("low",1.05),("nominal",1.20),("high",1.30)]} for k,v in combined.items()}
    target.mkdir(exist_ok=True,parents=True)
    (target/"dino-head.bin").write_bytes(geometry)
    manifest=dict(format=1,units_per_metre=100000,hinge_mm=HINGE,mass_by_variant_g=mass,
        mass_assumption="Cuffs are additions to the stock mouth; 1.05/1.20/1.30 g/cm3 equivalent print densities. Weigh the final TPU parts.",
        source_head_sha256=HEAD_SHA,source_hardware_revision=CAD["REVISION"],meshes=records)
    (target/"dino-head.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
    print(len(geometry),"bytes",mass)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runtime",type=Path)
    args=parser.parse_args()
    export(args.runtime,args.runtime/"assets/sim")
