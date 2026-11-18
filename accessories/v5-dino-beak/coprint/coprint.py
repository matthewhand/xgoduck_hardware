"""Mechanically captive TPU cheek teeth in pinned V5 rigid shells. Coordinates are mm."""
import json
from pathlib import Path
import runpy
from xml.etree import ElementTree as ET
from zipfile import ZipFile, ZIP_DEFLATED

import numpy as np
from manifold3d import Error, Manifold, Mesh, OpType

HERE = Path(__file__).resolve().parent
CAD = runpy.run_path(str(HERE.parent / 'dino-beak.py'))
FIT = runpy.run_path(str(HERE.parent / 'verify-fit.py'))
CHIN = runpy.run_path(str(HERE.parents[1] / 'v5-face-plates' / 'magnetic' / 'chin.py'))
NAMESPACE = 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
STOCK = {'upper':'HEAD_BOTTOM', 'lower':'JAW'}
STATIONS = {'upper':[6.5,13.5,20.5,27.5], 'lower':[4.8,11.8,18.8,25.8]}


def box(center,size):
    return Manifold.cube(size,True).translate(center)


def united(parts):
    return Manifold.batch_boolean(parts,OpType.Add)


def tooth(side,y):
    upper=side=='upper'
    z=12.62 if upper else 4.6
    # A transverse buried T-bar keeps a narrow neck captured even if the two
    # materials do not chemically bond. Square ends provide anti-rotation.
    key=box((42.72,y,z),(0.95,4.15,1.80))
    neck=box((43.82,y,z),(2.25,1.80,1.40))
    roots=[CAD['egg']((44.82,y+dy,z),(.92,.92,.91)) for dy in (-.82,.82)]
    tip=CAD['egg'](((48.25 if upper else 45.4),y,(7.05 if upper else 8.55)),(.63,.53,.58))
    return key+neck+Manifold.batch_hull([*roots,tip]), key


def build(side):
    stock=CAD['read_stock'](STOCK[side])
    teeth=[];keys=[]
    for reach in STATIONS[side]:
        part,key=tooth(side,CAD['FRONT'][side]+reach)
        for sign in [1,-1]:
            teeth.append(part if sign==1 else part.mirror((1,0,0)))
            keys.append(key if sign==1 else key.mirror((1,0,0)))
    soft=united(teeth)
    rigid=stock-soft
    assert rigid.status()==Error.NoError
    assert len([c for c in rigid.decompose() if c.volume()>0])==1
    assert len(soft.decompose())==8
    for t in teeth: CAD['check_solid'](t)
    assert abs((soft^rigid).volume())<.0001
    assert abs((stock-(rigid+soft)).volume())<.0001
    assert max((k-stock).volume() for k in keys)<.0001
    return dict(stock=stock,rigid=rigid,soft=soft,teeth=teeth,keys=keys)


def fit(parts):
    report={}
    for side,r in parts.items():
        outward=[]
        fore=[]
        for i,t in enumerate(r['teeth']):
            direction=1 if i%2==0 else -1
            outward.append((t.translate((direction*.65,0,0))^r['rigid']).volume())
            fore.extend((t.translate((0,j*.65,0))^r['rigid']).volume() for j in (-1,1))
        assert min(outward)>.10 and min(fore)>.10
        report[side]=dict(roots=8,solid_tpu_volume_cm3=r['soft'].volume()/1000,
            displaced_rigid_volume_cm3=(r['stock']-r['rigid']).volume()/1000,
            minimum_intersection_outward_shift_065mm_mm3=min(outward),
            minimum_intersection_fore_aft_shift_065mm_mm3=min(fore))
    head,lower=parts.values()
    angle='static'
    jaw=FIT['jaw_pose']
    upper_pad=CAD['read_stock']('MOUTH_UP_TPU').rotate((180,0,0)).translate((0,-43.840,11))
    lower_pad=CAD['read_stock']('MOUTH_BOTTOM_TPU').translate((0,-15.54,2.8))
    face=CAD['read_stock']('FACE').transform(((0,-1,0,0),(0,0,1,-54.848),(-1,0,0,33.258)))
    tof=CAD['read_stock']('FACE_TOF').transform(((1,0,0,24.693),(0,0,-1,-48.048),(0,1,0,30.627)))
    gaps={}
    def gap(name,a,b):
        d=2.0 if b.is_empty() else a.min_gap(b,2.)
        if d<.001: raise ValueError(f'{name} collision at {angle} degrees, {(a^b).volume()} mm3')
        gaps[name]=min(gaps.get(name,2.),float(d))
    top,bot=head['soft'],lower['soft']
    for name,s in [('upper pad',upper_pad),('face',face),('ToF housing',tof)]:
        gap('upper teeth vs '+name,top,s)
    gap('lower teeth vs lower pad',bot,lower_pad)
    # Only the cheek region lies within the 2 mm query cap. Jaw rotation about
    # the X axis leaves lateral position unchanged to within the registered 0.004 mm.
    crop=box((48,0,12),(22,185,110));crop+=crop.mirror((1,0,0))
    zones=[s^crop for s in [lower['rigid'],lower_pad,head['rigid'],upper_pad,face,tof]]
    for a in range(121):
        angle=a*.25
        moving=jaw(bot,angle)
        gap('opposing teeth',top,moving)
        gap('upper teeth vs lower shell',top,jaw(zones[0],angle))
        gap('upper teeth vs lower pad',top,jaw(zones[1],angle))
        for name,s in zip(('upper shell','upper pad','face','ToF housing'),zones[2:]):
            gap('lower teeth vs '+name,moving,s)
    combined=CHIN['build'](lower['rigid'])
    cuts=lower['rigid']-combined['host']
    geometry=lower['soft']
    distances={k:float(geometry.min_gap(v,50)) for k,v in
               [('hatch excavation',cuts),('lid',combined['lid']),('steel targets',combined['steel']),
                ('beard',combined['beard']),('magnets',combined['magnets'])]}
    assert min(distances.values())>3
    assert abs(lower['soft'].volume()-(lower['soft']-combined['host']).volume())<.001
    # The hatch opens inward and translates 3.6 mm in Y. A separation on X
    # larger than its motion also keeps the whole withdrawal path clear.
    assert combined['lid'].bounding_box()[3]+10<min(k.bounding_box()[0] for k in lower['keys'][::2])
    # Pitch is about X, so this lateral bound holds over all 121 poses.
    inner_x=min(k.bounding_box()[0] for k in head['keys'][::2])
    widest=max(max(abs(v.bounding_box()[0]),abs(v.bounding_box()[3]))
               for k,v in combined.items() if k!='host')
    report['upper_teeth_to_moving_chin_parts_lateral_bound_mm']=inner_x-widest-.004
    assert report['upper_teeth_to_moving_chin_parts_lateral_bound_mm']>20
    report['minimum_clearance_mm_capped_at_2']=gaps
    report['sampled_jaw_deg']=[0,30,.25]
    report['lower_chin_compatibility_mm_capped_at_50']=distances
    report['nominal_mass_change_g']=sum(r['solid_tpu_volume_cm3']*1.20-r['displaced_rigid_volume_cm3']*1.20 for r in (report['upper'],report['lower']))
    return report,combined


def export_3mf(path, rigid, tpu):
    """Write two registered volumes as one assembly with one build item."""
    ns=NAMESPACE
    # Coordinates stay in the upstream part frame. Any bed placement is shared.
    z_lift=-min(rigid.bounding_box()[2],tpu.bounding_box()[2])
    with ZipFile(path,'w',ZIP_DEFLATED,compresslevel=3) as archive:
        archive.writestr('[Content_Types].xml','<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        archive.writestr('_rels/.rels','<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="model" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        with archive.open('3D/3dmodel.model','w') as stream:
            def emit(s):stream.write(s.encode('utf8'))
            emit(f'<model unit="millimeter" xml:lang="en-US" xmlns="{ns}"><resources>')
            emit('<basematerials id="1"><base name="Rigid shell" displaycolor="#6B8586"/><base name="TPU teeth" displaycolor="#E6A63C"/></basematerials>')
            for oid,name,shape,material in [(2,'Rigid shell',rigid,0),(3,'TPU teeth',tpu,1)]:
                mesh=shape.to_mesh()
                emit(f'<object id="{oid}" type="model" name="{name}" pid="1" pindex="{material}"><mesh><vertices>')
                emit(''.join(f'<vertex x="{x:.8f}" y="{y:.8f}" z="{z:.8f}"/>' for x,y,z in mesh.vert_properties[:,:3]))
                emit('</vertices><triangles>')
                emit(''.join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a,b,c in mesh.tri_verts))
                emit('</triangles></mesh></object>')
            emit('<object id="4" type="model" name="Co-printed V5"><components><component objectid="2"/><component objectid="3"/></components></object></resources>')
            emit(f'<build><item objectid="4" transform="1 0 0 0 1 0 0 0 1 0 0 {z_lift:.8f}"/></build></model>')
    # Round-trip the on-disk coordinates, topology, material mapping and build
    # transforms. In particular an importer must not drop TPU to the bed alone.
    with ZipFile(path) as archive:
        assert archive.testzip() is None
        root=ET.fromstring(archive.read('3D/3dmodel.model'))
    def items(parent,tag):return parent.findall('m:'+tag,{'m':ns})
    resources=items(root,'resources')[0]
    o1,o2,assy=items(resources,'object')
    assert [o1.attrib['pindex'],o2.attrib['pindex']]==['0','1']
    assert [v.attrib for v in items(items(assy,'components')[0],'component')]==[{'objectid':'2'},{'objectid':'3'}]
    item=items(items(root,'build')[0],'item')
    assert len(item)==1 and item[0].attrib['objectid']=='4'
    placement=list(map(float,item[0].attrib['transform'].split()))
    assert np.allclose(placement,[1,0,0,0,1,0,0,0,1,0,0,z_lift])
    recovered=[]
    for obj,original in [(o1,rigid),(o2,tpu)]:
        mesh=items(obj,'mesh')[0]
        vs=np.array([[v.attrib[a] for a in 'xyz'] for v in items(items(mesh,'vertices')[0],'vertex')],dtype=np.float32)
        tri=np.array([[v.attrib[a] for a in ['v1','v2','v3']] for v in items(items(mesh,'triangles')[0],'triangle')],dtype=np.uint32)
        edges=np.concatenate((tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]))
        assert np.all(np.unique(np.sort(edges,axis=1),return_counts=True,axis=0)[1]==2)
        rebuilt=Manifold(Mesh(vs,tri))
        assert rebuilt.status()==Error.NoError
        assert abs(rebuilt.volume()-original.volume())<.02
        assert np.allclose(rebuilt.bounding_box(),original.bounding_box(),atol=1e-5)
        recovered.append(rebuilt)
    assert abs((recovered[0]^recovered[1]).volume())<.001
    assert abs(min(r.bounding_box()[2]+z_lift for r in recovered))<1e-5


def main():
    parts={s:build(s) for s in STOCK}
    report,combined=fit(parts)
    out=HERE/'print';out.mkdir(exist_ok=True)
    for side,r in parts.items():
        export_3mf(out/(STOCK[side].lower()+'-keyed-teeth.3mf'),r['rigid'],r['soft'])
    lower=parts['lower']
    export_3mf(out/'jaw-keyed-teeth-magnetic-hatch.3mf',combined['host'],lower['soft'])
    r=parts['upper']
    # A real upper cheek section retaining its true host/TPU partition.
    y=CAD['FRONT']['upper']+STATIONS['upper'][2]
    section=box((44,y,12.00),(13,6.50,14.0))
    coupon=r['rigid']^section
    # The inboard cut face and its grip foot go down together. The tooth grows
    # above the section instead of balancing the coupon on its rounded tip.
    foot=box((37.04,y,12.0),(.96,12,10))
    coupon+=foot
    CAD['check_solid'](coupon)
    export_3mf(out/'one-tooth-cheek-coupon.3mf',coupon.rotate((0,-90,0)),r['teeth'][4].rotate((0,-90,0)))
    (HERE/'fit.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
