"""Two removable T-slot rail cartridges for the V5 V1.1 enclosure. All lengths in mm."""
from pathlib import Path
import argparse,json,runpy,sys
import numpy as np
from manifold3d import Manifold,OpType,CrossSection

HERE=Path(__file__).resolve().parent
C=runpy.run_path(str(HERE.parent/'charging-pack.py'))
B=C['block']
TRAVEL=5.8
SLOTS=(-22.,-6.5)
SURFACE=29.895
W=15.7


def pair(part):
    return part+part.mirror((1,0,0))

def left_rail(right):
    x=C['LEFT']
    return right.mirror((1,0,0))-B(x-2.5,x+2.5,-19,-13.7,-.1,1.2)


def right_cuts():
    holes=[]
    for y in SLOTS:
        # Under the retained surface lip the foot of the T can translate in Y.
        holes.append(B(28.48,29.22,y-1.8,y+TRAVEL+1.8,W-2.95,W+2.95))
        holes.append(B(29.12,30,y-1.8,y+TRAVEL+1.8,W-1.62,W+1.62))
        holes.append(B(28.48,30,y+TRAVEL-1.82,y+TRAVEL+1.82,W-2.95,W+2.95))
    holes.append(B(29.00,30,-12.5,-8.8,20.0,23.3))
    return Manifold.batch_boolean(holes,OpType.Add)


def build():
    stock=C['stock_enclosure']()
    fixed,pack,rigid,tooth=C['build'](stock,.25)
    receiver=stock-pair(right_cuts())
    # The clip body starts outside the official enclosure wall. The rail head
    # and lower stop keep their original seated position and cover clearance.
    rail=(fixed-stock)^B(30.08,50,-30,2,-1,8)
    spine=B(30.08,32.0,-27,1,4.5,19.9)
    rail+=spine
    for y in SLOTS:
        stem=B(29.075,30.25,y-1.51,y+1.51,W-1.32,W+1.32)
        head=B(28.70,29.08,y-1.51,y+1.51,W-2.62,W+2.62)
        rail+=stem+head
    # Flexure rooted toward the feet. The external raised tip provides a grip
    # when pulling the tooth away from the sidewall during service.
    root=B(30.08,31.45,-26,-23,19.45,23.0)
    tongue=B(30.08,31.45,-26,-8.0,20.2,22.95)
    grip=B(31.35,33.15,-10.9,-8.05,20.1,23.05)
    snap=B(29.26,30.20,-12.18,-9.12,20.35,22.90)
    body=rail+root
    cartridge=body+tongue+grip+snap
    # Retraction by 1.05 mm at the detent clears the pocket. Evaluate the
    # rigid surfaces at both endpoints of the two prescribed service motions.
    released=body+(tongue+grip+snap).translate((1.05,0,0))
    C['checked'](receiver); C['checked'](cartridge)
    return stock,receiver,cartridge,released,pack,rigid+tooth.translate((0,0,-.85))


def x_sweep(s,distance):
    return C['sweep'](s.rotate((0,0,90)),distance).rotate((0,0,-90))


def check_and_profile():
    stock,rec,rail,rel,pack,free=build()
    rig=rec+rail+left_rail(rail)
    checks={}
    def record(k,a,b):
        f=max(0.,(a^b).volume());checks[k]=round(f,7);return f
    zero=[
        record('seated',rail,rec),record('cover_seated',pack,rig),
        record('cover_service',C['sweep'](free,76),rig),
        record('unlocking_slide',C['sweep'](rel,TRAVEL),rec),
        record('lateral_withdrawal',x_sweep(rel.translate((0,TRAVEL,0)),14),rec)]
    stop=record('lower_cover_stop',free.translate((0,-.5,0)),rig)
    latch=record('cover_latch',pack.translate((0,1.5,0)),rig)
    catch=record('rail_detent',rail.translate((0,1.2,0)),rec)
    trapped=record('rail_heads',rail.translate((.8,0,0)),rec)
    removed=stock-rec
    print('checks',checks,'removed',removed.volume())
    if max(zero)>1e-4 or min(stop,latch,catch,trapped)<.1:raise ValueError('Failed interference checks')
    sys.path.insert(0,str(HERE.parents[1]/'v5-raptor-tail'))
    from tail_physics import triangles,mass_properties,trunk,collider_vertices
    vertices=trunk(triangles(removed));mass,center,inertia=mass_properties(vertices,np.full(len(vertices),1240.))
    # A negative-only subtraction wholly between the existing stock exterior
    # extrema preserves convex support. Compare the exact stock hull vertices.
    from scipy.spatial import ConvexHull,cKDTree
    stock_vertices=np.unique(triangles(stock).reshape(-1,3),axis=0)
    hull=stock_vertices[ConvexHull(stock_vertices).vertices]
    remaining=rec.to_mesh().vert_properties[:,:3]
    max_delta=float(cKDTree(remaining).query(hull)[0].max())
    if max_delta>.002:raise ValueError('Stock hull changed')
    cartridges=rail+left_rail(rail)
    tris=trunk(triangles(cartridges))
    positive,cc,ii=mass_properties(tris,np.full(len(tris),1240.))
    return dict(part='stock V1.1 enclosure with two pairs of recessed keyways per side',mass_removed_kg=mass,removed_com_m=center.tolist(),
                removed_inertia_kg_m2=inertia.tolist(),retained_stock_hull_vertex_error_mm=max_delta,checks_mm3=checks,
                cartridges=dict(mass_kg=positive,com_m=cc.tolist(),inertia_kg_m2=ii.tolist(),
                                collider_hulls_m=collider_vertices(cartridges,np.zeros(3),'mount')),
                installed_rail_volume_cm3=(rail.volume()+left_rail(rail).volume())/1000,
                installed_extra_vs_stock_g=(rail.volume()+left_rail(rail).volume()-removed.volume())*.00124)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,default=HERE/'print')
    args=p.parse_args();profile=check_and_profile()
    (HERE/'receiver-profile.json').write_text(json.dumps(profile,indent=2)+'\n',encoding='utf-8')
    stock,rec,rail,*_=build()
    out=args.out;out.mkdir(parents=True,exist_ok=True)
    C['write_stl'](out/'flush-receiver-v1p1.stl',rec)
    for n,s in [('right',rail),('left',left_rail(rail))]:
        rotated=s.rotate((0,0,0));low=np.array(rotated.bounding_box()).reshape(2,3)[0]
        C['write_stl'](out/(n+'-rail-cartridge.stl'),rotated.translate(-low))
    print(profile)


if __name__=='__main__':main()
