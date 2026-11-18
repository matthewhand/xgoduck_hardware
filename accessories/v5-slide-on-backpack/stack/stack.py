"""V5 removable-pack upper deck and repeatable accessory slide, seated STL mm."""
import argparse
from pathlib import Path
import runpy
import json
import numpy as np
from manifold3d import Manifold

HERE=Path(__file__).resolve().parent
tail=runpy.run_path(str(HERE.parents[1]/'v5-raptor-tail'/'raptor-tail.py'))
box=tail['block']
union=tail['join']
STEP=6.20


def land(x):
    return box(x-3.2,x+3.2,23.4,30,-9.2,-6.2)


def dovetail(top,clearance,back,front):
    depth=3.5
    # Insertion is from the rear (-Z). The wide end is captured below the lips.
    return Manifold.hull_points([(x*(w+clearance),top+y,z)
        for z in (back,front) for y,w in ((-depth-clearance,5.4),(clearance+.3,3.7)) for x in (-1,1)])


def crosspin(top):
    return Manifold.cylinder(17,.95,.95,24).rotate((0,90,0)).translate((-8.5,top-2.0,-16))


def socket(blank,top):
    return blank-dovetail(top,.25,-25,-9.5)-crosspin(top)


def deck():
    top=44.
    blank=box(-38,38,top-5,top,-23,-7)
    for x in (-34.5,34.5):
        foot=land(x)
        post=Manifold.batch_hull([box(x-3.2,x+3.2,28,30,-9.2,-7.2),
                                  box(x-3.2,x+3.2,38.8,40.8,-15,-10)])
        blank+=foot+post
        hole=Manifold.cylinder(8,1.68,1.68,32).translate((x,26.5,-12))
        blank-=hole
    # Open a side service slot aligned with the pin. Leave the region above
    # the pin intact to hold the track under vertical load.
    return socket(blank,top)-box(7.3,39,top-3.15,top-.85,-17.15,-14.85)


def cartridge():
    top=44.+STEP
    blank=box(-30,30,44.20,top,-23,-8)
    male=dovetail(44.,0,-22.4,-9.85)+box(-3.5,3.5,43.5,44.5,-22.4,-9.85)
    return (socket(blank,top)+male)-crosspin(44.)-box(7.3,31,top-3.15,top-.85,-17.15,-14.85)


def pin(width=38.):
    # Flanged pull pin. 1.6 mm shank inside a 1.9 mm bore. Fit-check sample.
    return (Manifold.cylinder(width+8.7,.8,.8,24).rotate((0,90,0)).translate((-8.3,42,-16))+
            Manifold.cylinder(1.4,2,2,32).rotate((0,90,0)).translate((width+.3,42,-16)))


def print_pose(part):
    part=part.rotate((90,0,0))
    return part.translate((0,0,-part.bounding_box()[2]))


def measure(output):
    output.mkdir(exist_ok=True,parents=True)
    deck_part,first,second,lock=deck(),cartridge(),cartridge().translate((0,STEP,0)),pin()
    stock,fixed,cover,_,yoke=tail['base'](.25)
    base=fixed+cover+yoke
    # The deck shares the original yoke bolt axes. Existing screw heads
    # move outward by the foot thickness with correspondingly longer screws.
    overlaps={}
    for name,a,b in [('deck vs enclosure/cover/yoke',deck_part,base),('first vs deck',first,deck_part),
                     ('second vs first',second,first),('pin vs seated first and deck',lock,first+deck_part),
                     ('pin vs seated second',pin(30.).translate((0,STEP,0)),second+first)]:
        overlaps[name]=round((a^b).volume(),5)
        if overlaps[name]>.0001:raise ValueError((name,overlaps[name]))
    # Sweep exactly in -Z by rotating this axis to the shared +Y sweep function.
    sweep=tail['CAD']['sweep']
    swept=sweep(first.rotate((90,0,0)),30).rotate((-90,0,0))
    for fixture in [deck_part+base,first.translate((0,-STEP,0))]:
        if (swept^fixture).volume()>.0001: raise ValueError('Accessory insertion blocked')
    stop=(first.translate((0,0,1))^deck_part).volume()
    locked=(first.translate((0,0,-1))^lock).volume()
    if min(stop,locked)<.01:raise ValueError('Missing stop or pin engagement')
    solids={'upper-deck':deck_part,'stack-cartridge':first,'side-pin':lock,'cartridge-pin':pin(30.)}
    for name,solid in solids.items():
        tail['write_stl'](output/(name+'.stl'),print_pose(solid))
    stats={'clearance_per_face_mm':.25,'stack_pitch_mm':STEP,'overlap_mm3':overlaps,
           'stop_overlap_at_1mm_mm3':round(stop,3),'pin_overlap_at_1mm_mm3':round(locked,3),
           'solid_print_mass_g_at_1p24':{n:round(s.volume()*.00124,3) for n,s in solids.items()}}
    (HERE/'stack-fit.json').write_text(json.dumps(stats,indent=2)+'\n',encoding='utf8')
    print(stats)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    measure(parser.parse_args().output)
