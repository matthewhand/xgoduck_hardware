"""Interior steel-target hatch and sealed magnetic beard in upstream V5 JAW coordinates, mm."""
import json
from pathlib import Path
import runpy

import numpy as np
from manifold3d import Manifold

HERE = Path(__file__).resolve().parent
FACE = runpy.run_path(str(HERE.parent/'face-plates.py'))
CAD = FACE['cad']
TRAVEL = 3.6
TARGET_X = (-10., 10.)
TARGET_Y = -18.2


def box(x0,x1,y0,y1,z0,z1):
    return Manifold.cube((x1-x0,y1-y0,z1-z0)).translate((x0,y0,z0))


def disc(x,y,z0,z1,r):
    return Manifold.cylinder(z1-z0,r,r,64).translate((x,y,z0))


def both(part):
    return part + part.mirror((1,0,0))


def polygon(points,z0,z1):
    return FACE['slab'](points,z0,z1-z0)


def tongue():
    # The diagonal nose cams inward as the hatch slides toward the beak.
    return polygon([(16.4,-16.1),(17.0,-16.1),(17.60,-15.45),
                    (17.60,-14.65),(16.4,-14.65)],1.57,2.10)


def keeper(compressed=False):
    # Two free-ended PETG cantilevers can be squeezed through the inside access slots.
    frame = box(-16.9,16.9,-23.15,-13.1,1.48,2.64)
    frame -= both(box(14.7,15.65,-21.35,-12.9,1.4,2.8))
    lug = box(16.6,18.25,-22.75,-20.95,1.54,2.10)
    if compressed:
        # Piecewise rigid deflection envelope, straight taper from root to tip.
        arm = box(15.65,16.9,-21.35,-13.1,1.48,2.64) + tongue()
        frame -= both(arm)
        arm = arm.transform(((1,-.106,0,-.106*21.35),(0,1,0,0),(0,0,1,0)))
        return frame + both(arm) + both(lug)
    return frame + both(lug) + both(tongue())


def receiver_cut():
    # Inside-only loading bay. The guide lips retain the front tabs and rear hooks in Z.
    pocket = box(-17.12,17.12,-23.35,-13.1+TRAVEL+.20,1.42,3)
    lug_sweep = box(16.9,18.46,-22.93,-20.95+TRAVEL+.18,1.42,2.24)
    lug_entry = box(16.9,18.46,-22.93+TRAVEL,-20.95+TRAVEL+.18,1.42,3)
    catch = box(16.9,17.82,-16.29,-14.48,1.42,2.24)
    # Small tool wells above both catch tips provide interior release access.
    # Only the outer ledge (X >17.35) roofs the rear hook.
    access = box(16.40,17.35,-16.25,-14.48,2.16,3)
    for x in TARGET_X:
        pocket += disc(x,TARGET_Y,.75,1.65,4.14)
    return pocket + both(lug_sweep+lug_entry+catch+access)


def beard():
    # Local height increases away from the chin; this is also the print-bed orientation.
    ends = [disc(x,-TARGET_Y,0,3.60,5.15) for x in TARGET_X]
    cradle = Manifold.batch_hull(ends)
    edge = [(-13,2.8),(-10,2.5),(0,2.5),(10,2.5),(13,2.8),(12.2,7.2),
            (9,10.9),(8.4,9.0),(4.1,13.2),(3.5,12.0),(.1,16.3),
            (-3.4,13.0),(-3.9,14.0),(-8.5,9.4),(-8.8,11.7),(-12,7.6)]
    fringe = FACE['slab'](edge,0,2.20).rotate((90,0,0)).translate((0,20.3,0))
    shell = cradle + fringe
    magnets = Manifold()
    for x in TARGET_X:
        shell -= disc(x,-TARGET_Y,.60,2.80,4.14)
        magnets += disc(x,-TARGET_Y,.60,2.60,4.0)
    return FACE['chin'](shell), FACE['chin'](magnets)


def build(jaw=None):
    stock = CAD['read_stock']('JAW') if jaw is None else jaw
    cut = receiver_cut()
    steel = sum([disc(x,TARGET_Y,.75,1.35,4) for x in TARGET_X],Manifold())
    fringe, magnets = beard()
    return dict(host=stock-cut,lid=keeper(),beard=fringe,steel=steel,magnets=magnets)


def vol(p):
    return max(0, p.volume())


def validate(parts,stock):
    host, lid, steel = (parts[k] for k in ('host','lid','steel'))
    shell, magnets = parts['beard'], parts['magnets']
    pad = CAD['read_stock']('MOUTH_BOTTOM_TPU').translate((0,-15.54,2.8))
    fixed = CAD['read_stock']('HEAD_BOTTOM') + CAD['read_stock']('MOUTH_UP_TPU').rotate((180,0,0)).translate((0,-43.840,11))
    # In the installed model all steel is recessed; the original chin remains flat.
    for item in (host,lid):
        CAD['check_solid'](item)
    assert vol(host-stock)<1e-5 and vol((stock-host)^box(-25,25,-30,-5,-1,.749))<1e-5
    assert (steel-host).volume()>60 and vol(steel^(host+lid))<1e-5
    assert vol(pad^(lid+steel))<1e-5 and vol(lid^host)<1e-5
    # A closed magnet cavity produces one positive exterior plus two negative inner shells.
    assert len([p for p in shell.decompose() if p.volume()>0])==1
    assert len([p for p in shell.decompose() if p.volume()<0])==2
    assert vol(magnets^shell)<1e-5
    rigid_reverse = vol(lid.translate((0,.75,0))^host)
    rigid_lift = vol(lid.translate((0,0,.65))^host)
    assert min(rigid_reverse,rigid_lift)>.04
    # Lift the spring tips inward, slide to the drop-in openings, then lift the hatch.
    squeezed=keeper(True)
    collision=max(vol(squeezed.translate((0,y,0))^host) for y in np.linspace(0,TRAVEL,37))
    lift=max(vol(squeezed.translate((0,TRAVEL,z))^host) for z in np.linspace(0,3,31))
    steel_drop=max(vol(steel.translate((0,0,z))^host) for z in np.linspace(0,3,31))
    assert max(collision,lift,steel_drop)<1e-4,(collision,lift,steel_drop)
    assert vol(shell^(host+steel+lid+pad))<1e-5
    gaps=[FACE['jaw_pose'](shell,a).min_gap(fixed,20) for a in np.arange(0,30.01,.25)]
    assert min(gaps)>.5
    # Signed results for removed host plastic and modeled materials, not a pull-force rating.
    mass_g=dict(host_delta=(host.volume()-stock.volume())*.0012,
                lid=lid.volume()*.0012,beard=shell.volume()*.0012,
                mild_steel=steel.volume()*.00785,ndfeb=magnets.volume()*.0075)
    return dict(stock_skin_mm=.75,nominal_magnet_to_steel_gap_mm=1.35,
                steel_disc_diameter_thickness_mm=[8,.60],magnet_diameter_thickness_mm=[8,2],
                hatch_travel_mm=TRAVEL,nominal_inner_height_mm=2.8,lid_top_mm=2.64,
                maximum_hatch_slide_intersection_mm3=collision,maximum_hatch_lift_intersection_mm3=lift,
                maximum_target_insert_intersection_mm3=steel_drop,
                seated_hatch_blocks_rigid_reverse_mm3=rigid_reverse,seated_hatch_blocks_direct_lift_mm3=rigid_lift,
                jaw_sweep_deg=[0,30,.25],minimum_beard_to_opposite_jaw_mm_capped_at_20=min(gaps),
                estimated_mass_g=mass_g,whole_loadout_increment_g=sum(mass_g.values()))


def export(filename,part):
    # Preserve internal voids. The generic one-component writer discards them.
    screw = runpy.run_path(str(HERE.parents[1]/'v5-dino-beak'/'screw-teeth'/'screw-teeth.py'))
    screw['write_host'](screw['bed'](part),HERE/'print'/filename)


def main():
    stock=CAD['read_stock']('JAW')
    parts=build(stock)
    report=validate(parts,stock)
    (HERE/'print').mkdir(exist_ok=True)
    export('jaw-interior-steel-hatch.stl',parts['host'])
    export('chin-hatch-lid.stl',parts['lid'])
    export('scalloped-magnetic-beard.stl',FACE['chin'](parts['beard']))
    coupon=box(-21,21,-25.5,-7,0,2.8)-receiver_cut()
    export('chin-hatch-coupon.stl',coupon)
    screw=runpy.run_path(str(HERE.parents[1]/'v5-dino-beak'/'screw-teeth'/'screw-teeth.py'))
    cheek=screw['build']()['lower']['mod']
    combined=build(cheek)['host']
    # Features on the side walls and under-pad cutouts must remain disjoint.
    side_edits=(stock-cheek)+(cheek-stock)
    report['cheek_pocket_to_hatch_clearance_mm_capped_at_40']=side_edits.min_gap(stock-parts['host'],40)
    assert vol(cheek-stock)<1e-4 and report['cheek_pocket_to_hatch_clearance_mm_capped_at_40']>5
    assert abs((stock-parts['host']).volume()-(cheek-combined).volume())<.001
    export('jaw-flush-cheek-pockets-and-chin-hatch.stl',combined)
    (HERE/'chin-fit.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(report)


if __name__=='__main__':
    main()
