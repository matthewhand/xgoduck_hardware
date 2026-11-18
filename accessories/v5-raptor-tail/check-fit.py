"""Mechanical checks for the V5 passive tail using triangle solids in millimetres."""
from pathlib import Path
import json
import runpy
import numpy as np

HERE=Path(__file__).resolve().parent
d=runpy.run_path(str(HERE/'raptor-tail.py'))
cad=d['CAD']
rot=d['rotate']
block=d['block']
root,knee=d['ROOT'],d['KNEE']
TOL=1e-4


def overlap(a,b):
    return max(0.,(a^b).volume())


def run():
    checks={}
    for variant in ['standard','short']:
        parts=d['components'](variant)
        for n,(group,s) in parts.items():
            if n!='rail-addition': cad['checked'](s)
        stock,fixed,cover,_,_=d['base'](.25)
        group=d['assembly_parts'](variant)
        if any(overlap(fixed,s)>TOL for n,(_,s) in parts.items() if n!='rail-addition'):
            raise ValueError('Tail collides with host')
        parked=group['mount']
        proximal,distal=group['proximal'],group['distal']
        for i in np.linspace(-18,18,73):
            if max(overlap(parked,rot(proximal,root,[i,0,0])),
                   overlap(proximal,rot(distal,knee,[0,i,0])))>TOL:
                raise ValueError(f'Hinge obstruction at {i}')
        stops=[]
        for g,other,center,axis in [(proximal,parked,root,np.array([1,0,0])),(distal,proximal,knee,np.array([0,1,0]))]:
            angles=[]
            for sign in [-1,1]:
                lo,hi=18.,22.
                for _ in range(12):
                    mid=(lo+hi)/2
                    if overlap(other,rot(g,center,axis*mid*sign))>TOL: hi=mid
                    else:lo=mid
                angles.append(round(sign*hi,3))
                if not 18.<hi<20:raise ValueError('Stops outside expected range')
            stops.append(angles)
        for p in [-16,-8,0,8,16]:
            for y in [-16,-8,0,8,16]:
                distal_pose=rot(rot(distal,knee,[0,y,0]),root,[p,0,0])
                if overlap(parked,distal_pose)+overlap(fixed,distal_pose)>TOL:
                    raise ValueError('Tip hits mount')
        # Clearance for the entire service translation: sweep the released
        # cover exactly; moving tail/fastener extent boxes suffice for the
        # rigid enclosure because the tail stays behind its rear plane.
        for n,(g,s) in parts.items():
            if n in ['tail-cover','rail-addition']:continue
            lo,hi=np.array(s.bounding_box()).reshape(2,3)
            envelope=block(lo[0],hi[0],lo[1],hi[1]+76,lo[2],hi[2])
            if overlap(envelope,fixed)>TOL:
                raise ValueError('Service path blocked for '+n)
        checks[variant]={'working_deg':[-16,16],'poses_sampled':25,'individual_angles_sampled':73,'onset_deg_pitch_yaw':stops}
    aperture=block(-29.6,-18.4,-26,0,0,3)
    reader=runpy.run_path(str(d['BACKPACK']/'pack-id.py'))
    electronics=reader['keepouts']()
    tag=electronics['tag']
    checks['rail']={}
    for gap in [.15,.25,.35]:
        stock,fixed,cover,released,_=d['base'](gap)
        free=overlap(cad['sweep'](released,76),fixed)
        wired_host=fixed+reader['reader_mount']()+electronics['reader']+electronics['antenna']
        nfc_sweep=overlap(cad['sweep'](released+tag,76),wired_host)
        down=overlap(released.translate((0,-.5,0)),fixed)
        up=overlap(cover.translate((0,1.5,0)),fixed)
        if max(free,nfc_sweep)>TOL or min(down,up)<.1 or overlap(cover,tag)>TOL or overlap(aperture,fixed)>TOL or overlap(aperture,cover)>TOL:
            raise ValueError('Rail, tag or antenna fit failed')
        checks['rail'][str(gap)]={'sweep_mm3':round(free,6),'reader_and_tag_sweep_mm3':round(nfc_sweep,6),'bottom_stop_mm3':round(down,3),'latch_overlap_mm3':round(up,3)}
    checks['tag']={'placeholder_mm':[6,17,.7],'reader_clearance_checked_mm':[11.2,26,3],
                   'first_shared_rail_obstruction_z_mm':4.4,'first_stock_obstruction_z_mm':6.0}
    print(json.dumps(checks,indent=2))
    (HERE/'fit-results.json').write_text(json.dumps(checks,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    run()
