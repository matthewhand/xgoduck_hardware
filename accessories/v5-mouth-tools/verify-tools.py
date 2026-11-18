"""Check mouth-tool pickup, gripping and first contact against the actual V5 print solids."""
from pathlib import Path
import json,runpy
import numpy as np
HERE=Path(__file__).resolve().parent
g=runpy.run_path(str(HERE/'mouth-tools.py'))
# Continuous swept union for translations along Y in the stock coordinate frame.
sweep=runpy.run_path(str(HERE.parent/'v5-slide-on-backpack/charging-pack.py'))['sweep']
eps=1e-4


def hit(a,b):return max(0.,(a^b).volume())


def vertical(s,distance):
    return sweep(s.rotate((-90,0,0)),distance).rotate((90,0,0))


def verify():
    u,p,j,q=g['upper_and_lower']();h=g['handle']();st=g['stand']();pin,cap=g['retainer']()
    grasp_j=g['open_jaw'](j,g['GRASP']);grasp_q=g['open_jaw'](q,g['GRASP'])
    opened=u+p+g['open_jaw'](j,g['OPEN'])+g['open_jaw'](q,g['OPEN'])
    # Stay still in the tool frame. The Duck approaches the loaded stand while
    # open and 2.2 mm above its final height, then dips to touch the upper pad.
    rigid=u+grasp_j
    report={'grasp_angle_deg':g['GRASP'],'open_angle_deg':g['OPEN'],'variants':{}}
    tol=1e-4
    for name in g['TIPS']:
        tip=g['tip'](name); tool=h+tip+pin+cap
        if hit(h,tip)>tol or hit(h,pin)>tol or hit(tip,pin)>tol:
            raise ValueError('Pin or shank interference')
        # Sweep the much smaller tool toward the opened head, at the 2.2 mm
        # height offset. Moving it forward (-Y) is the reciprocal head motion.
        initial=tool.translate((0,0,-2.2))
        entrance=hit(sweep(initial,-30),opened)
        stand_entrance=hit(sweep(st.translate((0,0,-2.2)),-30),opened)
        # Rotating the jaw holds its upper pad stationary. Record clear rigid
        # motion for the full closure range in half-degree samples.
        closure=0.;stand_hits=0.
        for a in np.linspace(g['GRASP'],g['OPEN'],44):
            low=g['open_jaw'](j,a)
            closure=max(closure,hit(tool,u+low))
            stand_hits=max(stand_hits,hit(st,u+p+low+g['open_jaw'](q,a)))
        dip=hit(vertical(tool.translate((0,0,-2.2)),2.2),u+g['open_jaw'](j,g['OPEN']))
        # Check the *tool* translating clear of the stand. Exact prism sweep
        # checks sloped tips and the open sockets as the two bars lift free.
        raised=tool.translate((0,0,20))
        lift=hit(vertical(tool,20),st)
        withdraw=hit(sweep(raised,32),st)
        uv,lv=hit(tool,p),hit(tool,grasp_q)
        if max(entrance,stand_entrance,closure,stand_hits,dip,lift,withdraw)>tol or min(uv,lv)<1:
            raise ValueError(f'{name} failed approach/close/lift {entrance,stand_entrance,closure,stand_hits,dip,lift,withdraw,uv,lv}')
        # Unloaded mass and two-support static stability in the upright station.
        elements=[(h,1.24),(tip,1.20),(pin,1.24),(cap,1.20)]
        masses=[];centers=[]
        for s,density in elements:
            v,cent=g['mesh_moments'](s);masses.append(v*density/1000);centers.append(cent)
        center=np.average(centers,weights=masses,axis=0)
        if not min(g['SEATS'])<center[1]<max(g['SEATS']):raise ValueError('CG outside support polygon')
        # Compare tool apex with the whole nose and other tool vertices on
        # a flat plane; the second contact of a level fork is intentionally tied.
        # Use both pads/rigid parts from the official prints and all tool geometry.
        whole=tool+rigid+p+grasp_q
        mesh=whole.to_mesh();verts=mesh.vert_properties[:,:3]
        ids=np.arange(len(verts)); sites=np.array(g['TIPSITES'][name])
        toolmask=np.min(np.linalg.norm(verts[:,None,:]-sites[None,:,:],axis=-1),axis=1)<g['RADII'][name]+1.5
        checks={}
        for a in (10,20,30,40):
            theta=np.deg2rad(a); heights=verts@np.array([0,np.sin(theta),np.cos(theta)])
            checks[str(a)]=round(float(heights[~toolmask].min()-heights[toolmask].min()),3)
        report['variants'][name]=dict(estimated_mass_g=round(sum(masses),3),cg_mm=np.round(center,3).tolist(),
                              station_support_projection_y_mm=[min(g['SEATS']),max(g['SEATS'])],
                              upper_pad_overlap_mm3=round(uv,3),lower_pad_overlap_mm3=round(lv,3),
                              rigid_closure_mm3=round(closure,7),upper_rigid_dip_mm3=round(dip,7),
                              entry_with_stand_mm3=round(max(entrance,stand_entrance),7),
                              station_lift_mm3=round(lift,7),exit_mm3=round(withdraw,7),
                              tip_first_margin_by_downward_pitch_mm=checks)
    print(json.dumps(report,indent=2))
    (HERE/'fit-results.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':verify()
