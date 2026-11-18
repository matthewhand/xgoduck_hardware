"""Round the outer face of the V1.1 guard, retaining the original slide and openings."""
import argparse
from functools import lru_cache
import json
from pathlib import Path
import runpy
import sys

import numpy as np
from manifold3d import CrossSection, Manifold, Mesh

HERE = Path(__file__).resolve().parent
RAILS = HERE.parent/'removable-rails'
TAIL = HERE.parents[1]/'v5-raptor-tail'
sys.path.insert(0,str(TAIL))
rails = runpy.run_path(str(RAILS/'rail-cartridges.py'))
cad = rails['C']
from tail_physics import trunk, triangles, collider_vertices, mass_properties


@lru_cache()
def base():
    stock,receiver,rail,*_=rails['build']()
    fixed,plain,rigid,tooth=cad['build'](stock,.25,charging_contacts=False)
    track=rail+rails['left_rail'](rail)
    return fixed,receiver,track,plain,rigid,tooth


def rounded(depth,station,half_width,flare=0,side_y=-13,side_z=-3):
    # Two low ellipsoidal shoulders on the existing face. Trim their footprint
    # to the guard lands so the vents, service holes and latch stay accessible.
    plain=base()[3]
    section=plain.slice(-2.6)
    lands=section.extrude(depth+2).translate((0,0,-3.2-depth))
    lands-=cad['block'](-43,-32.5,-24,2,-25,4)
    sections=[Manifold.sphere(1,40).scale((half_width,34,depth+.25)).translate((x,station,-2.95)) for x in (-25,25)]
    extra=(sections[0]+sections[1]) ^ lands
    if flare:
        # Oval contact pads grow outward from the solid outer edges. Leave
        # the flex tongue and its finger well open. Everything lies on the
        # exterior side of the rail grooves.
        side_lands=cad['block'](-80,80,-35,4,-30,-1.9)
        side_lands-=cad['block'](-40,-32.5,-24,2,-30,4)
        for sign in (-1,1):
            pad=Manifold.sphere(1,32).scale((flare+3,16,4)).translate((sign*39,side_y,side_z))
            extra+=pad ^ side_lands
    return plain + extra, extra


def profile(cover):
    track=base()[2]
    v=trunk(triangles(cover+track))
    mass,com,I=mass_properties(v,np.full(len(v),1240.))
    return dict(parent='trunk_base',mass_kg=mass,com_m=com.tolist(),inertia_kg_m2=I.tolist(),
                collider_hulls_m=collider_vertices(cover,np.zeros(3),'mount')+collider_vertices(track,np.zeros(3),'mount'))


def export(parameters, output=HERE, profile_name='rounded-profile.json'):
    fixed,receiver,track,plain,rigid,tooth=base()
    cover,extra=rounded(*parameters)
    path=output/'rounded-protective-cover.stl'
    bed=cover.simplify(.002).rotate((180,0,0))
    bed=bed.translate((0,0,-bed.bounding_box()[2]))
    # The section/extrusion shares coplanar edges with the original plate.
    # Weld coincident vertices at STL precision before writing the surface.
    mesh=bed.to_mesh()
    xyz,indices=np.unique(mesh.vert_properties[mesh.tri_verts,:3].reshape(-1,3),axis=0,return_inverse=True)
    faces=indices.reshape(-1,3)
    faces=faces[np.all(np.diff(np.sort(faces,axis=1),axis=1)>0,axis=1)]
    printable=Manifold(Mesh(xyz,faces.astype(np.uint32)))
    assert abs(printable.volume()-cover.volume())<1, (printable.status(),printable.volume(),cover.volume())
    cad['write_stl'](path,printable)
    runpy.run_path(str(TAIL.parent/'v5-dino-beak/verify-fit.py'))['check_export'](path)
    moving=rigid+extra+tooth.translate((0,0,-.85))
    swept=cad['sweep'](moving,76)
    gaps=[max(0,(swept^fixture).volume()) for fixture in (fixed,receiver+track)]
    removed=max(0,(plain-cover).volume())
    added=extra-plain
    # The lateral pads extend past the old outline. Check the actual clear
    # openings and flexure access rather than treating air outside as an opening.
    def rectangle(x0,x1,y0,y1):
        return CrossSection.square((x1-x0,y1-y0)).translate((x0,y0))
    reserved=rectangle(-1,22.35,-25.2,3.8)+rectangle(-1,41,8,22.2)
    for y in [-21,-15,-9,-3,3,9]:
        reserved+=rectangle(-18,-5,y,y+2)
    for x,y in [(-8,-31.9),(20,-31.9),(-23.9,18.9)]:
        reserved+=CrossSection.circle(4.1,32).translate((x,y))
    blocked=max(0,(added.project() ^ reserved).area())
    latch=max(0,(added^cad['block'](-40,-32.5,-24,2,-25,4)).volume())
    engaged=(cover.translate((0,1.5,0))^(receiver+track)).volume()
    stop=(moving.translate((0,-.5,0))^(receiver+track)).volume()
    assert max(gaps+[blocked,removed,latch])<1e-4,(gaps,blocked,removed,latch)
    assert min(engaged,stop)>.1,(engaged,stop)
    report=dict(shoulder_depth_mm=parameters[0],shoulder_center_y_mm=parameters[1],
                shoulder_half_width_mm=parameters[2],side_flare_mm=parameters[3],
                side_center_y_mm=parameters[4],side_center_z_mm=parameters[5],
                before_bounds_mm=plain.bounding_box(),after_bounds_mm=cover.bounding_box(),
                release_slide_mm=76,service_overlap_mm3=gaps,locked_interference_at_1p5_mm3=engaged,
                stop_interference_at_half_mm3=stop,original_material_removed_mm3=removed,
                blocked_opening_area_mm2=blocked,extra_material_at_release_mm3=latch,
                added_print_mass_g=round(added.volume()*.00124,3))
    (output/'fit.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/profile_name).write_text(json.dumps(profile(cover),separators=(',',':'))+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parameters',type=float,nargs=6)
    print(export(parser.parse_args().parameters))
