"""Render projected tusk visibility from the nominal forward-facing V5 lens site."""
import argparse
import json
import sys
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from PIL import Image, ImageDraw
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from cad_figure import raster, label, bg


def near_clip(tris):
    """Clip triangles to positive depth before perspective division."""
    depth = tris[:,:,2]
    inside = depth >= .5
    result = [tris[inside.all(axis=1)]]
    for tri in tris[inside.any(axis=1) & ~inside.all(axis=1)]:
        polygon = []
        for a,b in zip(tri, np.roll(tri,-1,axis=0)):
            if a[2]>=.5:
                polygon.append(a)
            if (a[2]>=.5) != (b[2]>=.5):
                polygon.append(a+(.5-a[2])/(b[2]-a[2])*(b-a))
        result.append([ [polygon[0],polygon[i],polygon[i+1]] for i in range(1,len(polygon)-1)])
    return np.concatenate(result)


def photograph(sim, vfov):
    model = ElementTree.parse(sim/"model/robot_walk.xml")
    lens = np.fromstring(model.find(".//site[@name='head_camera']").get("pos"),sep=" ")*1000
    manifest = json.loads((sim/"dino-head.json").read_text())
    data = (sim/"dino-head.bin").read_bytes()
    w,h=640,480
    focal=h/(2*np.tan(np.deg2rad(vfov/2)))
    image=np.empty((h,w,3),np.uint8); image[:]=[222,233,234]
    depth=np.zeros((h,w),np.float32)
    for item in manifest["meshes"]:
        if item["variant"]!="all" and item["name"] not in ("upper-teeth-tusks","lower-teeth"):
            continue
        p=np.frombuffer(data,"<i2",count=item["vertices"]*3,offset=item["position_offset"]).reshape(-1,3)*.01
        f=np.frombuffer(data,"<u2",count=item["indices"],offset=item["index_offset"]).reshape(-1,3)
        tris=near_clip(((p-lens)*[1,1,-1])[f])
        if not len(tris): continue
        normals=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0])
        light=np.abs(normals@np.array([.32,.81,.49]))/np.maximum(np.linalg.norm(normals,axis=1),1e-12)
        cuff=item["name"]=="upper-teeth-tusks"
        rgb=np.array([231,161,52] if cuff else [70,82,88])
        cols=np.uint8((.68+.31*light[:,None])*rgb)
        projected=tris.copy(); projected[:,:,2]=1/tris[:,:,2]
        projected[:,:,0]=w/2+focal*tris[:,:,0]*projected[:,:,2]
        projected[:,:,1]=h/2-focal*tris[:,:,1]*projected[:,:,2]
        raster(projected.astype(np.float32),cols,depth,image)
    mask=(image[:,:,0].astype(float)>image[:,:,1].astype(float)*1.2)
    for x0 in (0,w//2):
        yy,xx=np.where(mask[:,x0:x0+w//2])
        print(vfov, "left" if x0==0 else "right", len(xx),"pixels",
              [int(xx.min()+x0),int(yy.min()),int(xx.max()+x0),int(yy.max())] if len(xx) else [])
        if vfov==45:
            assert len(xx)>100 and yy.min()>h*.55
    if vfov==45: assert not mask[:,int(w*.2):int(w*.8)].any()
    return Image.fromarray(image)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runtime",type=Path)
    args=parser.parse_args()
    sim=args.runtime/"assets/sim"
    view=photograph(sim,45)
    for angle in (35,55): photograph(sim,angle)
    card=Image.new("RGB",(680,583),bg)
    label(card,(20,9),"Predicted view from the V5 camera",25,True)
    label(card,(20,47),"640 × 480 full frame   ·   assumed 45° vertical field of view",17)
    card.paste(view,(20,80))
    card.save(sim/"tusk-camera.png",optimize=True)
