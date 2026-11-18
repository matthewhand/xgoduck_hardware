"""Render V5 pickup geometry and synthetic aim from the model's lens landmark."""
import argparse,json,runpy,sys
from pathlib import Path
from xml.etree import ElementTree
import numpy as np
from PIL import Image,ImageDraw

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from cad_figure import render,label,bg,ink,muted,raster
g=runpy.run_path(str(HERE/'mouth-tools.py'))
near_clip=runpy.run_path(str(HERE.parent/'v5-dino-beak/check-camera.py'))['near_clip']
TIPCOL={'pointer':[238,166,66],'fork':[85,168,155],'horn':[198,129,91]}
HANDLE=[72,103,117];PIN=[91,96,98];CAP=[163,121,99];CH=[177,188,173];PAD=[85,94,96];ST=[166,177,191]
R=np.array([[-1.,0,0],[0,0,1],[0,1,0]])
OFFSET=np.array([-.00325,-17.,-17.05394])


def t(s):
    m=s.to_mesh();return m.vert_properties[m.tri_verts,:3].astype(np.float32)


def head_meshes(sim,angle):
    # Restore the stock neutral head-shell and face electronics, and swing the
    # exact lower print and TPU pad about the measured horn axis.
    spec=json.loads((sim/'dino-head.json').read_text())
    raw=(sim/'dino-head.bin').read_bytes()
    parts=[]
    for rec in spec['meshes']:
        if rec['name'] not in ('head-shell','head-hardware'):continue
        v=np.frombuffer(raw,'<i2',count=3*rec['vertices'],offset=rec['position_offset']).reshape(-1,3)*.01
        ix=np.frombuffer(raw,'<u2',count=rec['indices'],offset=rec['index_offset']).reshape(-1,3)
        parts.append((((v-OFFSET)@R)[ix].astype(np.float32),CH if rec['name']=='head-shell' else [68,82,86]))
    u,p,j,l=g['upper_and_lower']()
    return parts+[(t(s.simplify(.06)),c) for s,c in [(u,[129,153,145]),(p,PAD),(g['open_jaw'](j,angle),[109,141,132]),(g['open_jaw'](l,angle),PAD)]]


def camera(head, assembly, tips, vfov, lens):
    w,h=360,270
    rgb=np.empty((h,w,3),np.uint8);rgb[:]=[220,231,231]; depths=np.zeros((h,w),np.float32)
    segmentation=np.zeros_like(rgb);zseg=depths.copy()
    focal=h/(2*np.tan(np.deg2rad(vfov/2)))
    for tri,col,which in [(tt,co,False) for tt,co in head]+[(tt,co,i==3) for i,(tt,co) in enumerate(assembly)]:
        # First-person nominal -Z direction in jaw_soft, +Y up.
        pp=(tri@R.T+OFFSET-lens)*[1,1,-1]
        clipped=near_clip(pp)
        if not len(clipped):continue
        normal=np.cross(clipped[:,1]-clipped[:,0],clipped[:,2]-clipped[:,0])
        level=.65+.34*np.abs(normal@np.array([.32,.81,.49]))/np.maximum(1e-8,np.linalg.norm(normal,axis=1))
        xyz=clipped.copy(); xyz[:,:,2]=1/clipped[:,:,2]
        xyz[:,:,0]=w/2+focal*clipped[:,:,0]*xyz[:,:,2];xyz[:,:,1]=h/2-focal*clipped[:,:,1]*xyz[:,:,2]
        raster(xyz.astype(np.float32),np.uint8(level[:,None]*col),depths,rgb)
        ids=np.zeros((len(xyz),3),np.uint8)
        if which:
            marker=(tips@R.T+OFFSET-lens)*[1,1,-1]
            distance=np.linalg.norm(clipped.mean(1)[:,None,:]-marker[None,:,:],axis=2)
            for idx in range(len(tips)): ids[distance[:,idx]<4.6,0]=idx+1
        raster(xyz.astype(np.float32),ids,zseg,segmentation)
    return Image.fromarray(rgb),[int(np.sum(segmentation[:,:,0]==i+1)) for i in range(len(tips))]


def main():
    p=argparse.ArgumentParser();p.add_argument('runtime',type=Path);args=p.parse_args()
    sim=args.runtime/'assets/sim';out=HERE/'figures';out.mkdir(exist_ok=True)
    head=head_meshes(sim,g['GRASP']);h=t(g['handle']());pi,cap=[t(s) for s in g['retainer']()]
    core=[(h,HANDLE),(pi,PIN),(cap,CAP)]
    tips={n:t(g['tip'](n)) for n in g['TIPS']}
    options={n:core+[(tips[n],TIPCOL[n])] for n in tips}
    eye=[.82,-.9,.46];up=[0,0,1]
    im=Image.new('RGB',(1170,670),bg)
    label(im,(25,12),'Mouth-held keyboard tools',29,True)
    label(im,(25,55),'A shared pad-gripped paddle accepts three rounded TPU tips.',17,False,muted)
    left,_=render(head+options['pointer'],600,306,[.9,-.5,.25],up,focus=[0,-35,16],k=2.86)
    im.paste(left,(0,93))
    right,_=render([(h,HANDLE),(pi+[-13,0,0],PIN),(cap+[9,0,0],CAP),(tips['fork']+[0,-14,0],TIPCOL['fork'])],570,306,[.93,-.5,.82],up,focus=[0,-84,7],k=5.1)
    im.paste(right,(600,93));label(im,(615,104),'Keyed shank and cross-pin',17,True)
    draw=ImageDraw.Draw(im);draw.line((595,98,595,392),fill='#bbc9c3');draw.line((25,400,1145,400),fill='#bbc9c3')
    names=['Round pointer','Twin fork','Curved horn']
    for i,name in enumerate(g['TIPS']):
        part,_=render([(tips[name],TIPCOL[name])],390,148,[.65,-.30,.92],up,focus=[0,-96,10],k=3.4)
        im.paste(part,(390*i,420));label(im,(390*i+24,568),names[i],22,True)
        label(im,(390*i+25,612),['One 3.0 mm radius contact','Two 2.7 mm tips, 23 mm apart','One 3.2 mm radius contact'][i],16,False,muted)
    im.save(out/'mouth-tools.png',optimize=True)
    station=t(g['stand']())
    im=Image.new('RGB',(1150,554),bg)
    label(im,(24,12),'Presenting the tool for pickup',29,True)
    label(im,(26,55),'The stand registers the common paddle with either end free.',17,False,muted)
    panel,_=render([(station,ST)]+options['fork'],540,355,eye,up,focus=[0,-79,-7],k=4.65)
    im.paste(panel,(0,98))
    op=[(x+[0,14,2.2],c) for x,c in head_meshes(sim,g['OPEN'])]
    panel,_=render(op+[(station,ST)]+options['pointer'],610,355,[1,-.21,.18],up,focus=[0,-33,1],k=2.54)
    im.paste(panel,(540,98));draw=ImageDraw.Draw(im);draw.line((540,98,540,464),fill='#bbc9c3')
    label(im,(24,466),'Four support ears seat in open cradles.',18,True)
    label(im,(568,466),'Approach open, close on the pads, then lift.',18,True)
    label(im,(25,513),'Jaw shown at 26° for approach and 4.5° for the modeled grasp. Stand can be mounted on a height-adjusted platform.',16,False,muted)
    im.save(out/'pickup-stand.png',optimize=True)
    xml=ElementTree.parse(sim/'model/robot_walk.xml')
    lens=np.fromstring(xml.find(".//site[@name='head_camera']").get('pos'),sep=' ')*1000
    im=Image.new('RGB',(1130,435),bg)
    label(im,(19,9),'Predicted view from the Duck camera',27,True)
    label(im,(20,48),'Uncalibrated forward-facing lens · 45° vertical field of view',16,False,muted)
    rows={}
    for i,k in enumerate(g['TIPS']):
        counts={}
        for fov in (35,45,55):
            view,num=camera(head,options[k],np.array(g['TIPSITES'][k]),fov,lens)
            counts[str(fov)]=num
            if fov==45: im.paste(view,(12+i*375,85))
        rows[k]=counts;label(im,(15+i*375,371),names[i],19,True)
    label(im,(18,407),'Rendered with head and tool occlusion. Actual mounting angle, lens crop and key visibility need a camera check.',15,False,muted)
    im.save(out/'camera-aim.png',optimize=True)
    (HERE/'camera-results.json').write_text(json.dumps({'resolution_px':[360,270],'visible_tip_pixels':rows},indent=2)+'\n')
    print(rows)
if __name__=='__main__':main()
