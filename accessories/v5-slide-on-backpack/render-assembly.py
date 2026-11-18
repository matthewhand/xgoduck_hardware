from pathlib import Path
import runpy, struct, sys, os, math
from zipfile import ZipFile
from xml.etree import ElementTree as ET
import numpy as np
from numba import njit
from PIL import Image,ImageDraw,ImageFont
import argparse
parser=argparse.ArgumentParser(description="Render the backpack assembly from the exported STLs, upstream assembly and proposed harness.")
here=Path(__file__).resolve().parent
hw=here.parents[1]
parser.add_argument('--runtime',type=Path,default=hw.parent/'xgoduck_runtime_arduino')
parser.add_argument('--assembly',type=Path,required=True,help='Pinned upstream xgoduckV5.3mf')
parser.add_argument('--stls',type=Path,required=True,help='Directory containing the two 0.25 mm top-entry exports')
parser.add_argument('--figures',type=Path,default=here/'figures')
args=parser.parse_args()
rt=args.runtime.resolve()
sys.path.insert(0,str(rt/'.cache/fixed-chair-cad'))
from manifold3d import Manifold
g=runpy.run_path(str(hw/'accessories/v5-slide-on-backpack/charging-pack.py'))
stock=g['stock_enclosure'](); fixed,pack,_,_=g['build'](stock,.25); rails=fixed-stock
out=args.figures;out.mkdir(exist_ok=True,parents=True)
def mesh(s):
 m=s.to_mesh();return m.vert_properties[m.tri_verts,:3].astype(np.float32)
sys.path.insert(0,str(here.parent))
from cad_figure import render, label, body, contrast, teal, amber, bg, ink, muted
def heading(im,i,title,sub):
 label(im,(i*600+25,12),title,28,True);label(im,(i*600+26,54),sub,17,False,muted)
def finish(im,name):
 ImageDraw.Draw(im).line([(600,14),(600,im.height-16)],fill='#c3cdc7')
 im.save(out/name,optimize=True)
def xf(v,values):
 t=np.fromstring(values,sep=' ');return v@t[:9].reshape(3,3)+t[9:]
def assemble():
 z=ZipFile(args.assembly)
 cfg=ET.fromstring(z.read('Metadata/model_settings.config'));root=ET.fromstring(z.read('3D/3dmodel.model'))
 parts=[];reg=None; hips=[]
 for obj in cfg.findall('object'):
  name=obj.find("metadata[@key='name']").get('value');key=obj.get('id')
  its=[i for i in cfg.iter('assemble_item') if i.get('object_id')==key]
  if len(its)!=2:continue
  origin=np.fromstring(its[0].get('transform'),sep=' ')[-3:]
  if not (390<origin[0]<450 and -550<origin[1]<-410) or name=='ENCLOSURE_FOR_V1':continue
  comp=root.find(".//{*}object[@id='%s']"%key).find('.//{*}component')
  fn=next(v for k,v in comp.attrib.items() if k.endswith('}path'))
  data=ET.fromstring(z.read(fn.lstrip('/'))).find(".//{*}object[@id='%s']"%comp.get('objectid'))
  vs=np.array([[float(t.get(a)) for a in 'xyz'] for t in data.findall('.//{*}vertex')])
  ix=np.array([[int(t.get(a)) for a in ['v1','v2','v3']] for t in data.findall('.//{*}triangle')])
  vv=xf(xf(vs,its[1].get('transform')),its[0].get('transform'))
  if name=='ENCLOSURE_FOR_V1P1':
   reg=vv
   continue
  if name=='HIP_YAW': hips.append((vv.min(0)+vv.max(0))*.5)
  if name not in ('BATTERY','BODY_SHELL_LEFT','BODY_SHELL_RIGHT','HIP_YAW_SUPPORT'): continue
  col=body if name.startswith(('BODY','FACE','HEAD_BOTTOM','HIP_ROL_SHELL','SHANK')) else contrast
  if 'TPU' in name:col=[198,158,99]
  if name=='BATTERY':col=[139,154,157]
  parts.append((vv[ix].astype(np.float32),col))
 def rotate(p):return p[...,[0,2,1]]*[1,1,-1]
 # Register via all stock vertices to the enclosure's actual 3MF transform.
 from scipy.spatial import cKDTree
 src=rotate(mesh(stock));shift=reg.min(axis=0)-src.min(axis=(0,1))
 error=cKDTree(reg).query(src.reshape(-1,3)+shift)[0].max()
 assert error<.001,error
 print('Stock registration mm',error,'parts',len(parts),flush=True)
 # Use the walking model to supply the moving head, neck, legs and servo bodies.
 # The V5 3MF has isolated slicing positions for several of those pieces.
 sys.path.append(str(rt/'.cache/qualification-deps'))
 import mujoco as mj
 model=mj.MjModel.from_xml_path(str(rt/'assets/sim/model/robot_walk.xml'));data=mj.MjData(model);mj.mj_forward(model,data)
 moving=[];simhips=[]
 for i in range(model.ngeom):
  if model.geom_group[i]!=2: continue
  idx=model.geom_dataid[i]; name=model.mesh(idx).name
  if name=='trunk_base': continue
  v=model.mesh_vert[model.mesh_vertadr[idx]:model.mesh_vertadr[idx]+model.mesh_vertnum[idx]]@data.geom_xmat[i].reshape(3,3).T+data.geom_xpos[i]
  v=v[:,[1,0,2]]*[1000,1000,-1000]
  if name.endswith('hip_yaw'):simhips.append((v.min(0)+v.max(0))*.5)
  faces=model.mesh_face[model.mesh_faceadr[idx]:model.mesh_faceadr[idx]+model.mesh_facenum[idx]]
  color=body if name in ['head_pitch','right_hip_pitch','left_hip_pitch'] else [162,147,111] if name.endswith('ankle') else contrast
  moving.append((v[faces],color))
 delta=np.mean(hips,axis=0)-np.mean(simhips,axis=0)
 parts.extend([((tris+delta).astype(np.float32),col) for tris,col in moving])
 return parts, lambda pts: (rotate(pts)+shift).astype(np.float32)
parts,transform=assemble()
harness=runpy.run_path(str(hw/'accessories/v5-slide-on-backpack/harness-geometry.py'))
def cable_meshes(park):
 return [(rec,mesh(solid),col) for rec,solid,col in harness['parts'](park)]
route=cable_meshes(0)
def world_harness(items,side=None,delta=(0,0,0)):
 return [(transform(tri)+delta,col) for rec,tri,col in items if side is None or rec['side']==side]
im=Image.new('RGB',(1200,580),bg)
for i,ls in enumerate([[(transform(mesh(stock)),[123,148,151])],[(transform(mesh(fixed)),teal),(transform(mesh(pack)),amber)]]):
 scene=parts+ls+(world_harness(route) if i else [])
 panel,p=render(scene,600,472,[.66,-1,-.24],[0,0,-1],focus=[422,-485,126],k=1.51)
 im.paste(panel,(i*600,88))
heading(im,0,'Before','Upstream V5 · V1.1 enclosure')
heading(im,1,'After','Fork · cover and proposed contact harness')
finish(im,'duck-before-after.png')
# Render directly from inside the left channel and outside the matching fixed rail.
im=Image.new('RGB',(1200,526),bg)
for i,(lst,eye,focus,k) in enumerate([
 ([(mesh(stock),[140,158,159]),(mesh(rails),teal)],[-.56,.81,-1],[-32,-14,2],9.5),
 ([(mesh(pack),amber)],[.63,-1.4,1.3],[-34,-15,.1],10.9)]):
 panel,project=render(lst,600,390,eye,[0,-1,0],focus,k)
 im.paste(panel,(600*i,88))
 def point(v):return tuple(np.round(project(v)[:2]+[i*600,88]).astype(int))
 pts=[([g['LEFT'],-16,0],'A'),([g['LEFT'],-6,0],'B')] if i==0 else [([g['LEFT'],-27,1.3],'C'),([g['LEFT'],-15.5,.45],'D')]
 for v,l in pts:
  x,y=point(v); d=ImageDraw.Draw(im); off=55 if l in ('A','C') else -53
  d.line([(x,y),(x+off,y-20)],fill='#20353c',width=2);d.ellipse((x-3,y-3,x+3,y+3),fill='#20353c');d.ellipse((x+off-12,y-32,x+off+12,y-8),fill=ink);label(im,(x+off-6,y-31),l,15,True,'white')
heading(im,0,'Fixed enclosure','Exterior · left rail enlarged')
heading(im,1,'Removable cover','Interior · looking into the left groove')
label(im,(24,480),'A   Latch notch       B   Dovetail head',18,True)
label(im,(625,480),'C   Groove mouth       D   Ramped latch tooth',18,True)
finish(im,'rail-and-groove.png')

# The moving surfaces below are read back from the exported binary STL files.
export=args.stls
def read_stl(name):
 dt=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
 with (export/name).open('rb') as f:
  f.seek(80); count=struct.unpack('<I',f.read(4))[0]; a=np.fromfile(f,dtype=dt)
 assert len(a)==count
 return a['v'].copy()
fixed_file=read_stl('duck-v5-v1p1-top-entry-rail-enclosure.stl')
cover_file=read_stl('duck-v5-v1p1-top-entry-contact-backpack.stl')+[0,0,g['FACE']]
def compare_vertices(source,expected):
 from scipy.spatial import cKDTree
 a=np.unique(source.reshape(-1,3),axis=0);b=np.unique(expected.reshape(-1,3),axis=0)
 assert max(cKDTree(a).query(b)[0].max(),cKDTree(b).query(a)[0].max())<.001
compare_vertices(fixed_file,mesh(fixed)); compare_vertices(cover_file,mesh(pack))
fw=transform(fixed_file);cw=transform(cover_file)
battery=next((t,c) for t,c in parts if c==[139,154,157])
eye=[.72,-1,-.24];up=[0,0,-1]
whole,_=render(parts+[(fw,teal),(cw,amber)]+world_harness(route),330,370,eye,up,focus=[422,-485,126],k=1.23)
parked_route=cable_meshes(1)
frames=[];times=[]
# At full removal the hand-disconnected carrier half parks outboard of the slide.
schedules=[('Fit the rail enclosure to the battery bracket',x,0,False,1) for x in np.linspace(42,0,12)]
schedules += [('Park the carrier pigtail and align the cover',0,76,True,1)]
schedules += [('Lower the cover with the connector unplugged',0,x,True,1) for x in np.linspace(76,0,24)]
schedules += [('Seat the latch and join the keyed connector',0,0,True,f) for f in np.linspace(1,0,9)]
schedules += [('Seated harness · carrier port position needs measurement',0,0,True,0)]
for i,(title,pull,travel,show,park) in enumerate(schedules):
 im=Image.new('RGB',(940,510),bg)
 im.paste(whole,(0,70))
 group=[battery,(fw+[0,-pull,0],teal)]
 if abs(pull)<.05:
  current=parked_route if park==1 else route if park==0 else cable_meshes(park)
  group+=world_harness(current,'carrier')
 if show:
  group.append((cw+[0,0,-travel],amber))
  group+=world_harness(route,'pack',[0,0,-travel])
 panel,_=render(group,610,398,[.35,-1,-.21],up,focus=[427,-558,117],k=2.02)
 im.paste(panel,(330,59))
 d=ImageDraw.Draw(im);d.line((330,69,330,435),fill='#bdcbc4')
 label(im,(23,12),title,25,True)
 label(im,(24,453),'Installed location',16,True,muted)
 label(im,(353,453),'Teal  enclosure     Ochre  cover',15,True,muted)
 label(im,(354,477),'Proposed hardware: red and blue leads, violet isolator, green connector',15,False,muted)
 frames.append(im);times.append(1500 if i in (0,11,12,36) else 2400 if i==len(schedules)-1 else 100)
# Use one fixed palette for stable colours through motion.
swatch=Image.new('RGB',(235,123*5),bg)
for i,idx in enumerate([0,14,15,25,40]):swatch.paste(frames[idx].resize((235,123)),(0,123*i))
palette=swatch.quantize(colors=224)
frames=[f.quantize(palette=palette,dither=Image.Dither.NONE) for f in frames]
frames[0].save(out/'backpack-assembly.gif',save_all=True,append_images=frames[1:],duration=times,loop=0,optimize=True,disposal=1)

# A cutaway at the headward rim exposes the actual insulated-lead centreline paths.
im=Image.new('RGB',(1200,724),bg)
label(im,(25,13),'How the charging harness leaves the cover',29,True)
label(im,(26,57),'Seated V1.1 layout · dimensioned printed parts and proposed harness envelopes',18,False,muted)
group=[(tri,col) for rec,tri,col in route if rec.get('kind')!='mount']
cut=g['block'](-48,68,26,32.2,-4,7)
# Keep the contact lip, hide the rest of the cover and the carrier tray.
group.insert(0,(mesh(pack^cut),[173,184,181]))
panel,pr=render(group,1200,435,[.07,.16,-1],[0,1,0],focus=[12,32,2],k=9.4)
im.paste(panel,(0,96))
draw=ImageDraw.Draw(im)
def callout(position,letter,offset):
 x,y=pr(position)[:2]+[0,96]
 xx,yy=x+offset[0],y+offset[1]
 draw.line([(x,y),(xx,yy)],fill='#273f47',width=2)
 draw.ellipse((xx-13,yy-13,xx+13,yy+13),fill='#22383d')
 label(im,(xx-5,yy-11),letter,16,True,'white')
for pos,l,off in [([-28,30,-3.3],'1',(8,58)),([19,34,-.9],'2',(15,62)),([-3,43,-3],'3',(40,-55)),([48,40.8,0],'4',(-30,-58)),([61,24,4],'5',(-27,37)),([38,12,13],'6',(67,20))]:
 callout(pos,l,off)
draw.line((25,535,1175,535),fill='#bfccc7')
for (x,y,title,detail) in [(25,549,'1   Folded metal pads','Two separate strips on 56 mm centres.'),(422,549,'2   Insulated pad leads','Each exits through its own underside groove.'),(823,549,'3   Keyed isolator','Both conductors open when away from the dock.'),(25,631,'4   Keyed disconnect','Separate the two halves before lifting the cover.'),(422,631,'5   Carrier service loop','Pigtail parks outside the slide envelope.'),(823,631,'6   Carrier plug envelope','DC1 position and mating depth await measurement.')]:
 label(im,(x,y),title,20,True);label(im,(x,y+33),detail,15,False,muted)
im.save(out/'contact-harness-cutaway.png',optimize=True)
# True orthographic material sections of the nominal seated solids.
im=Image.new('RGB',(1200,365),bg)
heading(im,0,'Across the right rail','Transverse section · cover seated')
heading(im,1,'Through the latch','Longitudinal section · left rail')
def section(solids,crop,matrix,h,origin,zoom,axes):
 draw=ImageDraw.Draw(im)
 for solid,color in solids:
  sec=(solid^crop).transform(matrix).slice(h)
  for path in sec.to_polygons():
   a=np.array(path)
   signed=.5*np.sum(a[:,0]*np.roll(a[:,1],-1)-a[:,1]*np.roll(a[:,0],-1))
   pts=[tuple(axes(v)*zoom+origin) for v in path]
   draw.polygon(pts,fill=color if signed>0 else bg,outline='#34474b',width=1)
section([(stock,'#9ba9a8'),(rails,'#449b97'),(pack,'#e6a349')],g['block'](27,43,-27,0,-4,7.9),
 ((1,0,0,0),(0,0,-1,0),(0,1,0,0)),-18,np.array([296.,149.]),17,lambda v:np.array([v[0]-g['RIGHT'],-v[1]]))
section([(stock,'#9ba9a8'),(rails,'#449b97'),(pack,'#e6a349')],g['block'](-39,-33,-26,-2,-4,4),
 ((0,1,0,0),(0,0,1,0),(1,0,0,0)),g['LEFT'],np.array([902.,204.]),20,lambda v:np.array([v[0]+14.5,v[1]]))
label(im,(24,306),'8.0 mm rail head · 0.25 mm nominal clearance per face',17,True)
label(im,(625,306),'The cover tooth reaches 0.55 mm into the rail notch.',17,True)
finish(im,'seated-sections.png')
