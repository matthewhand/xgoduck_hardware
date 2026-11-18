"""Orthographic CAD figure rendering with a triangle depth buffer."""
import math
import numpy as np
from numba import njit
from PIL import Image, ImageDraw, ImageFont
def unit(x):return np.array(x)/np.linalg.norm(x)
S=2;body=[192,197,185];contrast=[72,91,97];teal=[59,161,157];amber=[235,164,62];bg=(247,247,242);ink='#23333a';muted='#506571'
@njit
def raster(t,cols,buf,img):
 H,W=buf.shape
 for n in range(len(t)):
  a,b,c=t[n]; x0=max(0,int(math.floor(min(a[0],b[0],c[0])))); x1=min(W-1,int(math.ceil(max(a[0],b[0],c[0]))))
  y0=max(0,int(math.floor(min(a[1],b[1],c[1])))); y1=min(H-1,int(math.ceil(max(a[1],b[1],c[1]))))
  den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
  if abs(den)<1e-8:continue
  for y in range(y0,y1+1):
   for x in range(x0,x1+1):
    u=((b[1]-c[1])*(x+.5-c[0])+(c[0]-b[0])*(y+.5-c[1]))/den
    v=((c[1]-a[1])*(x+.5-c[0])+(a[0]-c[0])*(y+.5-c[1]))/den
    if min(u,v,1-u-v)<-.00001:continue
    z=c[2]+u*(a[2]-c[2])+v*(b[2]-c[2])
    if z>buf[y,x]:
     buf[y,x]=z
     for ch in range(3):img[y,x,ch]=cols[n,ch]
def render(parts,w,h,eye,up,focus=None,k=None):
 eye=unit(eye);right=unit(np.cross(up,eye));up=unit(np.cross(eye,right));basis=np.array([right,-up,eye]).T
 v=np.concatenate([p.reshape(-1,3)[::11] for p,col in parts])@basis
 if focus is None:
  mid=(v.max(axis=0)+v.min(axis=0))/2
 else:mid=np.array(focus)@basis
 if k is None:k=min((w-65)/np.ptp(v[:,0]),(h-36)/np.ptp(v[:,1]))
 def project(p):
  a=(np.array(p)@basis-mid);a[...,:2]*=k;a[...,:2]+=[w/2,h/2];return a
 img=np.empty((h*S,w*S,3),np.uint8);img[:]=bg;buf=np.full((h*S,w*S),-1e6,np.float32)
 light=unit(eye+up*.75-right*.65)
 for t,col in parts:
  n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]); n/=np.maximum(1e-10,np.linalg.norm(n,axis=1))[:,None]
  # Two-sided shading preserves the appearance of source volumes with reversed axes.
  n[n@eye<0]*=-1
  shade=.52+.46*np.maximum(0,n@light)
  colors=np.uint8(np.clip(shade[:,None]*col,0,255))
  tt=project(t).astype(np.float32);tt[:,:,:2]*=S
  raster(tt,colors,buf,img)
 return Image.fromarray(img).resize((w,h),Image.Resampling.LANCZOS),project
def label(im,pt,s,size,bold=False,col=ink):
 try:
  font=ImageFont.truetype('C:/Windows/Fonts/'+('segoeuib.ttf' if bold else 'segoeui.ttf'),size)
 except OSError:
  font=ImageFont.truetype('DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf',size)
 ImageDraw.Draw(im).text(pt,s,fill=col,font=font)
