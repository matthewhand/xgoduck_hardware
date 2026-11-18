"""Draw a scale section through an inner magnet, the original FACE plane and the target keeper."""
from pathlib import Path
import runpy

import numpy as np
from PIL import Image,ImageDraw,ImageFont

HERE=Path(__file__).resolve().parent
BROW=runpy.run_path(str(HERE/'brow.py'))


def font(px,bold=False):
    for name in (('segoeuib.ttf','DejaVuSans-Bold.ttf') if bold else ('segoeui.ttf','DejaVuSans.ttf')):
        try: return ImageFont.truetype(name,px)
        except OSError: continue
    return ImageFont.load_default()


def section():
    image=Image.new('RGB',(1000,512),'#142028')
    pen=ImageDraw.Draw(image)
    pen.text((24,14),'Hidden magnetic brow mount',font=font(30,True),fill='#f3f7ef')
    pen.text((24,58),'Section through the stock face plane and removable mask foot',font=font(16),fill='#b4c9c6')
    px=55
    panel=(19,90,564,485)
    crop=Image.new('RGB',(panel[2]-panel[0],panel[3]-panel[1]),'#20303a')
    x,y=BROW['POINTS'][0]
    face=BROW['magnetic_face']().transform(((0,-1,0,0),(-1,0,0,33.258),(0,0,-1,0)))
    entries=[(face,'#8299a1'),(BROW['mask_local'](),'#dd9966'),
             (BROW['target_discs'](),'#c6dfe4'),(BROW['mask_magnets'](),'#ffdc78'),
             (BROW['rear_keepers'](),'#62c9b8')]
    for solid,color in entries:
        # Slice 0.15 mm off the centreline, away from coincident mesh edges.
        contours=solid.translate((-x,-y-.15,0)).rotate((90,0,0)).slice(0).to_polygons()
        # XOR nested contours to preserve magnet cavities and rear-tool access.
        alpha=np.zeros((crop.height,crop.width),np.uint8)
        for line in contours:
            coords=[(round(crop.width/2+a*px),round(222+b*px)) for a,b in line]
            single=Image.new('L',crop.size,0)
            ImageDraw.Draw(single).polygon(coords,fill=255)
            alpha^=np.asarray(single)
        crop.paste(color,(0,0,*crop.size),Image.fromarray(alpha))
    image.paste(crop,panel[:2])
    pen.text((31,103),'OUTSIDE',font=font(12,True),fill='#f1f4ee')
    pen.text((31,453),'INSIDE',font=font(12,True),fill='#f1f4ee')
    headers=[('Removable mask foot','#dd9966','0.68 mm printed skin over each magnet'),
             ('Captive magnet','#ffdc78','4 mm diameter × 2 mm disc'),
             ('Unpierced FACE','#8299a1','0.70 mm of original outer skin'),
             ('Steel target','#c6dfe4','4 mm diameter × 0.50 mm mild steel'),
             ('Rear-loading keeper','#62c9b8','Flexible rim seats behind an internal lip')]
    for i,(title,color,detail) in enumerate(headers):
        top=108+72*i
        pen.rounded_rectangle((593,top+3,608,top+18),4,fill=color)
        pen.text((620,top-3),title,fill='#f2f4ec',font=font(19,True))
        pen.text((620,top+27),detail,fill='#c1d1cf',font=font(14))
    image.save(HERE/'magnetic-section.png',optimize=True)


if __name__=='__main__':
    section()
