"""Render the interior hatch and a clipped vertical section through the fitted right target."""
from pathlib import Path
import runpy
import sys

from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]))
from cad_figure import render,label,bg

cad=runpy.run_path(str(HERE/'chin.py'))
parts=cad['build']()
box=cad['box']
substrate=[201,195,169]
keeper=[69,158,152]
steel=[106,119,125]
plum=[164,107,125]
magnet=[212,157,68]


def triangles(p):
    m=p.to_mesh()
    return m.vert_properties[m.tri_verts,:3].astype(float)


def plate(items,eye,focus,scale):
    return render([(triangles(p),col) for p,col in items],540,350,eye,[0,0,1],focus,scale)[0]


sheet=Image.new('RGB',(1080,485),bg)
inner=parts['host'] ^ box(-22,22,-26,-7,-.1,3.1)
exploded=[(inner,substrate),(parts['steel'].translate((0,0,3.8)),steel),
          (parts['lid'].translate((0,3.6,10)),keeper)]
sheet.paste(plate(exploded,(.63,.70,.68),(0,-17,5.3),9.8),(0,40))
section=box(3,10.,-26,-8,-17,4)
items=[(parts[k]^section,c) for k,c in [('host',substrate),('steel',steel),('lid',keeper),
                                         ('beard',plum),('magnets',magnet)]]
sheet.paste(plate(items,(1,.07,.02),(9,-17.2,-2.5),23),(540,40))
for x,title,desc in [(20,'Inside the jaw','Hatch lifted and targets raised to show the wells.'),
                     (560,'Section through one target','The stock chin skin and beard skin separate steel and magnet.')]:
    label(sheet,(x,9),title,24,True)
    label(sheet,(x,386),desc,15)
for x,title,col in [(20,'Jaw',substrate),(120,'Hatch',keeper),(240,'Steel target',steel),(415,'Beard',plum),(525,'Sealed magnet',magnet)]:
    ImageDraw.Draw(sheet).rounded_rectangle((x,443,x+20,460),radius=3,fill=tuple(col))
    label(sheet,(x+27,438),title,16)
sheet.save(HERE/'chin-hatch.png',optimize=True)
