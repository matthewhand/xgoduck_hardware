"""Illustrate the deck, reusable track and mating cartridge from assembly-coordinate meshes."""
from pathlib import Path
import runpy
import sys

from PIL import Image

HERE=Path(__file__).resolve().parent
d=runpy.run_path(str(HERE/'stack.py'))
sys.path.insert(0,str(HERE.parents[1]))
from cad_figure import render, label

def tri(p):
    m=p.to_mesh()
    return m.vert_properties[m.tri_verts,:3].astype(float)

def main():
    deck,cart,pin=d['deck'](),d['cartridge'](),d['pin']()
    sheet=Image.new('RGB',(980,360),(247,247,242))
    for i,spread in enumerate((0,12)):
        parts=[(tri(deck),[98,112,117]),
               (tri(cart.translate((0,spread,0))),[116,170,160]),
               (tri(cart.translate((0,d['STEP']+spread*2,0))),[215,154,91]),
               (tri(pin.translate((spread*.8,0,0))),[81,77,72]),
               (tri(d['pin'](30).translate((spread*.8,d['STEP']+spread,0))),[81,77,72])]
        panel,_=render(parts,490,274,[.78,.85,-1],[0,1,0])
        sheet.paste(panel,(i*490,65))
        label(sheet,(i*490+16,14),['Two seated cartridges','Cartridges and side pins'][i],24,True)
    sheet.save(HERE/'stack.png',optimize=True)
if __name__=='__main__':
    main()
