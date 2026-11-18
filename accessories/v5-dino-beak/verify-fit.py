"""Screen side cuffs on stock V5 jaws. This is a rigid geometric fit test."""
from pathlib import Path
import runpy
import struct
import numpy as np

HERE = Path(__file__).resolve().parent
cad = runpy.run_path(str(HERE / 'dino-beak.py'))
read = cad['read_stock']


def check_export(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I',data,80)[0]
    assert len(data) == 84+50*count and count > 0, path.name
    tris = np.frombuffer(data,offset=84,dtype=cad['DTYPE'])['vertex']
    points,ix = np.unique(tris.reshape(-1,3),axis=0,return_inverse=True)
    indices = ix.reshape(-1,3)
    assert np.isfinite(points).all() and abs(float(points[:,2].min()))<.01, path.name
    edges = np.concatenate([indices[:,[0,1]],indices[:,[1,2]],indices[:,[2,0]]])
    _, counts = np.unique(np.sort(edges,axis=1),return_counts=True,axis=0)
    assert np.all(counts==2), path.name
    cad['check_solid'](cad['Manifold'](cad['Mesh'](points,indices.astype(np.uint32))))


def jaw_pose(part, angle):
    # Centre of the V5 horn circle in JAW.stl. Neutral relative placement
    # comes from registering the two stock front lip components in head_roll.
    pivot = np.array([0.,24.05,20.30])
    head_offset = np.array([.00325,-24.89448,-.50])
    return part.translate(-pivot).rotate((angle,0,0)).translate(pivot+head_offset)


def main():
    for path in sorted((HERE/'print').glob('*.stl')):
        check_export(path)
    head,jaw = read('HEAD_BOTTOM'),read('JAW')
    top_pad = read('MOUTH_UP_TPU').rotate((180,0,0)).translate((0,-43.840,11))
    bottom_pad = read('MOUTH_BOTTOM_TPU').translate((0,-15.54,2.8))
    face = read('FACE').transform(((0,-1,0,0),(0,0,1,-54.848),(-1,0,0,33.258)))
    tof = read('FACE_TOF').transform(((1,0,0,24.693),(0,0,-1,-48.048),(0,1,0,30.627)))
    upper = cad['build'](head,'upper',teeth=True,tusks=True)
    lower = cad['build'](jaw,'lower',teeth=True)
    for side,cuff,stock,pad in [('upper',upper,head,top_pad),('lower',lower,jaw,bottom_pad)]:
        interference = cuff ^ stock
        active = [s for s in interference.decompose() if s.volume() > 1e-5]
        assert len(active)==4 and all(abs(s.bounding_box()[0])>42 for s in active)
        assert (cuff ^ pad).volume()<1e-6
        print(f'{side} grip pads: {interference.volume():.3f} mm3 interference in four patches')
    minimum = np.full(5,5.0)
    fixed = [upper,upper,upper,head,top_pad]
    for angle in np.arange(0,30.01,.25):
        moving = [jaw_pose(part,angle) for part in (lower,jaw,bottom_pad)]
        opposing = [*moving,moving[0],moving[0]]
        minimum = np.minimum(minimum,[a.min_gap(b,5.0) for a,b in zip(fixed,opposing)])
    assert np.all(minimum > 0), minimum
    print('0..30 degrees in 0.25-degree steps, minimum mm (capped at 5):')
    for title,gap in zip(['upper/lower cuffs','upper cuff/jaw','upper cuff/lower pad',
                          'lower cuff/head','lower cuff/upper pad'],minimum):
        print(f'  {title}: {gap:.3f}')
    for title,part in [('face shell',face),('ToF housing',tof)]:
        gap = upper.min_gap(part,50)
        assert gap > 0
        print(f'Upper teeth+tusks to {title}: {gap:.2f} mm')


if __name__=='__main__':
    main()
