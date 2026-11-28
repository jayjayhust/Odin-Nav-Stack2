"""Bake the vendor robot models into robot_models.json for mesh_nav.html.

  python3 make_robot_models.py                     # default vendor paths
  python3 make_robot_models.py --go2 DIR --l1 DIR   # other checkouts

The sim needs the dog to look like the real machine while staying a single
portable HTML file (mesh_nav.html is opened from file:// as often as from a
server, and a fetch on file:// fails silently - see the build.py note on the
inlined example OBJs). So the meshes are baked here, at build time, into one
JSON blob that build.py inlines:

  { "<kind>": { "urdf": "<slim URDF text>",
                "meshes": { "<file>": {min:[3], span:[3], v:"<b64 u16>", i:"<b64 idx>"} } } }

The meshes are far too heavy to inline as-is (Go2's DAE set is 25 MB, ZSL-1's
STL set 12 MB, ~150k triangles for the base link alone). Vertex-cluster
decimation on a millimetre grid cuts that by an order of magnitude; the dog is
a display-only body a few dozen pixels tall, so the loss is invisible, while
the joint tree - which decides the silhouette and the trot - is kept exactly.

Collada is converted with the assimp CLI (the DAEs are the only non-STL
inputs); STL is read directly. The emitted URDF is the original minus inertial,
collision, limit and material blocks, with mesh filenames reduced to basenames
so the runtime needs no package:// resolution.
"""
import argparse, base64, json, os, re, shutil, subprocess, sys, tempfile
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RES_3D = '/home/jay/workspace/robotics/3d_resources/urdf'
DEFAULT_GO2 = os.path.join(RES_3D, 'unitree_ros/robots/go2_description')
DEFAULT_L1 = os.path.join(RES_3D, 'genisom_model/zsl-1')

# kind -> (urdf path inside the vendor dir, mesh dir, clustering grid in metres)
MODELS = {
    'go2': ('urdf/go2_description.urdf', 'dae', 0.004),
    'l1':  ('urdf/ZSL-1.urdf', 'meshes', 0.005),
}


def die(msg):
    sys.exit('make_robot_models: ' + msg)


def find_mesh(mesh_dir, filename):
    """Resolve a URDF mesh reference (package://, ../meshes/x.STL, bare name)."""
    base = os.path.basename(filename.replace('\\', '/'))
    for cand in (os.path.join(mesh_dir, base), os.path.join(mesh_dir, base.lower())):
        if os.path.exists(cand):
            return cand
    for root, _, files in os.walk(mesh_dir):
        for f in files:
            if f.lower() == base.lower():
                return os.path.join(root, f)
    return None


def stl_triangles(path):
    """Binary or ascii STL -> (N,3,3) float array."""
    raw = open(path, 'rb').read()
    head = raw[:200].lstrip()
    if head[:5].lower() == b'solid' and b'facet' in raw[:4000]:
        txt = raw.decode('utf-8', 'replace')
        v = np.array(re.findall(r'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', txt),
                     dtype=np.float32)
        return v.reshape(-1, 3, 3)
    n = int(np.frombuffer(raw[80:84], dtype='<u4')[0])
    rec = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
    tri = np.frombuffer(raw[84:84 + 50 * n], dtype=rec)['v']
    return tri.reshape(-1, 3, 3).astype(np.float32)


def dae_triangles(path):
    """Collada -> (N,3,3) float array, back in the URDF frame.

    `assimp export` writes its own Y-up frame: the COLLADA importer applies the
    file's <up_axis>Z_UP</up_axis> conversion (p -> (x, z, -y)) and an OBJ has no
    up axis to convert back from. Without undoing it every leg mesh lies on its
    side - the dog renders as a shell with the limbs thrown forward, which is
    exactly what the first cut of this bake did. The mesh files stay in the
    frame the URDF's <visual><origin> expects, so the inverse is applied here.
    """
    if not shutil.which('assimp'):
        die('assimp CLI not found (needed to convert %s)' % os.path.basename(path))
    tmp = tempfile.mkdtemp(prefix='robotmesh_')
    try:
        obj = os.path.join(tmp, 'm.obj')
        r = subprocess.run(['assimp', 'export', path, obj],
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        # assimp exits 1 on some warnings but still writes the file
        if not os.path.exists(obj):
            die('assimp could not convert %s: %s' % (path, r.stderr.decode()[-300:]))
        verts, tris = [], []
        for line in open(obj, encoding='utf-8'):
            if line.startswith('v '):
                verts.append([float(x) for x in line.split()[1:4]])
            elif line.startswith('f '):
                idx = [int(tok.split('/')[0]) - 1 for tok in line.split()[1:]]
                for k in range(1, len(idx) - 1):        # fan-triangulate
                    tris.append([idx[0], idx[k], idx[k + 1]])
        tri = np.asarray(verts, np.float32)[np.asarray(tris, np.int64)]
        # (x, y, z)_yup -> (x, -z, y)_zup
        return np.stack([tri[..., 0], -tri[..., 2], tri[..., 1]], axis=-1)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def decimate(tri, res):
    """Vertex clustering on a `res` grid, then drop degenerate triangles.

    Every input vertex collapses onto its grid cell; the cell centroid becomes
    the vertex, so the shape stays closed and the triangle count follows the
    surface area instead of the modeller's tessellation.
    """
    P = tri.reshape(-1, 3).astype(np.float64)
    F = np.arange(len(P), dtype=np.int64).reshape(-1, 3)
    q = np.round(P / res).astype(np.int64)
    _, inv, counts = np.unique(q, axis=0, return_inverse=True, return_counts=True)
    centroids = np.zeros((len(counts), 3))
    for k in range(3):
        centroids[:, k] = np.bincount(inv, weights=P[:, k], minlength=len(counts)) / counts
    F = inv[F]
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
    a, b, c = (centroids[F[:, 0]], centroids[F[:, 1]], centroids[F[:, 2]])
    area = np.linalg.norm(np.cross(b - a, c - a), axis=1) * 0.5
    F = F[area > res * res * 0.02]                      # slivers off a crease
    used, F = np.unique(F, return_inverse=True)
    F = F.reshape(-1, 3)
    return centroids[used].astype(np.float32), F.astype(np.int64)


def pack(verts, faces):
    """Quantize to uint16 over the local bbox + base64 both buffers."""
    mn = verts.min(axis=0)
    span = np.maximum(verts.max(axis=0) - mn, 1e-6)
    q = np.round((verts - mn) / span * 65535.0).astype('<u2')
    idx = faces.astype('<u2' if len(verts) < 65536 else '<u4')
    return {'min': [round(float(x), 5) for x in mn],
            'span': [round(float(x), 5) for x in span],
            'v': base64.b64encode(q.tobytes()).decode('ascii'),
            'i': base64.b64encode(idx.tobytes()).decode('ascii'),
            't': int(len(faces))}


def num(el, attr, default='0 0 0'):
    return (el.get(attr) if el is not None else None) or default


def slim_urdf(root, mesh_keys):
    """URDF text with only the kinematic chain that carries a visual mesh.

    Links without geometry are kept when a visual link hangs off them (the
    rotor / calflower shells), because dropping a middle link would break the
    transform chain; everything else - inertial, collision, limits, materials
    the sim does not use - is dropped to keep the inline text small.
    """
    links = {l.get('name'): l for l in root.findall('link')}
    joints = root.findall('joint')
    keep = set()
    for name, l in links.items():
        if l.find('visual/geometry') is not None:
            keep.add(name)
    grew = True
    while grew:                                    # close over the ancestors
        grew = False
        for j in joints:
            p, c = j.find('parent').get('link'), j.find('child').get('link')
            if c in keep and p in links and p not in keep:
                keep.add(p); grew = True
    out = ['<robot name="%s">' % root.get('name')]
    for name in links:
        if name not in keep:
            continue
        l = links[name]
        out.append(' <link name="%s">' % name)
        for vis in l.findall('visual'):
            geo = vis.find('geometry')
            if geo is None:
                continue
            o = vis.find('origin')
            out.append('  <visual><origin xyz="%s" rpy="%s"/><geometry>'
                       % (num(o, 'xyz'), num(o, 'rpy')))
            mesh = geo.find('mesh')
            if mesh is not None and os.path.basename(mesh.get('filename')) in mesh_keys:
                out.append('<mesh filename="%s"/>' % os.path.basename(mesh.get('filename')))
            else:
                for tag in ('box', 'cylinder', 'sphere'):
                    g = geo.find(tag)
                    if g is None:
                        continue
                    attrs = ' '.join('%s="%s"' % (k, v) for k, v in g.attrib.items())
                    out.append('<%s %s/>' % (tag, attrs))
                    break
            out.append('</geometry></visual>')
        out.append('</link>')
    for j in joints:
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        if p not in keep or c not in keep:
            continue
        o = j.find('origin')
        ax = j.find('axis')
        out.append(' <joint name="%s" type="%s"><origin xyz="%s" rpy="%s"/>'
                   % (j.get('name'), j.get('type'), num(o, 'xyz'), num(o, 'rpy')))
        out.append('  <parent link="%s"/><child link="%s"/><axis xyz="%s"/></joint>'
                   % (p, c, num(ax, 'xyz', '1 0 0')))
    out.append('</robot>')
    return '\n'.join(out)


def bake(kind, urdf_path, mesh_dir, res):
    root = ET.parse(urdf_path).getroot()
    refs = []
    for vis in root.iter('visual'):
        mesh = vis.find('geometry/mesh')
        if mesh is not None:
            refs.append(os.path.basename(mesh.get('filename')))
    refs = sorted(set(refs))
    blobs, before, after = {}, 0, 0
    for name in refs:
        path = find_mesh(mesh_dir, name)
        if path is None:
            print('  !! missing mesh %s' % name)
            continue
        tri = dae_triangles(path) if path.lower().endswith('.dae') else stl_triangles(path)
        if not len(tri):
            print('  !! empty mesh %s' % name)
            continue
        verts, faces = decimate(tri, res)
        blobs[name] = pack(verts, faces)
        before += len(tri); after += len(faces)
        print('  %-22s %7d -> %5d tris  %4d verts  %5.1f KB' %
              (name, len(tri), len(faces), len(verts),
               (len(blobs[name]['v']) + len(blobs[name]['i'])) / 1024.0))
    urdf = slim_urdf(root, set(blobs))
    return {'urdf': urdf, 'meshes': blobs}, before, after


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--go2', default=DEFAULT_GO2, help='go2_description package dir')
    ap.add_argument('--l1', default=DEFAULT_L1, help='genisom zsl-1 model dir')
    ap.add_argument('--out', default=os.path.join(HERE, 'robot_models.json'))
    args = ap.parse_args()
    srcs = {'go2': args.go2, 'l1': args.l1}
    out = {}
    for kind, (urdf_rel, mesh_sub, res) in MODELS.items():
        base = srcs[kind]
        urdf_path = os.path.join(base, urdf_rel)
        if not os.path.exists(urdf_path):
            die('URDF not found: %s (pass --%s DIR)' % (urdf_path, kind))
        print('%s <- %s (grid %d mm)' % (kind, urdf_path, res * 1000))
        out[kind], before, after = bake(kind, urdf_path, os.path.join(base, mesh_sub), res)
        print('  total %d -> %d tris' % (before, after))
    tmp = args.out + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, separators=(',', ':'))
    os.replace(tmp, args.out)
    print('wrote %s (%.1f KB)' % (os.path.relpath(args.out, HERE),
                                  os.path.getsize(args.out) / 1024.0))


if __name__ == '__main__':
    main()
