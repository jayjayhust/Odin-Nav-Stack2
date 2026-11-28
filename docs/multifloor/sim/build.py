"""Build mesh_nav.html from the template plus the bundled assets.

  python3 build.py [--profile ../../../robots/go2/neupan/config.yaml]

--profile injects stuck_escape.route_rotation from a robot's NeuPAN config as
window.__ESCAPE_CFG, so the page's stuck-escape decisions are tuned by the same
file the robot uses (see docs/multifloor/escape-exploration.md). Parsing is
hand-rolled for that one sub-map on purpose - no PyYAML dependency here, and
the smaller the parsed surface the less the build can disagree with what
NeuPAN's yaml.safe_load actually sees.
"""
import glob, json, os, re, sys

ROOT = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PROFILE = os.path.normpath(os.path.join(ROOT, '..', '..', '..',
                                                'robots', 'go2', 'neupan', 'config.yaml'))


def profile_path(argv):
    for i, a in enumerate(argv):
        if a == '--profile':
            if i + 1 >= len(argv):
                sys.exit('--profile needs a path')
            return os.path.abspath(argv[i + 1])
        if a.startswith('--profile='):
            return os.path.abspath(a.split('=', 1)[1])
    return DEFAULT_PROFILE


def _scalar(tok):
    tok = tok.strip()
    if tok in ('true', 'True'):
        return True
    if tok in ('false', 'False'):
        return False
    try:
        return int(tok)
    except ValueError:
        pass
    try:
        return float(tok)
    except ValueError:
        pass
    return tok.strip('\'"')


def read_route_rotation(path):
    """stuck_escape.route_rotation as a dict, or {} when absent."""
    if not os.path.exists(path):
        sys.exit('profile not found: ' + path)
    out, section, block = {}, None, False
    for raw in open(path, encoding='utf-8'):
        ln = raw.split('#', 1)[0].rstrip() if not raw.lstrip().startswith('#') else ''
        if not ln.strip():
            continue
        indent = len(ln) - len(ln.lstrip())
        key = ln.strip().split(':', 1)[0]
        if indent == 0:
            section, block = key, False
            continue
        if section == 'stuck_escape' and indent == 2 and key == 'route_rotation':
            block = True
            continue
        if not block:
            continue
        if indent <= 2:
            block = False
            continue
        if indent != 4 or ':' not in ln.strip():
            continue
        val = ln.strip().split(':', 1)[1].strip()
        if val.startswith('['):
            out[key] = [_scalar(v) for v in re.findall(r'[^,\[\]]+', val)]
        else:
            out[key] = _scalar(val)
    return out


tpl_p = os.path.join(ROOT, "mesh_nav.template.html")
out_p = os.path.join(ROOT, "mesh_nav.html")
tpl = open(tpl_p, encoding='utf-8').read()
three = open(os.path.join(ROOT, "three.min.js"), encoding='utf-8').read()
obj_four = json.dumps(open(os.path.join(ROOT, "example_four_floor.obj")).read())
obj_room = json.dumps(open(os.path.join(ROOT, "example_single_floor.obj")).read())
obj_terrain = json.dumps(open(os.path.join(ROOT, "example_multi_terrain.obj")).read())
# The inspection example is inlined, not fetched: the page is opened straight
# from disk as often as from a server, and a fetch on file:// fails silently.
# json.loads also rejects a hand-edit that is not valid JSON, at build time.
insp_task = json.dumps(json.load(open(os.path.join(ROOT, "inspection_task.example.json"),
                                       encoding='utf-8')), ensure_ascii=False)
files = sorted(glob.glob(os.path.join(ROOT, "recognizers/*.js")))
files = [f for f in files if f.endswith("terrain.js")] + [f for f in files if not f.endswith("terrain.js")]
rec = "\n".join(open(f, encoding='utf-8').read() for f in files)
# The robot dog's vendor meshes (宇树 Go2, 智身 钢镚 L1) are baked by
# make_robot_models.py and inlined for the same reason as the example OBJs: the
# page is opened from file:// as often as from a server, and a mesh fetch on
# file:// fails silently. A missing file is a build error, not a silent fallback
# - the page would quietly show the procedural body instead.
robots_p = os.path.join(ROOT, "robot_models.json")
if not os.path.exists(robots_p):
    sys.exit('robot_models.json missing - run make_robot_models.py first')
robots = json.dumps(json.load(open(robots_p, encoding='utf-8')), separators=(',', ':'))
assert tpl.count("/*__ROBOT_MODELS__*/null") == 1
tpl = tpl.replace("/*__ROBOT_MODELS__*/null", robots, 1)
assert tpl.count("/*__RECOGNIZERS__*/") == 1
out = tpl.replace("/*__RECOGNIZERS__*/", rec, 1)
out = out.replace("/*__THREE__*/", three, 1)
out = out.replace("/*__DATA_ROOM__*/", obj_room, 1)
out = out.replace("/*__DATA_TERRAIN__*/", obj_terrain, 1)
out = out.replace("/*__DATA_FOUR__*/", obj_four, 1)
assert out.count("/*__INSPECTION_TASK__*/") == 1
out = out.replace("/*__INSPECTION_TASK__*/", insp_task, 1)

# escape route_rotation -> window.__ESCAPE_CFG (the contract reads it once, at
# definition; keys the contract does not know are reported, not ignored, so a
# typo in the profile cannot silently ship as "the sim ignores my tuning").
prof = profile_path(sys.argv[1:])
cfg = read_route_rotation(prof)
assert out.count("/*__ESCAPE_CFG__*/") == 1
if cfg:
    known = set(re.findall(r'^  ([a-z_]+):', out.split('const ESCP = Object.assign({', 1)[1]
                           .split('}, (typeof window', 1)[0], re.M))
    dropped = sorted(set(cfg) - known)
    if dropped:
        print('warning: %s keys the sim contract does not read: %s'
              % (os.path.relpath(prof, ROOT), ', '.join(dropped)))
    keep = {k: v for k, v in cfg.items() if k in known}
    inj = 'window.__ESCAPE_CFG = ' + json.dumps(keep) + ';'
else:
    inj = ''
out = out.replace("/*__ESCAPE_CFG__*/", inj, 1)
open(out_p, 'w', encoding='utf-8').write(out)
print('built', len(out), '| profile', os.path.relpath(prof, ROOT),
      '| route_rotation', len(cfg), 'keys,', len(inj), 'chars injected')
