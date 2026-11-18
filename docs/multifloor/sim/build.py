import json, sys
tpl_p = "mesh_nav.template.html"
out_p = "mesh_nav.html"
tpl = open(tpl_p).read()
three = open("three.min.js").read()
obj1 = json.dumps(open("example_single_floor.obj").read())
obj2 = json.dumps(open("example_single_floor_25cm.obj").read())
obj3 = json.dumps(open("example_four_floor.obj").read())
import glob
files = sorted(glob.glob("recognizers/*.js"))
files = [f for f in files if f.endswith("terrain.js")] + [f for f in files if not f.endswith("terrain.js")]
rec = "\n".join(open(f).read() for f in files)
assert tpl.count("/*__RECOGNIZERS__*/") == 1
out = tpl.replace("/*__RECOGNIZERS__*/", rec, 1)
out = out.replace("/*__THREE__*/", three, 1)
out = out.replace("/*__OBJ2__*/", obj1, 1)
out = out.replace("/*__OBJ3__*/", obj2, 1)
out = out.replace("/*__OBJ__*/", obj3, 1)
open(out_p, "w").write(out)
print("built", len(out))
