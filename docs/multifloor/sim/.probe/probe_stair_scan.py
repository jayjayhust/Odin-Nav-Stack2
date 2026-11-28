#!/usr/bin/env python3
"""Why does the stair approach never register the post? Drop the post at
station 2.0 first, then pure-scan from approach poses and split the dynamic
returns by height band: pc_to_scan only keeps dz in [-0.1, +0.3] relative to
the sensor origin (base_z + 0.30)."""
import json
from repro_penetration import session

EXPR = ('(function(){var a=__MESH.pathAt(2.0);'
        'var o=__MESH.drop(a[0],a[1],a[2],false);'
        'var out=[];'
        'for(var s=0.2;s<=2.0;s+=0.3){'
        ' var p=__MESH.pathAt(s), b=__MESH.pathAt(s+0.3);'
        ' var yaw=Math.atan2(b[1]-p[1],b[0]-p[0]);'
        ' var pa=__MESH.pathAt(Math.max(0,s-0.3)), pb=__MESH.pathAt(s+0.3);'
        ' var pitch=Math.atan2(pb[2]-pa[2],Math.hypot(pb[0]-pa[0],pb[1]-pa[1])||1);'
        ' var r=__MESH.scan(p[0],p[1],p[2],yaw,pitch);'
        ' var oz=p[2]+0.30, dyn=r.pts.filter(q=>q.dyn);'
        ' var inb=dyn.filter(q=>{var dz=q.z-oz;return dz>=-0.1&&dz<=0.3;});'
        ' var above=dyn.filter(q=>q.z-oz>0.3).length;'
        ' var below=dyn.filter(q=>q.z-oz<-0.1).length;'
        ' var hor=dyn.filter(q=>{var h=Math.hypot(q.x-p[0],q.y-p[1]);'
        '   return h>=0.2&&h<=5.0;}).length;'
        ' out.push({s:+s.toFixed(1),z:+p[2].toFixed(2),pts:r.pts.length,'
        '  dyn:dyn.length,inBand:inb.length,above:above,below:below,'
        '  inRange:hor,scan:r.scan,mem:r.mem,'
        '  obs:r.obs.map(q=>[+q.x.toFixed(2),+q.y.toFixed(2),+q.r.toFixed(2)])});}'
        'return JSON.stringify({post:[o.x,o.y,o.z,o.r,o.h],rows:out});})()')

setup, samples, _ = session('single,stair,sensor', 2, EXPR, 'JSON.stringify({})')
d = json.loads(setup)
print('post:', [round(v, 2) for v in d['post']])
for row in d['rows']:
    print(json.dumps(row))