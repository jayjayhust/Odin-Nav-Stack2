# 复刻 floor_manager/src/map_io.cpp 的解析与栅格化规则，验证行序/阈值是否与 map_server 一致
import math, struct, tempfile, os

def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    # 简化：仅测 P5 / P2 各一例
    tokens = []
    i = 0
    magic = data[0:2].decode()
    i = 2
    def next_int(i):
        while i < len(data) and data[i:i+1].isspace(): i += 1
        if data[i:i+1] == b'#':
            while data[i:i+1] != b'\n': i += 1
            return next_int(i+1)
        j = i
        while j < len(data) and data[j:j+1].isdigit(): j += 1
        return int(data[i:j]), j
    w, i = next_int(i); h, i = next_int(i); mv, i = next_int(i)
    if magic == 'P5':
        i += 1  # 单个分隔符
        px = list(data[i:i+w*h])
    else:
        px = list(map(int, data[i:].split()))
    return w, h, mv, px

def to_grid(w, h, mv, px, negate=False, occ_th=0.65, free_th=0.196):
    data = [-1]*(w*h)
    for y in range(h):
        src_row = h - 1 - y
        for x in range(w):
            pixel = px[src_row*w + x]
            shade = pixel / mv
            occupancy = shade if negate else (1.0 - shade)
            if occupancy > occ_th: cell = 100
            elif occupancy < free_th: cell = 0
            else: cell = -1
            data[y*w + x] = cell
    return data

# 4x3: 四周黑墙，中间两格自由
p5 = b'P5\n4 3\n255\n' + bytes([0,0,0,0, 0,254,254,0, 0,0,0,0])
open('/tmp/t.pgm','wb').write(p5)
w,h,mv,px = read_pgm('/tmp/t.pgm')
g = to_grid(w,h,mv,px)
print("P5 4x3 grid:", [g[i*h:(i+1)*h] if False else g[i*4:(i+1)*4] for i in range(3)])
assert g[0*4+1] == 100 and g[0*4+2] == 100, "底行应为墙"
assert g[2*4+1] == 100 and g[2*4+2] == 100, "顶行应为墙"
assert g[1*4+1] == 0 and g[1*4+2] == 0, "中间应为自由"
print("行翻转 + 阈值: PASS")

# P2 ASCII 同样内容
p2 = b'P2\n# comment\n4 3\n255\n0 0 0 0\n0 254 254 0\n0 0 0 0\n'
open('/tmp/t2.pgm','wb').write(p2)
w,h,mv,px = read_pgm('/tmp/t2.pgm')
g2 = to_grid(w,h,mv,px)
assert g2 == g, "P2 与 P5 结果应一致"
print("P2/P5 一致性: PASS")

# negate: 语义反转，黑变自由
gn = to_grid(w,h,mv,px,negate=True)
assert gn[1*4+1] == 100 and gn[1*4+2] == 100 and gn[0*4+1] == 0
print("negate 语义: PASS")

# origin 解析（与 parseOriginList 相同的容错: 带/不带方括号、逗号或空格分隔）
for s in ["[1.0, -2.0, 0.5]", "1.0 -2.0 0.5", "[1.0, -2.0]"]:
    v = s.strip().lstrip('[').rstrip(']').replace(',',' ').split()
    x,y = float(v[0]), float(v[1]); yaw = float(v[2]) if len(v)>2 else 0.0
    assert (x,y) == (1.0,-2.0)
    qz, qw = math.sin(yaw/2), math.cos(yaw/2)
print("origin/yaw 四元数: PASS", f"qz={qz:.4f} qw={qw:.4f}")
