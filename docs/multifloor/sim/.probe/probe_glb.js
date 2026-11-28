'use strict';

// Regression for the offline GLB importer. The parser is sliced from the page so
// this exercises the shipped implementation, not a second decoder in the test.
const assert = require('assert');
const fs = require('fs');
const path = require('path');

const base = path.resolve(__dirname, '..');
const tpl = fs.readFileSync(path.join(base, 'mesh_nav.template.html'), 'utf8');
const between = (a, b) => {
  const i = tpl.indexOf(a), j = tpl.indexOf(b, i);
  if (i < 0 || j < 0) throw new Error('slice not found: ' + a);
  return tpl.slice(i, j);
};
const parseGLB = (0, eval)(between('function parseGLB(file){',
  '// ---------- map: surface voxels') + '\n;parseGLB;');
const parseOBJ = (0, eval)(between('function parseOBJ(text){',
  'function parseSTL(buf){') + '\n;parseOBJ;');

const JSON_CHUNK = 0x4e4f534a, BIN_CHUNK = 0x004e4942;
const pad4 = n => (4 - (n & 3)) & 3;
const asBuffer = value => {
  if (Buffer.isBuffer(value)) return value;
  if (value instanceof ArrayBuffer) return Buffer.from(value);
  return Buffer.from(value.buffer, value.byteOffset, value.byteLength);
};

class BinBuilder {
  constructor(){ this.parts = []; this.length = 0; }
  add(value, align = 4){
    const data = Buffer.from(asBuffer(value));
    const padding = (align - (this.length % align)) % align;
    if (padding) this.parts.push(Buffer.alloc(padding));
    const offset = this.length + padding;
    this.parts.push(data);
    this.length = offset + data.length;
    return offset;
  }
  bytes(){ return Buffer.concat(this.parts, this.length); }
}

function chunk(type, data){
  const body = Buffer.from(data);
  return Buffer.concat([
    (() => { const h = Buffer.alloc(8); h.writeUInt32LE(body.length, 0); h.writeUInt32LE(type, 4); return h; })(),
    body,
    Buffer.alloc(pad4(body.length)),
  ]);
}

function packChunks(chunks){
  const body = Buffer.concat(chunks), out = Buffer.alloc(12 + body.length);
  out.writeUInt32LE(0x46546c67, 0);
  out.writeUInt32LE(2, 4);
  out.writeUInt32LE(out.length, 8);
  body.copy(out, 12);
  return out;
}

function makeGlb(doc, bin, jsonText){
  const json = Buffer.from(jsonText === undefined ? JSON.stringify(doc) : jsonText, 'utf8');
  const chunks = [chunk(JSON_CHUNK, json)];
  if (bin !== undefined && bin !== null) chunks.push(chunk(BIN_CHUNK, asBuffer(bin)));
  return packChunks(chunks);
}

const view = (buffer, byteOffset, byteLength, extra) =>
  Object.assign({buffer, byteOffset, byteLength}, extra || {});
const posAccessor = (bufferView, count, extra) =>
  Object.assign({bufferView, componentType: 5126, count, type: 'VEC3'}, extra || {});
const idxAccessor = (bufferView, count, componentType = 5125) =>
  ({bufferView, componentType, count, type: 'SCALAR'});

function triangleFixture(){
  const bin = new BinBuilder();
  const positions = new Float32Array([0,0,0, 1,0,0, 0,1,0]);
  const indices = new Uint16Array([0,1,2]);
  const po = bin.add(positions), io = bin.add(indices);
  const doc = {
    asset: {version: '2.0'},
    buffers: [{byteLength: bin.length}],
    bufferViews: [view(0, po, positions.byteLength), view(0, io, indices.byteLength)],
    accessors: [posAccessor(0, 3), idxAccessor(1, 3, 5123)],
    meshes: [{primitives: [{attributes: {POSITION: 0}, indices: 1}]}],
    nodes: [{mesh: 0}],
    scenes: [{nodes: [0]}],
    scene: 0,
  };
  return {doc, bin, buf: makeGlb(doc, bin.bytes())};
}

function fixtureFromGeometry(positions, indices, indexType = 5125){
  const bin = new BinBuilder();
  const po = bin.add(positions), io = bin.add(indices);
  const doc = {
    asset: {version: '2.0'},
    buffers: [{byteLength: bin.length}],
    bufferViews: [view(0, po, positions.byteLength), view(0, io, indices.byteLength)],
    accessors: [posAccessor(0, positions.length / 3),
                idxAccessor(1, indices.length, indexType)],
    meshes: [{primitives: [{attributes: {POSITION: 0}, indices: 1}]}],
    nodes: [{mesh: 0}],
    scenes: [{nodes: [0]}],
    scene: 0,
  };
  return {doc, bin, buf: makeGlb(doc, bin.bytes())};
}

let checks = 0;
const ok = (name, fn) => { fn(); ++checks; console.log('ok ' + checks + ' - ' + name); };
const closeArray = (actual, expected, eps = 1e-6) => {
  assert.strictEqual(actual.length, expected.length);
  for (let i = 0; i < actual.length; ++i)
    assert.ok(Math.abs(actual[i] - expected[i]) <= eps,
      'value ' + i + ': ' + actual[i] + ' != ' + expected[i]);
};
const rejects = (name, buf, pattern = /GLB/) => ok(name, () => {
  assert.throws(() => parseGLB(buf), pattern);
});
const clone = value => JSON.parse(JSON.stringify(value));

ok('file picker advertises and dispatches GLB', () => {
  assert.match(tpl, /accept="\.obj,\.stl,\.glb,\.txt"/);
  assert.match(tpl, /if \(glb\) o = parseGLB\(r\.result\);/);
  assert.match(tpl, /r\.readAsArrayBuffer\(f\)/);
});

ok('imports default to Y-up while bundled demos force Z-up', () => {
  assert.match(tpl, /<select id="up"><option value="z">Z 向上<\/option><option value="y" selected>/);
  assert.strictEqual((tpl.match(/document\.getElementById\('up'\)\.value = 'z';/g) || []).length, 3);
});

ok('indexed embedded-buffer triangle', () => {
  const out = parseGLB(triangleFixture().buf);
  assert.deepStrictEqual(Array.from(out.pos), [0,0,0, 1,0,0, 0,1,0]);
  assert.deepStrictEqual(Array.from(out.idx), [0,1,2]);
  assert.deepStrictEqual(out.stats, {nodes: 1, meshes: 1, primitives: 1, triangles: 1});
});

ok('default scene selection and non-indexed triangles', () => {
  const bin = new BinBuilder();
  const positions = new Float32Array([0,0,0, 1,0,0, 0,1,0]);
  const po = bin.add(positions);
  const doc = {
    asset: {version: '2.0'}, buffers: [{byteLength: bin.length}],
    bufferViews: [view(0, po, positions.byteLength)],
    accessors: [posAccessor(0, 3)],
    meshes: [
      {primitives: [{mode: 0, attributes: {POSITION: 0}}]},
      {primitives: [{attributes: {POSITION: 0}}]},
    ],
    nodes: [{mesh: 0}, {mesh: 1, translation: [5,0,0]}],
    scenes: [{nodes: [0]}, {nodes: [1]}], scene: 1,
  };
  const out = parseGLB(makeGlb(doc, bin.bytes()));
  assert.deepStrictEqual(Array.from(out.pos), [5,0,0, 6,0,0, 5,1,0]);
  assert.deepStrictEqual(Array.from(out.idx), [0,1,2]);
  assert.strictEqual(out.stats.nodes, 1);
});

ok('TRS/matrix hierarchy and mirrored winding', () => {
  const f = triangleFixture(), doc = clone(f.doc);
  doc.nodes = [
    {translation: [1,2,3], rotation: [0,0,Math.SQRT1_2,Math.SQRT1_2], scale: [2,3,4],
     children: [1]},
    {mesh: 0, matrix: [-1,0,0,0, 0,1,0,0, 0,0,1,0, 0.5,0,0,1]},
  ];
  doc.scenes = [{nodes: [0]}];
  const out = parseGLB(makeGlb(doc, f.bin.bytes()));
  closeArray(out.pos, [1,3,3, 1,1,3, -2,3,3]);
  assert.deepStrictEqual(Array.from(out.idx), [0,2,1]);
  assert.strictEqual(out.stats.nodes, 2);
});

ok('triangle strips, fans, and multiple primitives', () => {
  const bin = new BinBuilder();
  const strip = new Float32Array([0,0,0, 1,0,0, 0,1,0, 1,1,0]);
  const stripIdx = new Uint8Array([0,1,2,3]);
  const fan = new Float32Array([0,0,0, 1,0,0, 1,1,0, 0,1,0, -1,1,0]);
  const so = bin.add(strip), sio = bin.add(stripIdx), fo = bin.add(fan);
  const doc = {
    asset: {version: '2.0'}, buffers: [{byteLength: bin.length}],
    bufferViews: [view(0, so, strip.byteLength), view(0, sio, stripIdx.byteLength),
                  view(0, fo, fan.byteLength)],
    accessors: [posAccessor(0, 4), idxAccessor(1, 4, 5121), posAccessor(2, 5)],
    meshes: [{primitives: [
      {attributes: {POSITION: 0}, indices: 1, mode: 5},
      {attributes: {POSITION: 2}, mode: 6},
    ]}],
    nodes: [{mesh: 0}], scenes: [{nodes: [0]}], scene: 0,
  };
  const out = parseGLB(makeGlb(doc, bin.bytes()));
  assert.deepStrictEqual(Array.from(out.idx),
    [0,1,2, 2,1,3, 4,5,6, 4,6,7, 4,7,8]);
  assert.strictEqual(out.stats.triangles, 5);
  assert.strictEqual(out.stats.primitives, 2);
});

ok('interleaved positions with view and accessor offsets', () => {
  const bin = new BinBuilder();
  // Three 16-byte elements: [x, y, z, pad], behind an 8-byte prefix. The
  // bufferView and accessor each skip four bytes, so both offsets are exercised.
  const raw = new Float32Array([
    111, 222,
    0,0,0, 999,
    1,0,0, 999,
    0,1,0, 999,
  ]);
  const indices = new Uint16Array([0,1,2]);
  bin.add(raw);
  const io = bin.add(indices);
  const doc = {
    asset: {version: '2.0'}, buffers: [{byteLength: bin.length}],
    bufferViews: [view(0, 4, 48, {byteStride: 16}), view(0, io, indices.byteLength)],
    accessors: [posAccessor(0, 3, {byteOffset: 4}), idxAccessor(1, 3, 5123)],
    meshes: [{primitives: [{attributes: {POSITION: 0}, indices: 1}]}],
    nodes: [{mesh: 0}], scenes: [{nodes: [0]}], scene: 0,
  };
  const out = parseGLB(makeGlb(doc, bin.bytes()));
  assert.deepStrictEqual(Array.from(out.pos), [0,0,0, 1,0,0, 0,1,0]);
});

ok('sparse position accessor', () => {
  const bin = new BinBuilder();
  const sparseIndices = new Uint8Array([0,1,2]);
  const sparseValues = new Float32Array([99, 1,0,0, 2,0,0, 3,0,0]);
  const io = bin.add(sparseIndices), vo = bin.add(sparseValues);
  const doc = {
    asset: {version: '2.0'}, buffers: [{byteLength: bin.length}],
    bufferViews: [view(0, io, sparseIndices.byteLength),
                  view(0, vo, sparseValues.byteLength)],
    accessors: [{
      componentType: 5126, count: 3, type: 'VEC3',
      sparse: {count: 3,
        indices: {bufferView: 0, componentType: 5121},
        values: {bufferView: 1, byteOffset: 4}},
    }],
    meshes: [{primitives: [{attributes: {POSITION: 0}}]}],
    nodes: [{mesh: 0}], scenes: [{nodes: [0]}], scene: 0,
  };
  const out = parseGLB(makeGlb(doc, bin.bytes()));
  assert.deepStrictEqual(Array.from(out.pos), [1,0,0, 2,0,0, 3,0,0]);
});

ok('base64 data-URI buffer and typed-array subrange', () => {
  const f = triangleFixture(), doc = clone(f.doc);
  doc.buffers[0].uri = 'data:application/octet-stream;base64,' + f.bin.bytes().toString('base64');
  const packed = makeGlb(doc, null);
  const carrier = Buffer.concat([Buffer.alloc(7), packed, Buffer.alloc(5)]);
  const input = new Uint8Array(carrier.buffer, carrier.byteOffset + 7, packed.length);
  const out = parseGLB(input);
  assert.deepStrictEqual(Array.from(out.idx), [0,1,2]);
  assert.strictEqual(out.pos.length, 9);
});

const basic = triangleFixture();
rejects('bad GLB magic', (() => { const b = Buffer.from(basic.buf); b.writeUInt32LE(0, 0); return b; })());
rejects('unsupported glTF version', (() => { const b = Buffer.from(basic.buf); b.writeUInt32LE(1, 4); return b; })());
rejects('truncated file', basic.buf.subarray(0, basic.buf.length - 1));
rejects('invalid JSON chunk', makeGlb(null, null, '{'));
rejects('JSON document must be an object', makeGlb(null, null, 'null'), /GLB/);
rejects('duplicate JSON chunks', packChunks([
  chunk(JSON_CHUNK, JSON.stringify(basic.doc)), chunk(JSON_CHUNK, JSON.stringify(basic.doc)),
]));
rejects('external buffers are rejected', (() => {
  const doc = clone(basic.doc); doc.buffers[0].uri = 'mesh.bin'; return makeGlb(doc, basic.bin.bytes());
})());
for (const ext of ['KHR_draco_mesh_compression', 'EXT_meshopt_compression', 'KHR_mesh_quantization'])
  rejects('required extension rejected: ' + ext, (() => {
    const doc = clone(basic.doc); doc.extensionsRequired = [ext]; return makeGlb(doc, basic.bin.bytes());
  })());
rejects('extensionsRequired must be an array', (() => {
  const doc = clone(basic.doc); doc.extensionsRequired = {}; return makeGlb(doc, basic.bin.bytes());
})());
rejects('compressed primitive rejected', (() => {
  const doc = clone(basic.doc);
  doc.meshes[0].primitives[0].extensions = {KHR_draco_mesh_compression: {bufferView: 0}};
  return makeGlb(doc, basic.bin.bytes());
})());
rejects('meshopt buffer view rejected', (() => {
  const doc = clone(basic.doc);
  doc.bufferViews[0].extensions = {EXT_meshopt_compression: {buffer: 0}};
  return makeGlb(doc, basic.bin.bytes());
})());
rejects('skinned mesh rejected', (() => {
  const doc = clone(basic.doc); doc.nodes[0].skin = 0; return makeGlb(doc, basic.bin.bytes());
})());
rejects('morph target rejected', (() => {
  const doc = clone(basic.doc); doc.meshes[0].primitives[0].targets = [{}];
  return makeGlb(doc, basic.bin.bytes());
})());
rejects('GPU instancing rejected', (() => {
  const doc = clone(basic.doc); doc.nodes[0].extensions = {EXT_mesh_gpu_instancing: {}};
  return makeGlb(doc, basic.bin.bytes());
})());
rejects('out-of-range triangle index', (() => {
  const f = fixtureFromGeometry(new Float32Array([0,0,0, 1,0,0, 0,1,0]),
                                new Uint16Array([0,1,3]), 5123);
  return f.buf;
})());
rejects('POSITION must remain Float32 VEC3', (() => {
  const doc = clone(basic.doc); doc.accessors[0].componentType = 5122;
  return makeGlb(doc, basic.bin.bytes());
})());
rejects('point-only scene has no triangles', (() => {
  const doc = clone(basic.doc); doc.meshes[0].primitives[0].mode = 0;
  return makeGlb(doc, basic.bin.bytes());
})());
rejects('cyclic node hierarchy rejected', (() => {
  const doc = clone(basic.doc);
  doc.nodes = [{mesh: 0, children: [1]}, {children: [0]}];
  doc.scenes = [{nodes: [0]}];
  return makeGlb(doc, basic.bin.bytes());
})());
rejects('invalid node.children rejected', (() => {
  const doc = clone(basic.doc); doc.nodes[0].children = 'bad';
  return makeGlb(doc, basic.bin.bytes());
})());
rejects('accessor stride smaller than an element rejected', (() => {
  const doc = clone(basic.doc); doc.bufferViews[0].byteStride = 8;
  return makeGlb(doc, basic.bin.bytes());
})());
rejects('invalid accessor index rejected', (() => {
  const doc = clone(basic.doc); doc.meshes[0].primitives[0].attributes.POSITION = 99;
  return makeGlb(doc, basic.bin.bytes());
})());
rejects('malformed sparse index rejected', (() => {
  const bin = new BinBuilder();
  const si = new Uint8Array([3]), sv = new Float32Array([1,0,0]);
  const sio = bin.add(si), svo = bin.add(sv);
  const doc = {
    asset: {version: '2.0'}, buffers: [{byteLength: bin.length}],
    bufferViews: [view(0, sio, si.byteLength), view(0, svo, sv.byteLength)],
    accessors: [{componentType: 5126, count: 3, type: 'VEC3',
      sparse: {count: 1, indices: {bufferView: 0, componentType: 5121},
               values: {bufferView: 1}}}],
    meshes: [{primitives: [{attributes: {POSITION: 0}}]}],
    nodes: [{mesh: 0}], scenes: [{nodes: [0]}], scene: 0,
  };
  return makeGlb(doc, bin.bytes());
})());

// Planner parity: the same single-floor OBJ, once as OBJ and once as GLB.
const objText = fs.readFileSync(path.join(base, 'example_single_floor.obj'), 'utf8');
const raw = parseOBJ(objText);
const parity = fixtureFromGeometry(raw.pos, raw.idx, 5125);
const decoded = parseGLB(parity.buf);
ok('OBJ geometry round-trips through GLB exactly', () => {
  assert.deepStrictEqual(decoded.pos, raw.pos);
  assert.deepStrictEqual(decoded.idx, raw.idx);
});

global.window = global;
global.location = {hash: ''};
global.document = {getElementById: () => ({value: '0.45'})};
global.state = {stairOpen: 1, climbMid: 0.35, pipe: 0.60, bandSoft: 0, foot: 0.18,
  stairDecouple: 1, erased: new Set(), added: new Set(), globalMergePosts: false};
const recognizers = fs.readFileSync(path.join(base, 'recognizers/terrain.js'), 'utf8') + '\n' +
                    fs.readFileSync(path.join(base, 'recognizers/stairs.js'), 'utf8');
const engine = (0, eval)(recognizers + '\n' +
  between('const UNKNOWN = 0', '// ---------- 3D A*') + '\n' +
  between('function edgeRoom(m, id){', '// Why a cell is or is not usable right now') + '\n' +
  between('// ---------- 3D A*', '// A* walks the lattice') + '\n' +
  between('function flood(m, x, y, z){', '// ---------- obstacles') +
  '\n;({buildMap, planPath});');

let parityMapObj, parityMapGlb, parityPlanObj, parityPlanGlb;
ok('OBJ and GLB produce identical voxel maps and routes', () => {
  parityMapObj = engine.buildMap(raw.pos, raw.idx, 0.15, 0.30, false);
  parityMapGlb = engine.buildMap(decoded.pos, decoded.idx, 0.15, 0.30, false);
  for (const key of ['res', 'infR', 'lo', 'nx', 'ny', 'nz', 'cells', 'nStand', 'nInfl',
                     'nStair', 'zmin', 'zmax'])
    assert.deepStrictEqual(parityMapGlb[key], parityMapObj[key], key);
  for (const key of ['data', 'topFace', 'steep', 'capWide', 'stairCol', 'riseMask',
                     'stairProfile', 'stairRamp', 'stairCorr', 'treadTop'])
    assert.deepStrictEqual(parityMapGlb[key], parityMapObj[key], key);
  assert.strictEqual(parityMapGlb.flights.length, parityMapObj.flights.length);

  // Endpoints already used by the mesh no-go regression; do not invent a new
  // pair that might be unreachable for reasons unrelated to the importer.
  const start = [2.53,7.40,0.13], goal = [7.82,4.13,0.13];
  parityPlanObj = engine.planPath(parityMapObj, ...start, ...goal, null);
  parityPlanGlb = engine.planPath(parityMapGlb, ...start, ...goal, null);
  assert.ok(!parityPlanObj.error, parityPlanObj.error);
  assert.ok(!parityPlanGlb.error, parityPlanGlb.error);
  assert.deepStrictEqual(parityPlanGlb.pts, parityPlanObj.pts);
  assert.deepStrictEqual(parityPlanGlb.segs, parityPlanObj.segs);
  assert.strictEqual(parityPlanGlb.len, parityPlanObj.len);
  assert.strictEqual(parityPlanGlb.expanded, parityPlanObj.expanded);
});

console.log('GLB parser regressions: ' + checks + '/' + checks + ' passed');
console.log('planner parity: ' + parityPlanObj.pts.length + ' points, ' +
  parityPlanObj.len.toFixed(2) + ' m, ' + parityPlanObj.expanded + ' expanded');

const fixtureArg = process.argv.indexOf('--write-fixture');
if (fixtureArg >= 0){
  const out = process.argv[fixtureArg + 1];
  if (!out) throw new Error('--write-fixture needs an output path');
  fs.writeFileSync(out, parity.buf);
  console.log('wrote browser fixture: ' + out);
}
