// ======================================================================
// 模組化手臂運動學核心(零件庫 + 通用FK + 通用DLS數值IK)
// ----------------------------------------------------------------------
// 設計原則:
//   每個零件只描述"自己"的形狀:從插頭(輸入)到插孔(輸出)的一段齊次轉換矩陣。
//   組裝 = 依序把零件的矩陣乘起來,不使用DH參數,所以任何組法都適用。
//
// 插孔座標系的約定(所有零件共用):
//   Z軸 = 插入方向(下一個零件從插孔往外延伸的方向)
//   X軸 = 下一顆馬達"直接"插上去、0°時,它的側面朝向的方向
//
// 馬達模組的座標系:原點 = pivot插入端接合面中心,Z = pivot軸線(指向body),
//   X = 馬達0°時側面插孔的方向。馬達角度為正 = 繞自己的+Z軸右手方向旋轉。
//   (已驗證:這個約定下 servo值 = 模組角度,不再需要 HARDWARE_SIGN)
//
// 長度單位 mm,角度單位 度。矩陣為 4x4 row-major 的長度16陣列(跟原本網頁 ikMultiply 相同格式)
// ======================================================================

// ---------- 矩陣工具 ----------
function mkIdentity() { return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]; }
function mkMul(a, b) {
    const r = new Array(16).fill(0);
    for (let i = 0; i < 4; i++)
        for (let j = 0; j < 4; j++) {
            let s = 0;
            for (let k = 0; k < 4; k++) s += a[i*4+k] * b[k*4+j];
            r[i*4+j] = s;
        }
    return r;
}
function mkChain(...ms) { return ms.reduce((acc, m) => mkMul(acc, m), mkIdentity()); }
function mkTr(x, y, z) { return [1,0,0,x, 0,1,0,y, 0,0,1,z, 0,0,0,1]; }
function mkRz(deg) { const t = deg*Math.PI/180, c = Math.cos(t), s = Math.sin(t); return [c,-s,0,0, s,c,0,0, 0,0,1,0, 0,0,0,1]; }
function mkRy(deg) { const t = deg*Math.PI/180, c = Math.cos(t), s = Math.sin(t); return [c,0,s,0, 0,1,0,0, -s,0,c,0, 0,0,0,1]; }
function mkRx(deg) { const t = deg*Math.PI/180, c = Math.cos(t), s = Math.sin(t); return [1,0,0,0, 0,c,-s,0, 0,s,c,0, 0,0,0,1]; }
function mkPos(T) { return [T[3], T[7], T[11]]; }
function mkAxisZ(T) { return [T[2], T[6], T[10]]; }
function mkAxisX(T) { return [T[0], T[4], T[8]]; }
function mkApply(T, p) { // 把零件局部座標的點 p 轉到世界座標
    return [T[0]*p[0]+T[1]*p[1]+T[2]*p[2]+T[3],
            T[4]*p[0]+T[5]*p[1]+T[6]*p[2]+T[7],
            T[8]*p[0]+T[9]*p[1]+T[10]*p[2]+T[11]];
}

// ---------- 零件庫 ----------
// path:零件內部的轉折點(局部座標),用來畫骨架線、產生碰撞膠囊;只影響碰撞/顯示,不影響FK
const PART_LIBRARY = {
    base: {
        name: '底座',
        groundHeight: 215,   // 底座插孔離地高度(世界座標原點 = 底座插孔接合面中心)
        T: mkRz(270),        // 馬達1在0°時,側面朝世界 -Y(這樣新系統的世界座標跟舊網頁完全一致)
    },
    motor: {
        name: '馬達模組',
        inputToAxis: 48,     // pivot接合面 → 旋轉點
        faces: {
            0: { name: '頂面', T: mkTr(0, 0, 40),                    path: [[0,0,0],[0,0,40]] },
            1: { name: '側面', T: mkMul(mkTr(40, 0, 0), mkRy(90)),   path: [[0,0,0],[40,0,0]] },
        },
    },
    connectors: {
        direct: { name: '直接連接', T: mkIdentity(),                              path: [] },
        short:  { name: '短轉接頭', T: mkMul(mkTr(0, 0, 28), mkRz(180)),           path: [[0,0,0],[0,0,28]] },
        rod2:   { name: '延長桿2',  T: mkChain(mkTr(68, 0, 85), mkRy(90), mkRz(180)), path: [[0,0,0],[0,0,85],[68,0,85]] },
        rod:    { name: '延長桿',   T: mkTr(116, 0, 101.3),                        path: [[0,0,0],[0,0,52],[116,0,52],[116,0,101.3]] },
    },
    endEffectors: {
        gripper: { name: '夾爪', T: mkTr(0, 0, 125.4), path: [[0,0,0],[0,0,125.4]] },
        none:    { name: '無',   T: mkIdentity(),      path: [] },
    },
};

// ---------- 組裝描述 ----------
// 組裝 = 從底座開始,一個接一個的零件序列(可以連續接多個連接件,也可以馬達直接接馬達):
//   { kind:'motor', face: 0|1 }      馬達模組;face = 下一個零件接在它的哪一面(0頂面 / 1側面)
//   { kind:'connector', key:'short' } 連接件(key 對應 PART_LIBRARY.connectors)
//   { kind:'ee', key:'gripper' }      末端(只能放最後)
// 規則:馬達最多 5 顆;末端之後不能再接
const MAX_MOTORS = 5;

// 目前實體手臂的組法:
const CURRENT_ARM_ASSEMBLY = {
    parts: [
        { kind: 'motor', face: 1 }, { kind: 'connector', key: 'short' },
        { kind: 'motor', face: 1 }, { kind: 'connector', key: 'rod2' },
        { kind: 'motor', face: 1 }, { kind: 'connector', key: 'short' },
        { kind: 'motor', face: 1 }, { kind: 'connector', key: 'short' },
        { kind: 'motor', face: 1 }, { kind: 'connector', key: 'rod' },
        { kind: 'ee', key: 'gripper' },
    ],
};

function motorCount(assembly) { return assembly.parts.filter(p => p.kind === 'motor').length; }

// 下一個零件可不可以接上去(給介面用)
function canAppend(assembly, part) {
    const last = assembly.parts[assembly.parts.length - 1];
    if (last && last.kind === 'ee') return false;
    if (part.kind === 'motor' && motorCount(assembly) >= MAX_MOTORS) return false;
    return true;
}

// ---------- 通用正向運動學 ----------
// thetas: 每顆馬達的角度(度),依照馬達在序列裡出現的順序
// 回傳:
//   pos, dir      末端位置(相對底座插孔)與指向;沒有末端時 = 最後一個插孔
//   T             末端完整矩陣
//   openSocket    還沒接東西的插孔(下一個零件會接在這),已接末端則為 null
//   motorFrames   每顆馬達"旋轉後"的座標系(畫body用)
//   inputFrames   每顆馬達 pivot 插入處的座標系(畫pivot用,不隨自己旋轉)
//   partFrames    每個連接件/末端的插頭座標系 {kind, key, partIndex, T}(畫連接件STL用)
//   skeleton      骨架點串列(世界座標),每點附 owner(屬於哪個剛體),用來畫線和做碰撞膠囊
function modularFK(assembly, thetas) {
    const lib = PART_LIBRARY;
    let T = lib.base.T.slice();
    const skeleton = [{ p: mkPos(T), owner: 'base' }];
    const motorFrames = [], inputFrames = [], partFrames = [];
    let mi = 0, endedWithEE = false;

    assembly.parts.forEach((part, pi) => {
        if (part.kind === 'motor') {
            inputFrames.push(T.slice());
            T = mkMul(T, mkTr(0, 0, lib.motor.inputToAxis));          // pivot接合面 → 旋轉點
            skeleton.push({ p: mkPos(T), owner: `m${mi}` });
            T = mkMul(T, mkRz(thetas[mi] || 0));                       // 旋轉
            motorFrames.push(T.slice());
            const face = lib.motor.faces[part.face ?? 0];               // 旋轉點 → 選用的面
            face.path.slice(1).forEach(p => skeleton.push({ p: mkApply(T, p), owner: `m${mi}` }));
            T = mkMul(T, face.T);
            mi++;
        } else {
            const def = part.kind === 'ee' ? lib.endEffectors[part.key] : lib.connectors[part.key];
            const owner = part.kind === 'ee' ? 'ee' : `c${pi}`;
            partFrames.push({ kind: part.kind, key: part.key, partIndex: pi, T: T.slice() });
            def.path.slice(1).forEach(p => skeleton.push({ p: mkApply(T, p), owner }));
            T = mkMul(T, def.T);
            if (part.kind === 'ee') endedWithEE = true;
        }
    });

    const d = mkAxisZ(T), dl = Math.hypot(...d) || 1;
    return { pos: mkPos(T), dir: d.map(v => v / dl), T, openSocket: endedWithEE ? null : T.slice(),
             motorFrames, inputFrames, partFrames, skeleton };
}

// ---------- 通用 DLS 數值逆向運動學 ----------
// 關節數 n = 序列裡的馬達數,自動決定模式:
//   n >= 5 且有給 targetDir → 位置+方向(6維誤差);否則 → 只做位置(3維誤差)
const MK_ORIENT_WEIGHT = 120;
const MK_JOINT_LIMIT = [-120, 120];

function mkOutputVec(assembly, thetas, posOnly) {
    const r = modularFK(assembly, thetas);
    return posOnly ? r.pos : [...r.pos, ...r.dir.map(v => v * MK_ORIENT_WEIGHT)];
}

function mkSolveLinear(A, b) { // 高斯消去(部分選主元),A 為 n×n
    const n = b.length, M = A.map((row, i) => [...row, b[i]]);
    for (let c = 0; c < n; c++) {
        let p = c;
        for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
        [M[c], M[p]] = [M[p], M[c]];
        if (Math.abs(M[c][c]) < 1e-12) return null;
        for (let r = c + 1; r < n; r++) {
            const f = M[r][c] / M[c][c];
            for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
        }
    }
    const x = new Array(n).fill(0);
    for (let r = n - 1; r >= 0; r--) {
        let s = M[r][n];
        for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
        x[r] = s / M[r][r];
    }
    return x;
}

function mkNorm(v) { return Math.hypot(...v); }

function modularIKAttempt(assembly, init, targetVec, posOnly, opts = {}) {
    const n = motorCount(assembly), h = 0.05, tol = opts.tol ?? 0.1, maxIter = opts.maxIter ?? 100;
    let th = init.slice(), lam = 5, iters = 0;
    let out = mkOutputVec(assembly, th, posOnly);
    let err = targetVec.map((v, k) => v - out[k]), errN = mkNorm(err);
    while (iters < maxIter && errN > tol) {
        iters++;
        // 數值 Jacobian (m × n)
        const m = targetVec.length, J = Array.from({ length: m }, () => new Array(n).fill(0));
        for (let j = 0; j < n; j++) {
            const t2 = th.slice(); t2[j] += h;
            const o2 = mkOutputVec(assembly, t2, posOnly);
            for (let k = 0; k < m; k++) J[k][j] = (o2[k] - out[k]) / h;
        }
        // (JᵀJ + λ²I) Δθ = Jᵀe
        const A = Array.from({ length: n }, (_, a) => Array.from({ length: n }, (_, b) => {
            let s = 0; for (let k = 0; k < m; k++) s += J[k][a] * J[k][b];
            return s + (a === b ? lam * lam : 0);
        }));
        const g = Array.from({ length: n }, (_, a) => { let s = 0; for (let k = 0; k < m; k++) s += J[k][a] * err[k]; return s; });
        const dth = mkSolveLinear(A, g);
        if (!dth) break;
        const cand = th.map((v, j) => Math.max(MK_JOINT_LIMIT[0], Math.min(MK_JOINT_LIMIT[1], v + dth[j])));
        const cOut = mkOutputVec(assembly, cand, posOnly);
        const cErr = targetVec.map((v, k) => v - cOut[k]), cN = mkNorm(cErr);
        if (cN < errN) { th = cand; out = cOut; err = cErr; errN = cN; lam = Math.max(lam * 0.7, 0.01); }
        else { lam *= 2.5; if (lam > 1e6) break; }
    }
    return { thetas: th, errNorm: errN, iterations: iters, converged: errN <= tol };
}

// targetPos:相對底座插孔的座標(mm);targetDir:可省略
function modularIK(assembly, targetPos, targetDir, opts = {}) {
    const n = motorCount(assembly);
    if (n === 0) return { thetas: [], errNorm: Infinity, iterations: 0, converged: false, mode: 'none' };
    const posOnly = !(targetDir && n >= 5);
    let targetVec = targetPos.slice();
    if (!posOnly) { const dl = mkNorm(targetDir) || 1; targetVec = [...targetPos, ...targetDir.map(v => v / dl * MK_ORIENT_WEIGHT)]; }
    const seeds = [
        opts.warmStart,
        new Array(n).fill(0),
        [30, -30, 30, 0, -30], [-30, 40, -40, 20, 30], [10, -60, 60, -10, -20], [0, 60, -60, 0, 40],
    ].filter(Boolean).map(s => Array.from({ length: n }, (_, j) => s[j] || 0));
    let best = null;
    for (const s of seeds) {
        const r = modularIKAttempt(assembly, s, targetVec, posOnly, opts);
        if (!best || r.errNorm < best.errNorm) best = r;
        if (r.converged) break;
    }
    best.mode = posOnly ? 'position' : 'position+direction';
    return best;
}

if (typeof module !== 'undefined') module.exports = { PART_LIBRARY, CURRENT_ARM_ASSEMBLY, MAX_MOTORS, motorCount, canAppend, modularFK, modularIK, mkMul, mkTr, mkRz, mkRy, mkChain };
