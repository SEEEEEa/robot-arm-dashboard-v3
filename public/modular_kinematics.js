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
        rod:    { name: '延長桿',   T: mkMul(mkTr(116, 0, 101.3), mkRz(-90)),      path: [[0,0,0],[0,0,52],[116,0,52],[116,0,101.3]] }, // 新版:插孔轉向,讓夾爪換角度(原本 Rz(180))
        rod3:   { name: '延長桿3',  T: mkMul(mkTr(0, 0, 80), mkRz(90)),             path: [[0,0,0],[0,0,80]] },
        rod4:   { name: '延長桿4',  T: mkMul(mkTr(110, 0, 8), mkRx(180)),           path: [[0,0,0],[0,0,36],[110,0,36],[110,0,8]] }, // U形回折:出口朝回頭方向
        scara1: { name: 'SCARA連接件1', T: mkMul(mkTr(0, 0, 128.5), mkRz(90)),        path: [[0,0,0],[0,0,128.5]] },
        scara2: { name: 'SCARA連接件2', T: mkMul(mkTr(-68, 0, 145), mkRy(-90)),       path: [[0,0,0],[0,0,145],[-68,0,145]] },
    },
    // 直線模組:自帶一顆馬達,馬達轉動 → 齒輪 → 齒條平移(移動關節)
    //   零件座標系同連接件:原點 = 插頭接合面中心,Z = 插入方向
    //   T(θ) = Tr(offset + travelDir·d(θ)) · R,  d(θ) = mmPerDeg·(θ − topAngle)
    linearModules: {
        scaraZ: {
            name: 'SCARA升降模組',
            // 齒輪A 60齒(馬達) → 齒輪C 20齒(×3) ; 同軸齒輪B 36齒(節圓半徑22.5) → 齒條(模數1.25)
            mmPerDeg: 3 * 22.5 * Math.PI / 180,   // ≈ 1.178 mm/度
            topAngle: -120,                        // 齒條在最高點時的馬達角度(d = 0)
            homeAngle: -120,                       // 歸零時的角度(這顆用特別ID,起始角度跟其他馬達不同)
            limit: [-120, 120],                    // 之後視情況縮小,避免脫齒
            offset: [-137.25, -10.5, 86.35],       // 最高點時:插頭接合面 → 齒條插孔中心
            travelDir: [0, -1, 0],                 // d 增加 = 齒條往這個方向移動(往下)
            R: mkRx(90),                           // 齒條插孔的朝向(開口朝齒條移動方向)
            rackLength: 300,
            path: [[0,0,0],[0,0,86.35],[-137.25,0,86.35]],
        },
    },
    endEffectors: {
        gripper: { name: '夾爪', T: mkTr(0, 0, 125.4), path: [[0,0,0],[0,0,125.4]] },
        none:    { name: '無',   T: mkIdentity(),      path: [] },
    },
};

// ---------- 模型與碰撞盒(由 tools/build_parts.py 從 STL 自動產生,請勿手改) ----------
// pieces: STL 檔 → 零件局部座標: local = R·(v − o);moving = 升降模組會隨 d 移動的部分;spin = 齒輪轉動
// collision: 零件局部座標下的軸對齊盒 [cx, cy, cz, hx, hy, hz](中心與半邊長, mm)
const PART_GEO = {"short":{"pieces":[{"file":"短轉接頭.stl","R":[[-1.0,-0.0,-0.0],[0.0,0.0,-1.0],[0.0,-1.0,0.0]],"o":[28.75,28.0,28.75]}],"collision":[[0.0,0.0,-5.0,28.75,28.75,10.0],[0.0,0.0,18.0,28.75,28.75,10.0]]},"rod":{"pieces":[{"file":"延長桿.stl","R":[[-1.0,-0.0,-0.0],[0.0,0.0,-1.0],[0.0,-1.0,0.0]],"o":[144.75,101.3,28.75]}],"collision":[[0.0,0.0,-5.0,28.75,28.75,10.0],[0.0,0.0,18.83,28.75,28.75,10.83],[2.42,0.14,50.87,19.65,16.98,21.2],[13.75,0.01,52.0,13.75,15.23,25.0],[109.17,-0.21,51.99,20.67,15.45,24.99],[112.85,0.0,64.34,20.63,17.46,12.31],[116.0,0.0,88.98,28.75,28.75,12.32]]},"rod2":{"pieces":[{"file":"延長桿2.stl","R":[[-1.0,-0.0,-0.0],[0.0,0.0,-1.0],[0.0,-1.0,0.0]],"o":[68.0,113.8,28.75]}],"collision":[[-0.0,0.0,-3.45,28.75,28.75,11.5],[-0.0,0.0,17.55,28.75,28.75,2.5],[3.51,0.0,81.74,21.01,17.5,21.69],[11.34,-0.24,87.12,11.33,15.58,22.82],[32.6,0.0,85.05,9.9,28.75,28.75],[57.75,0.0,85.05,10.25,28.75,28.75]]},"rod3":{"pieces":[{"file":"延長桿3.stl","R":[[-1.0,-0.0,-0.0],[0.0,0.0,-1.0],[0.0,-1.0,0.0]],"o":[28.75,80.0,28.75]}],"collision":[[0.0,0.0,-3.5,28.75,28.75,11.5],[0.0,0.0,17.5,28.75,28.75,2.5],[0.0,0.0,70.0,28.75,28.75,10.0]]},"rod4":{"pieces":[{"file":"延長桿4.stl","R":[[-1.0,-0.0,-0.0],[0.0,0.0,-1.0],[0.0,-1.0,0.0]],"o":[138.75,44.0,86.0]}],"collision":[[0.0,0.0,-3.5,28.75,28.75,11.5],[-3.95,0.0,29.5,24.8,28.75,14.5],[15.7,0.0,23.5,11.06,28.75,8.5],[28.18,0.0,32.0,0.57,24.5,12.0],[74.71,0.0,26.0,7.46,24.11,18.0],[105.06,0.0,33.5,22.19,28.75,10.5],[110.46,0.0,21.96,28.29,28.75,13.96]]},"scara1":{"pieces":[{"file":"SCARA連接件1.stl","R":[[1.0,-0.0,-0.0],[0.0,0.0,1.0],[0.0,-1.0,0.0]],"o":[28.75,128.5,28.75]}],"collision":[[0.0,0.0,-3.5,28.75,28.75,11.5],[0.0,0.0,17.5,28.75,28.75,2.5],[0.0,0.0,114.25,28.75,28.75,14.25]]},"scara2":{"pieces":[{"file":"SCARA連接件2.stl","R":[[1.0,-0.0,-0.0],[0.0,0.0,1.0],[0.0,-1.0,0.0]],"o":[68.0,173.8,28.75]}],"collision":[[0.0,0.0,-3.45,28.75,28.75,11.5],[0.0,0.0,17.55,28.75,28.75,2.5],[0.0,0.0,40.05,15.0,15.0,0.0],[0.0,0.0,100.05,15.0,15.0,0.0],[-3.5,0.0,141.74,21.0,17.5,21.69],[-11.34,0.24,147.23,11.31,15.58,22.72],[-32.59,0.0,145.05,9.91,28.75,28.75],[-57.75,0.0,145.05,10.25,28.75,28.75]]},"gripper":{"pieces":[{"file":"組合件1.stl","R":[[0.0,-1.0,-0.0],[0.0,0.0,-1.0],[1.0,0.0,0.0]],"o":[15.29,40.34,66.05]}],"collision":[[0.0,-0.0,-1.5,40.0,40.0,13.5],[0.0,-4.0,23.5,19.21,31.0,9.5],[0.0,-0.0,56.49,32.5,65.0,12.91],[0.0,-0.0,83.55,32.5,65.0,13.35],[0.0,0.23,113.4,6.0,46.11,13.5]]},"motor":{"pivot":{"file":"pivot.stl","R":[[1.0,0.0,0.0],[0.0,0.0,-1.0],[0.0,1.0,0.0]],"o":[0,0,0]},"body":{"file":"body.stl","R":[[1.0,0.0,0.0],[0.0,0.0,-1.0],[0.0,1.0,0.0]],"o":[0,0,0],"shiftZ":-48},"collision":{"pivot":[[-0.02,0.0,2.99,34.99,35.0,18.06]],"body":[[-0.14,0.0,1.0,40.11,40.0,40.11]]}},"base":{"pieces":[{"file":"底座成品2.stl","R":[[0.0,0.0,-1.0],[-1.0,0.0,0.0],[0.0,1.0,0.0]],"o":[143.55,215.28,227.75]}],"collision":[[-2.25,14.5,-175.0,100.0,127.5,40.0],[0,0,-67.5,33.0,33.0,67.5]]},"scaraZ":{"pieces":[{"file":"升降_馬達支架.stl","R":[[-1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,-1.0]],"o":[39.97,39.99,106.6]},{"file":"升降_齒輪A.stl","R":[[-1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,-1.0]],"o":[39.0,38.75,78.2],"spin":{"center":[0.25,0.0],"perMM":-0.014814814814814815}},{"file":"升降_齒輪BC.stl","R":[[-1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,-1.0]],"o":[-26.0,23.75,141.9],"spin":{"center":[-49.75,0.0],"perMM":0.044444444444444446}},{"file":"升降_齒條.stl","R":[[0.0,1.0,0.0],[-1.0,0.0,0.0],[0.0,0.0,1.0]],"o":[279.0,380.89,-57.56],"moving":true},{"file":"升降_LX224伺服馬達.stl","R":[[-1.0,0.0,0.0],[0.0,0.0,-1.0],[0.0,-1.0,0.0]],"o":[10.3,68.2,9.9]},{"file":"升降_BackHub.stl","R":[[-0.974761,0.0,-0.22325],[0.22325,0.0,-0.974761],[0.0,-1.0,0.0]],"o":[9.9937,68.2,9.8058],"spin":{"center":[0.25,0.0],"perMM":-0.014814814814814815}}],"collision":{"static":[[-27.96,16.56,18.41,67.93,56.55,33.41],[10.37,-2.49,62.5,29.6,37.5,2.5],[-55.31,15.31,63.05,15.28,55.3,3.05],[-83.03,36.69,83.83,12.87,36.42,22.77],[0.25,-0.0,73.2,38.75,38.75,5.0],[-49.75,0.0,80.4,23.75,23.75,14.0],[0.25,-10.01,42.65,10.05,19.91,25.55],[0.25,-0.0,66.4,9.75,9.75,1.8]],"moving":[[-135.04,11.0,86.31,31.25,21.5,28.75],[-77.94,2.35,86.88,5.85,23.35,7.5],[-76.49,51.51,86.88,4.4,25.1,7.5],[-76.49,102.07,86.88,4.4,25.38,7.5],[-76.49,152.9,86.88,4.4,25.34,7.5],[-76.49,203.75,86.88,4.4,25.38,7.5],[-76.49,254.14,86.88,4.4,24.86,7.5]]}}};

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

// SCARA 組法(使用者確認):兩顆直立轉軸 + 升降模組,夾爪朝下
const SCARA_ASSEMBLY = {
    parts: [
        { kind: 'motor', face: 1 }, { kind: 'connector', key: 'scara2' },
        { kind: 'motor', face: 1 }, { kind: 'connector', key: 'scara1' },
        { kind: 'linear', key: 'scaraZ' },
        { kind: 'ee', key: 'gripper' },
    ],
};

const isJoint = p => p.kind === 'motor' || p.kind === 'linear';   // 會吃掉一個馬達角度的零件
function motorCount(assembly) { return assembly.parts.filter(isJoint).length; }
// 每個關節的角度範圍與歸零角度(依序對應 thetas)
function jointInfo(assembly) {
    return assembly.parts.filter(isJoint).map(p => p.kind === 'linear'
        ? { kind: 'linear', key: p.key, limit: PART_LIBRARY.linearModules[p.key].limit.slice(), home: PART_LIBRARY.linearModules[p.key].homeAngle }
        : { kind: 'motor', limit: MK_JOINT_LIMIT.slice(), home: 0 });
}
function linearDisplacement(key, theta) { const L = PART_LIBRARY.linearModules[key]; return L.mmPerDeg * (theta - L.topAngle); }

// 下一個零件可不可以接上去(給介面用)
function canAppend(assembly, part) {
    const last = assembly.parts[assembly.parts.length - 1];
    if (last && last.kind === 'ee') return false;
    if (isJoint(part) && motorCount(assembly) >= MAX_MOTORS) return false;
    return true;
}

// ---------- 通用正向運動學 ----------
// thetas: 每顆馬達的角度(度),依照馬達在序列裡出現的順序
// 回傳:
//   pos, dir      末端位置(相對底座插孔)與指向;沒有末端時 = 最後一個插孔
//   T             末端完整矩陣
//   openSocket    還沒接東西的插孔(下一個零件會接在這),已接末端則為 null
//   motorFrames   每顆旋轉馬達"旋轉後"的座標系(畫body用);motorFrameJoint[k] = 它是第幾個關節
//   linearFrames  每個直線模組 {Tin 插頭座標系, Tblock 齒條插孔座標系, d 目前位移mm}
//   inputFrames   每顆馬達 pivot 插入處的座標系(畫pivot用,不隨自己旋轉)
//   partFrames    每個連接件/末端的插頭座標系 {kind, key, partIndex, T}(畫連接件STL用)
//   skeleton      骨架點串列(世界座標),每點附 owner(屬於哪個剛體),用來畫線和做碰撞膠囊
function modularFK(assembly, thetas) {
    const lib = PART_LIBRARY;
    let T = lib.base.T.slice();
    const skeleton = [{ p: mkPos(T), owner: 'base' }];
    const motorFrames = [], motorFrameJoint = [], inputFrames = [], partFrames = [], linearFrames = [];
    let mi = 0, endedWithEE = false;

    assembly.parts.forEach((part, pi) => {
        if (part.kind === 'motor') {
            inputFrames.push(T.slice());
            T = mkMul(T, mkTr(0, 0, lib.motor.inputToAxis));          // pivot接合面 → 旋轉點
            skeleton.push({ p: mkPos(T), owner: `m${mi}` });
            T = mkMul(T, mkRz(thetas[mi] || 0));                       // 旋轉
            motorFrames.push(T.slice()); motorFrameJoint.push(mi);
            const face = lib.motor.faces[part.face ?? 0];               // 旋轉點 → 選用的面
            face.path.slice(1).forEach(p => skeleton.push({ p: mkApply(T, p), owner: `m${mi}` }));
            T = mkMul(T, face.T);
            mi++;
        } else if (part.kind === 'linear') {
            const L = lib.linearModules[part.key];
            const th = thetas[mi] ?? L.homeAngle;
            const d = L.mmPerDeg * (th - L.topAngle);
            const off = L.offset.map((v, k) => v + L.travelDir[k] * d);
            linearFrames.push({ key: part.key, motorIndex: mi, partIndex: pi, Tin: T.slice(), d,
                                Tblock: mkMul(T, mkMul(mkTr(...off), L.R)) });
            L.path.slice(1).forEach(p => skeleton.push({ p: mkApply(T, p), owner: `m${mi}` }));
            skeleton.push({ p: mkApply(T, off), owner: `m${mi}` });
            T = mkMul(T, mkMul(mkTr(...off), L.R));
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
             motorFrames, motorFrameJoint, inputFrames, partFrames, linearFrames, skeleton };
}

// ---------- 通用 DLS 數值逆向運動學 ----------
// modularIKAttempt:單次 DLS(posOnly=true 只算位置3維,否則位置+方向6維)
// 模式選擇在後面的 modularIK 裡,依自由度分析自動決定
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
    const lim = jointInfo(assembly).map(j => j.limit);
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
        const cand = th.map((v, j) => Math.max(lim[j][0], Math.min(lim[j][1], v + dth[j])));
        const cOut = mkOutputVec(assembly, cand, posOnly);
        const cErr = targetVec.map((v, k) => v - cOut[k]), cN = mkNorm(cErr);
        if (cN < errN) { th = cand; out = cOut; err = cErr; errN = cN; lam = Math.max(lam * 0.7, 0.01); }
        else { lam *= 2.5; if (lam > 1e6) break; }
    }
    return { thetas: th, errNorm: errN, iterations: iters, converged: errN <= tol };
}

// ---------- 自由度分析 ----------
// 不只數馬達顆數,而是看這個組法的 Jacobian 實際有幾個有效方向:
//   任務 = 位置3維 + 方向2維(方向是單位向量,只有2個獨立量;夾爪滾轉不算)
//   在多組隨機姿態下算奇異值,取最大的有效秩(避開剛好落在奇異姿態的情況)
// 例:兩顆馬達轉軸共線時,效果重疊,有效自由度會比馬達數少
function mkSymEig(A) { // 對稱矩陣特徵值(Jacobi 旋轉法),n ≤ 5 很快
    const n = A.length, M = A.map(r => r.slice());
    for (let sweep = 0; sweep < 60; sweep++) {
        let off = 0;
        for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += M[i][j] * M[i][j];
        if (off < 1e-20) break;
        for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
            if (Math.abs(M[p][q]) < 1e-15) continue;
            const th = 0.5 * Math.atan2(2 * M[p][q], M[q][q] - M[p][p]), c = Math.cos(th), s = Math.sin(th);
            for (let k = 0; k < n; k++) { const kp = M[k][p], kq = M[k][q]; M[k][p] = c * kp - s * kq; M[k][q] = s * kp + c * kq; }
            for (let k = 0; k < n; k++) { const pk = M[p][k], qk = M[q][k]; M[p][k] = c * pk - s * qk; M[q][k] = s * pk + c * qk; }
        }
    }
    return M.map((r, i) => r[i]);
}
function mkSingularValues(J) { // J: m×n → 奇異值(由大到小)
    const n = J[0].length, m = J.length;
    const JtJ = Array.from({ length: n }, (_, a) => Array.from({ length: n }, (_, b) => { let s = 0; for (let k = 0; k < m; k++) s += J[k][a] * J[k][b]; return s; }));
    return mkSymEig(JtJ).map(v => Math.sqrt(Math.max(0, v))).sort((x, y) => y - x);
}
function mkRank(J, rel = 1e-3) {
    if (!J.length || !J[0].length) return 0;
    const sv = mkSingularValues(J); if (sv[0] < 1e-9) return 0;
    return sv.filter(v => v > rel * sv[0]).length;
}
// 5×n 任務 Jacobian:[位置(mm); 方向切平面兩分量 × MK_ORIENT_WEIGHT],中央差分
function mkTaskJacobian(assembly, thetas) {
    const n = thetas.length, h = 0.01, f0 = modularFK(assembly, thetas), d0 = f0.dir;
    const ref = Math.abs(d0[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
    let u = cross(d0, ref); const ul = mkNorm(u); u = u.map(v => v / ul);
    const w = cross(d0, u);
    const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    const Jp = [[], [], []], Jd = [[], []];
    for (let j = 0; j < n; j++) {
        const tp = thetas.slice(), tm = thetas.slice(); tp[j] += h; tm[j] -= h;
        const fp = modularFK(assembly, tp), fm = modularFK(assembly, tm);
        const hr = 2 * h * Math.PI / 180; // 對弧度微分,讓方向的單位是 rad
        for (let k = 0; k < 3; k++) Jp[k][j] = (fp.pos[k] - fm.pos[k]) / (2 * h);
        const dd = [0, 1, 2].map(k => (fp.dir[k] - fm.dir[k]) / hr);
        Jd[0][j] = dot(dd, u) * MK_ORIENT_WEIGHT * Math.PI / 180;
        Jd[1][j] = dot(dd, w) * MK_ORIENT_WEIGHT * Math.PI / 180;
    }
    return { Jp, Jd, J: [...Jp, ...Jd] };
}
function analyzeDOF(assembly, samples = 12) {
    const n = motorCount(assembly);
    if (n === 0) return { motors: 0, posDOF: 0, dirDOF: 0, fullControl: false };
    let posRank = 0, totalRank = 0, seed = 12345;
    const rnd = () => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648; };
    for (let k = 0; k < samples; k++) {
        const th = Array.from({ length: n }, () => rnd() * 200 - 100);
        const { Jp, J } = mkTaskJacobian(assembly, th);
        posRank = Math.max(posRank, mkRank(Jp));
        totalRank = Math.max(totalRank, mkRank(J));
    }
    return { motors: n, posDOF: posRank, dirDOF: totalRank - posRank, fullControl: posRank === 3 && totalRank === 5 };
}

// ---------- 位置優先的 IK(方向盡量接近) ----------
// 主任務:位置誤差 → 0;次任務:在"不影響位置"的空間(零空間)裡,讓方向盡量接近
//   Δθ = Jp⁺·ep + N·(Jd·N)⁺·(ed − Jd·Jp⁺·ep),  N = I − Jp⁺·Jp   (⁺ 為阻尼偽逆)
function mkDampedPinv(J, lam) { // n×m: (JᵀJ + λ²I)⁻¹ Jᵀ
    const m = J.length, n = J[0].length;
    const A = Array.from({ length: n }, (_, a) => Array.from({ length: n }, (_, b) => {
        let s = 0; for (let k = 0; k < m; k++) s += J[k][a] * J[k][b]; return s + (a === b ? lam * lam : 0); }));
    const P = Array.from({ length: n }, () => new Array(m).fill(0));
    for (let k = 0; k < m; k++) {
        const col = mkSolveLinear(A, Array.from({ length: n }, (_, a) => J[k][a]));
        if (!col) return null;
        for (let a = 0; a < n; a++) P[a][k] = col[a];
    }
    return P;
}
const mkMatVec = (M, v) => M.map(r => r.reduce((s, x, i) => s + x * v[i], 0));
const mkMatMul = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, x, k) => s + x * B[k][j], 0)));
function mkAngleDeg(a, b) {
    const c = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]) / ((mkNorm(a) || 1) * (mkNorm(b) || 1));
    return Math.acos(Math.max(-1, Math.min(1, c))) * 180 / Math.PI;
}

function modularIKPriorityAttempt(assembly, init, targetPos, targetDir, opts = {}) {
    const n = init.length, tolP = opts.tol ?? 0.1, maxIter = opts.maxIter ?? 120;
    const lim = jointInfo(assembly).map(j => j.limit);
    const td = targetDir.map(v => v / (mkNorm(targetDir) || 1));
    const evalTh = th => { const f = modularFK(assembly, th); return { th, f, pe: mkNorm(targetPos.map((v, k) => v - f.pos[k])), de: mkAngleDeg(f.dir, td) }; };
    // 把姿態拉回目標位置(只修位置,DLS 最小改變量,方向幾乎不受影響)
    const fixPos = (th, it) => modularIKAttempt(assembly, th, targetPos.slice(), true, { tol: tolP * 0.5, maxIter: it }).thetas;

    // 階段A:先用穩定的位置DLS把位置做到
    let cur = evalTh(fixPos(init, 100));
    let iters = 0;
    if (cur.pe > tolP) return { thetas: cur.th, posErr: cur.pe, dirErrDeg: cur.de, iterations: 100, converged: false };

    // 階段B:在不動到位置的空間(零空間)裡轉,讓方向越來越接近;每步之後再把位置拉回來
    let lam = 1, stall = 0;
    while (iters < maxIter) {
        iters++;
        const th = cur.th;
        const { Jp } = mkTaskJacobian(assembly, th);
        const h = 0.05, Jd = [[], [], []];
        for (let j = 0; j < n; j++) { const t2 = th.slice(); t2[j] += h; const d2 = modularFK(assembly, t2).dir;
            for (let k = 0; k < 3; k++) Jd[k][j] = (d2[k] - cur.f.dir[k]) / h * MK_ORIENT_WEIGHT; }
        const ed = td.map((v, k) => (v - cur.f.dir[k]) * MK_ORIENT_WEIGHT);
        const JpP = mkDampedPinv(Jp, 0.01); if (!JpP) break;
        const JpPJp = mkMatMul(JpP, Jp);
        const N = JpPJp.map((r, a) => r.map((x, b) => (a === b ? 1 : 0) - x));
        const JdNP = mkDampedPinv(mkMatMul(Jd, N), lam); if (!JdNP) break;
        let dth = mkMatVec(N, mkMatVec(JdNP, ed));
        const mx = Math.max(...dth.map(Math.abs)); if (mx > 15) dth = dth.map(v => v * 15 / mx);
        if (mx < 1e-6) break;
        let cand = th.map((v, j) => Math.max(lim[j][0], Math.min(lim[j][1], v + dth[j])));
        let c = evalTh(cand);
        if (c.pe > tolP) c = evalTh(fixPos(cand, 15));
        if (c.pe <= tolP && c.de < cur.de - 1e-6) {
            stall = (cur.de - c.de) < 1e-3 ? stall + 1 : 0;
            cur = c; lam = Math.max(lam * 0.7, 0.01);
            if (cur.de < 0.05 || stall >= 5) break;
        } else { lam *= 2.5; if (lam > 1e5) break; }
    }
    return { thetas: cur.th, posErr: cur.pe, dirErrDeg: cur.de, iterations: iters + 100, converged: cur.pe <= tolP };
}

// targetPos:相對底座插孔的座標(mm);targetDir:可省略
// 回傳 mode:
//   'position+direction' 位置與方向都達成
//   'position-priority'  方向無法完全達成(自由度不夠或目標到不了),位置優先、方向盡量接近
//   'position'           只求位置(沒給方向)
function modularIK(assembly, targetPos, targetDir, opts = {}) {
    const n = motorCount(assembly);
    if (n === 0) return { thetas: [], errNorm: Infinity, posErr: Infinity, iterations: 0, converged: false, mode: 'none' };
    const dof = opts.dof || analyzeDOF(assembly);
    const L = jointInfo(assembly);
    const seeds = [
        opts.warmStart,
        L.map(j => j.home),
        new Array(n).fill(0),
        [30, -30, 30, 0, -30], [-30, 40, -40, 20, 30], [10, -60, 60, -10, -20], [0, 60, -60, 0, 40],
        [-30, 30, -30, 0, 30], [60, -40, 80, 0, -60], [-60, 40, -80, 0, 60], [90, 20, -50, 45, 20], [-90, -20, 50, -45, -20],
    ].filter(Boolean).map(s => Array.from({ length: n }, (_, j) => Math.max(L[j].limit[0], Math.min(L[j].limit[1], s[j] ?? 0))));
    // 碰撞檢查:預設開啟,只接受不會碰撞的解;全部都會碰撞時,回傳最好的那個並標記 collided
    const avoid = opts.avoidCollision !== false;
    const collides = th => avoid && modularCollision(assembly, th, { firstOnly: true }).collided;
    const finish = (r, mode, collided) => {
        const f = modularFK(assembly, r.thetas);
        r.posErr = mkNorm(targetPos.map((v, k) => v - f.pos[k]));
        r.dirErrDeg = targetDir ? mkAngleDeg(f.dir, targetDir) : null;
        r.errNorm = r.errNorm ?? r.posErr;
        r.mode = mode; r.dof = dof; r.collided = !!collided;
        if (r.collided) r.hits = modularCollision(assembly, r.thetas).hits;
        return r;
    };

    // 1) 沒給方向 → 只求位置
    if (!targetDir) {
        let best = null, bestColl = null;
        for (const s of seeds) {
            const r = modularIKAttempt(assembly, s, targetPos.slice(), true, opts);
            if (r.converged && !collides(r.thetas)) return finish(r, 'position', false);
            if (r.converged) { if (!bestColl) bestColl = r; }
            else if (!best || r.errNorm < best.errNorm) best = r;
        }
        return bestColl ? finish(bestColl, 'position', true) : finish(best, 'position', collides(best.thetas));
    }
    // 2) 自由度足夠 → 先試位置+方向同時達成
    const compromise = []; // 位置+方向折衷解,方向通常已經很接近,拿來當位置優先的起點
    let fullColl = null;
    {
        const dl = mkNorm(targetDir) || 1, tv = [...targetPos, ...targetDir.map(v => v / dl * MK_ORIENT_WEIGHT)];
        for (const s of seeds) {
            const r = modularIKAttempt(assembly, s, tv, false, opts);
            if (dof.fullControl && r.converged) {
                if (!collides(r.thetas)) return finish(r, 'position+direction', false);
                if (!fullColl) fullColl = r;
            }
            compromise.push(r.thetas);
            if (!dof.fullControl && compromise.length >= 3) break;
        }
    }
    // 3) 做不到(或同時達成的解都會碰撞)→ 位置優先、方向盡量接近
    let best = null, bestAny = null;
    for (const s of [...compromise, ...seeds]) {
        const r = modularIKPriorityAttempt(assembly, s, targetPos, targetDir, opts);
        const better = (x, y) => !y || (x.converged && !y.converged) || (x.converged === y.converged && (x.converged ? x.dirErrDeg < y.dirErrDeg : x.posErr < y.posErr));
        if (better(r, bestAny)) bestAny = r;
        if (r.converged && !collides(r.thetas)) {
            if (better(r, best)) best = r;
            if (r.dirErrDeg < 0.1) break;
        }
    }
    if (best) return finish(best, 'position-priority', false);
    if (fullColl) return finish(fullColl, 'position+direction', true);
    bestAny.errNorm = bestAny.posErr;
    return finish(bestAny, 'position-priority', collides(bestAny.thetas));
}

// ---------- 碰撞檢查 ----------
// 做法:每個零件用幾個「盒子」包住(PART_GEO.collision,零件局部座標),依 FK 擺到世界座標變成 OBB,
//      兩兩用分離軸定理(SAT)檢查有沒有重疊。
// 剛體分組:兩顆馬達之間的所有東西(上一顆的 body、連接件、下一顆的 pivot)是同一個剛體,同組不互相檢查。
// 例外:同一個關節兩側本來就接觸的零件(pivot ↔ 自己的 body、升降支架 ↔ 齒條)不檢查。
// 另外檢查有沒有零件低於地面(離地 0)。
const MK_COLLISION_MARGIN = 1.0;   // 每個盒子各縮 1mm,避免接合面剛好貼在一起被誤判
const MK_FLOOR_Z = -PART_LIBRARY.base.groundHeight;

function mkObb(T, box, extraOffset) {
    const c = extraOffset ? box.slice(0, 3).map((v, k) => v + extraOffset[k]) : box.slice(0, 3);
    return {
        c: mkApply(T, c),
        u: [mkAxisX(T), [T[1], T[5], T[9]], mkAxisZ(T)],
        h: box.slice(3, 6).map(v => Math.max(0.5, v - MK_COLLISION_MARGIN)),
    };
}
function mkObbOverlap(A, B) { // 分離軸定理:15 條候選軸,任一條能分開就沒碰撞
    const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
    const d = [B.c[0]-A.c[0], B.c[1]-A.c[1], B.c[2]-A.c[2]];
    const axes = [...A.u, ...B.u];
    for (const a of A.u) for (const b of B.u) { const x = cross(a, b); if (dot(x, x) > 1e-9) axes.push(x); }
    for (const L of axes) {
        const rA = A.h[0]*Math.abs(dot(A.u[0], L)) + A.h[1]*Math.abs(dot(A.u[1], L)) + A.h[2]*Math.abs(dot(A.u[2], L));
        const rB = B.h[0]*Math.abs(dot(B.u[0], L)) + B.h[1]*Math.abs(dot(B.u[1], L)) + B.h[2]*Math.abs(dot(B.u[2], L));
        if (Math.abs(dot(d, L)) > rA + rB) return false;
    }
    return true;
}
function mkObbMinZ(o) { return o.c[2] - (o.h[0]*Math.abs(o.u[0][2]) + o.h[1]*Math.abs(o.u[1][2]) + o.h[2]*Math.abs(o.u[2][2])); }

// 回傳所有碰撞盒(世界座標)與分組資訊,畫面上也可以拿來顯示
function modularCollisionBoxes(assembly, thetas, fk) {
    fk = fk || modularFK(assembly, thetas);
    const boxes = [];
    let g = 0, mi = 0, ri = 0, li = 0, ci = 0, prevMotor = -1;
    // 直接插在某顆馬達面上的零件,跟那顆馬達自己的 pivot 不檢查(插頭在插孔裡、底板貼著 body,實際不會撞到 pivot)
    const push = (T, list, info, off) => list.forEach(b => boxes.push({ ...info, exemptPivot: prevMotor, obb: mkObb(T, b, off) }));
    push(mkIdentity(), PART_GEO.base.collision, { group: 0, name: '底座' });   // 底座盒本來就是世界座標,不能再乘 base.T(之前多轉了 90°)
    assembly.parts.forEach((part, pi) => {
        if (part.kind === 'motor') {
            push(fk.inputFrames[ri], PART_GEO.motor.collision.pivot, { group: g, name: `馬達${mi + 1} pivot`, joint: mi, side: 'a', isPivot: true });
            g++;
            prevMotor = -1;
            push(fk.motorFrames[ri], PART_GEO.motor.collision.body, { group: g, name: `馬達${mi + 1}`, joint: mi, side: 'b' });
            prevMotor = mi;   // 下一個零件是直接插在這顆馬達上的
            ri++; mi++;
            return;
        } else if (part.kind === 'linear') {
            const L = fk.linearFrames[li], lib = PART_LIBRARY.linearModules[part.key];
            push(L.Tin, PART_GEO[part.key].collision.static, { group: g, name: `${lib.name}(支架)`, joint: mi, side: 'a' });
            g++;
            prevMotor = -1;
            push(L.Tin, PART_GEO[part.key].collision.moving, { group: g, name: `${lib.name}(齒條)`, joint: mi, side: 'b' }, lib.travelDir.map(v => v * L.d));
            li++; mi++;
            return;
        } else {
            const T = fk.partFrames[ci++].T;
            const geo = part.kind === 'ee' ? PART_GEO[part.key] : PART_GEO[part.key];
            if (geo && geo.collision) push(T, geo.collision, { group: g, name: partNameOf(part) });
        }
        prevMotor = -1;
    });
    return boxes;
}
function partNameOf(part) {
    if (part.kind === 'motor') return PART_LIBRARY.motor.name;
    if (part.kind === 'linear') return PART_LIBRARY.linearModules[part.key].name;
    if (part.kind === 'ee') return PART_LIBRARY.endEffectors[part.key].name;
    return PART_LIBRARY.connectors[part.key].name;
}
// 主函式:{ collided, hits:[{a, b}] , boxes }
function modularCollision(assembly, thetas, opts = {}) {
    const boxes = modularCollisionBoxes(assembly, thetas, opts.fk);
    const hits = [], seen = new Set();
    for (let i = 0; i < boxes.length; i++) {
        const A = boxes[i];
        if (A.group > 0 && mkObbMinZ(A.obb) < MK_FLOOR_Z + 0.5) {
            if (!seen.has(A.name + '|地面')) { seen.add(A.name + '|地面'); hits.push({ a: A.name, b: '地面' }); }
            if (opts.firstOnly) return { collided: true, hits, boxes };
        }
        for (let j = i + 1; j < boxes.length; j++) {
            const B = boxes[j];
            if (A.group === B.group) continue;
            if (A.joint !== undefined && A.joint === B.joint && A.side !== B.side) continue; // 同一關節兩側本來就接觸
            if ((A.isPivot && B.exemptPivot === A.joint) || (B.isPivot && A.exemptPivot === B.joint)) continue; // 直接插在馬達面上的零件 ↔ 該馬達 pivot
            if (!mkObbOverlap(A.obb, B.obb)) continue;
            const key = A.name + '|' + B.name;
            if (!seen.has(key)) { seen.add(key); hits.push({ a: A.name, b: B.name }); }
            if (opts.firstOnly) return { collided: true, hits, boxes };
        }
    }
    return { collided: hits.length > 0, hits, boxes };
}

if (typeof module !== 'undefined') module.exports = { PART_GEO, modularCollision, modularCollisionBoxes, PART_LIBRARY, CURRENT_ARM_ASSEMBLY, SCARA_ASSEMBLY, MAX_MOTORS, motorCount, jointInfo, linearDisplacement, canAppend, modularFK, modularIK, analyzeDOF, mkMul, mkTr, mkRz, mkRy, mkChain };
