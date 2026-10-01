// 曲轴初级传动系设计模数 m = 0.12
const M_GEAR = 0.12;
// 副轴汇总系轻量紧凑模数 m_sum = 0.10 (外齿圈、副轴汇总大齿轮、MG2小齿轮)
const M_SUM_GEAR = 0.10;
// 行星排内部专用紧凑模数 m_int = 0.09 (保证齿圈拥有坚固的径向环壁厚度与高重合度)
const M_PLANET_INT = 0.09;
const C_WHEEL = 1.916; // 160/60-R17 周长 (米)

// 齿数定义
const Z_ICE = 40;         // 曲轴主动齿轮 40T
const Z_CARRIER_IN = 68;  // 行星架输入大齿轮 68T
const I_ICE_C = Z_CARRIER_IN / Z_ICE; // 68 / 40 = 1.70

const Z_SUN = 18;         // 太阳轮 18T (内部模数 0.09)
const Z_PLANET = 18;      // 3个行星轮 18T (内部模数 0.09)
const Z_RING_INT = 54;    // 齿圈内齿 54T (内部模数 0.09)
const Z_RING_EXT = 60;    // 齿圈外齿 60T (外部模数 0.10)
const K_PLANETARY = Z_RING_INT / Z_SUN; // 54 / 18 = 3.0

const Z_SUM_GEAR = 60;    // 副轴汇总大齿轮 60T
const Z_MG2_PINION = 22;  // MG2 驱动小齿轮 22T
const I_MG2 = Z_SUM_GEAR / Z_MG2_PINION; // 60 / 22 = 2.72727
const I_R_TO_SUM = Z_SUM_GEAR / Z_RING_EXT; // 60 / 60 = 1.0

const Z_FRONT_SPROCKET = 12; // 终传小链轮 12T
const Z_REAR_SPROCKET = 44;  // 后轮大链盘 44T
const I_CHAIN = Z_REAR_SPROCKET / Z_FRONT_SPROCKET; // 44 / 12 = 3.66667

// 各齿轮节圆分度圆半径 (R = m * Z / 2)
const R_ICE = (M_GEAR * Z_ICE) / 2;               // 2.40
const R_CARRIER_IN = (M_GEAR * Z_CARRIER_IN) / 2; // 4.08
// 行星排内部基于 m_int = 0.09 严格同心: R_sun(0.81) + 2*R_planet(0.81) = R_ring_int(2.43)
const R_SUN = (M_PLANET_INT * Z_SUN) / 2;         // 0.81
const R_PLANET = (M_PLANET_INT * Z_PLANET) / 2;   // 0.81
const R_RING_INT = (M_PLANET_INT * Z_RING_INT) / 2; // 2.43
// 外齿圈(60T)、副轴汇总大齿轮(60T)及MG2小齿轮(22T)基于轻量模数 m_sum = 0.10
const R_RING_EXT = (M_SUM_GEAR * Z_RING_EXT) / 2;     // 3.00
const R_SUM = (M_SUM_GEAR * Z_SUM_GEAR) / 2;          // 3.00
const R_MG2 = (M_SUM_GEAR * Z_MG2_PINION) / 2;        // 1.10
const R_FRONT_SPROCKET = 1.10;                    // 12T 小链轮有效分度半径
// 后轮大链盘严格与小链轮具有相同的理论齿距节圆半径 (4.033333...)
const R_REAR_SPROCKET = (R_FRONT_SPROCKET * Z_REAR_SPROCKET) / Z_FRONT_SPROCKET;
const CHAIN_PITCH = (Math.PI * 2 * R_FRONT_SPROCKET) / Z_FRONT_SPROCKET; // 节距 (约 0.57595865)
const NUM_CHAIN_LINKS = 120; // 标准整节数 (120 节)
const CHAIN_TARGET_LEN = NUM_CHAIN_LINKS * CHAIN_PITCH; // 链条理论闭环总长

// 解析求解使双轮公切线与包络总长严格等于 NUM_CHAIN_LINKS * CHAIN_PITCH 的中心距 (真车后轮链条张紧点)
function solveExactChainCenterDistance(r1, r2, targetLen) {
  let low = 15, high = 40;
  for (let i = 0; i < 60; i++) {
    const mid = (low + high) / 2;
    const alpha = Math.asin(Math.max(-1, Math.min(1, (r2 - r1) / mid)));
    const topLen = Math.sqrt(Math.max(0, mid * mid - (r2 - r1) * (r2 - r1)));
    const arcFront = r1 * (Math.PI - 2 * alpha);
    const arcRear = r2 * (Math.PI + 2 * alpha);
    const len = topLen * 2 + arcFront + arcRear;
    if (len < targetLen) low = mid; else high = mid;
  }
  return (low + high) / 2;
}

// 轴系中心坐标常量 (行星架齿轮连带行星排、MG1电机保持上移位置 Y=3.0)
const POS_PSD_X = 0;
const POS_PSD_Y = 3.0;

// 1. 曲轴箱与曲轴输出齿轮 (40T) 与 行星架输入大齿轮 (68T) 中心距
const DIST_ICE = R_ICE + R_CARRIER_IN; // 2.40 + 4.08 = 6.48
// 发动机高度恢复到默认位置 (基于原默认基准 Y=2.4 计算的高度，中心距严格保持 6.48 啮合)
const ANG_ICE_DEFAULT = (162 * Math.PI) / 180;
const POS_ICE_Y = 2.4 + Math.sin(ANG_ICE_DEFAULT) * DIST_ICE;
const DY_ICE = POS_ICE_Y - POS_PSD_Y;
const POS_ICE_X = POS_PSD_X - Math.sqrt(DIST_ICE * DIST_ICE - DY_ICE * DY_ICE);

// 2. 齿圈外齿 (60T) 与 副轴汇总大齿轮 (60T) 中心距
const DIST_SUM = R_RING_EXT + R_SUM; // 3.00 + 3.00 = 6.00
// 副轴高度恢复到默认位置 (原默认 Y=2.4+0.35=2.75，中心距严格保持 6.00 啮合)
const POS_SUM_Y = 2.4 + 0.35;
const DY_SUM = POS_SUM_Y - POS_PSD_Y;
const DX_SUM = Math.sqrt(DIST_SUM * DIST_SUM - DY_SUM * DY_SUM);
const POS_SUM_X = POS_PSD_X + DX_SUM;

// 3. MG2 驱动齿轮 (22T) 与 副轴大齿轮 (60T) 中心距 (随副轴高度恢复默认水平，保持避开 MG1)
const DIST_MG2 = R_MG2 + R_SUM; // 1.10 + 3.00 = 4.10
const DX_MG2_REL = -0.40;
const DY_MG2_REL = Math.sqrt(DIST_MG2 * DIST_MG2 - DX_MG2_REL * DX_MG2_REL);
const POS_MG2_X = POS_SUM_X + DX_MG2_REL;
const POS_MG2_Y = POS_SUM_Y + DY_MG2_REL;

// 后轮轴位置（严格根据 120 节链条张紧计算自动对齐，保证总周长等于 120 * CHAIN_PITCH）
const DIST_CHAIN_EXACT = solveExactChainCenterDistance(R_FRONT_SPROCKET, R_REAR_SPROCKET, CHAIN_TARGET_LEN);
const POS_REAR_WHEEL_Y = 3.0;
const DY_CHAIN = POS_REAR_WHEEL_Y - POS_SUM_Y;
const POS_REAR_WHEEL_X = POS_SUM_X + Math.sqrt(DIST_CHAIN_EXACT * DIST_CHAIN_EXACT - DY_CHAIN * DY_CHAIN);

// 运行动态变量 (默认设定为 100km/h 巡航与 4000rpm 发动机工况)
let currentSpeed = 100; // km/h
let currentIceRpm = 4000; // rpm
let isExploded = false;
let explodeFactor = 0;

// 累加绝对转角
const angles = {
  wheel: 0,
  sumShaft: 0,
  mg2: 0,
  ring: 0,
  carrier: 0,
  ice: 0,
  sun: 0,
  planetRel: 0
};

// 初始咬合静态相位偏置
const initialPhase = {
  ice: 0,
  carrier: 0,
  sun: 0,
  ring: 0,
  sumShaft: 0,
  mg2: 0,
  rearSprocket: 0,
  planets: [0, 0, 0]
};

function calculateKinematics(speedKmh, iceRpm) {
  const wheelRpm = (speedKmh * 1000) / (60 * C_WHEEL);
  const sumShaftRpm = wheelRpm * I_CHAIN;
  const mg2Rpm = sumShaftRpm * I_MG2;
  const ringRpm = sumShaftRpm * I_R_TO_SUM;
  const carrierRpm = iceRpm / I_ICE_C;
  const mg1Rpm = (1 + K_PLANETARY) * carrierRpm - K_PLANETARY * ringRpm;

  return { wheelRpm, sumShaftRpm, mg2Rpm, ringRpm, carrierRpm, mg1Rpm };
}

// 精确外齿齿轮几何体 (保持完整实心盘面机械质感，支持多组同色系高协调度旋转定位 Marker)
function createExternalGearMesh(teeth, pitchRadius, thickness, material, isHollow, innerRadius, addMarker = false, markerRadius = 0.20, markerDist = 0, markerColor = null, numMarkers = 3) {
  const shape = new THREE.Shape();
  const m = (2 * pitchRadius) / teeth;
  const ha = 1.0 * m;
  const hf = 1.2 * m;
  const rOuter = pitchRadius + ha;
  const rRoot = Math.max(0.1, pitchRadius - hf);
  const step = (Math.PI * 2) / teeth;

  for (let i = 0; i < teeth; i++) {
    const a0 = i * step;
    const a1 = a0 + step * 0.24;
    const a2 = a0 + step * 0.44;
    const a3 = a0 + step * 0.68;

    const p0x = Math.cos(a0) * rRoot;
    const p0y = Math.sin(a0) * rRoot;
    const p1x = Math.cos(a1) * rOuter;
    const p1y = Math.sin(a1) * rOuter;
    const p2x = Math.cos(a2) * rOuter;
    const p2y = Math.sin(a2) * rOuter;
    const p3x = Math.cos(a3) * rRoot;
    const p3y = Math.sin(a3) * rRoot;

    if (i === 0) shape.moveTo(p0x, p0y);
    else shape.lineTo(p0x, p0y);
    shape.lineTo(p1x, p1y);
    shape.lineTo(p2x, p2y);
    shape.lineTo(p3x, p3y);
  }
  shape.closePath();

  // 中心大孔 (仅在轴穿孔时镂空)
  if (isHollow && innerRadius > 0) {
    const hole = new THREE.Path();
    hole.absarc(0, 0, innerRadius, 0, Math.PI * 2, true);
    shape.holes.push(hole);
  }

  const extrudeSettings = {
    depth: thickness,
    bevelEnabled: true,
    bevelSegments: 2,
    steps: 1,
    bevelSize: 0.02,
    bevelThickness: 0.02
  };

  const geometry = new THREE.ExtrudeGeometry(shape, extrudeSettings);
  geometry.center();
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;

  // 添加工业正时/旋转定位 Marker 阵列 (多个点环向均布，颜色与齿轮同色系且保持清晰明暗/光泽区分度)
  if (addMarker && markerDist > 0) {
    let actualColor;
    if (markerColor !== null && markerColor !== undefined) {
      actualColor = new THREE.Color(markerColor);
    } else if (material && material.color) {
      actualColor = material.color.clone();
      const hsl = {};
      actualColor.getHSL(hsl);
      if (hsl.l > 0.55) {
        // 浅色金属（如银白铝、亮钢）：取同色系深钛钢/暗铬色 (保持金属感且高反差)
        hsl.l = Math.max(0.26, hsl.l - 0.42);
        hsl.s = Math.min(0.20, hsl.s * 0.7);
      } else {
        // 深色/中色金属（如阳极氧化蓝）：取同色系亮色阳极金属色
        hsl.l = Math.min(0.80, hsl.l + 0.32);
        hsl.s = Math.min(0.90, hsl.s * 1.1);
      }
      actualColor.setHSL(hsl.h, hsl.s, hsl.l);
    } else {
      actualColor = new THREE.Color(0x64748b);
    }

    const mMat = new THREE.MeshStandardMaterial({
      color: actualColor,
      metalness: 0.92,
      roughness: 0.18,
      emissive: actualColor,
      emissiveIntensity: 0.06 // 极微弱自发光，维持真实金属质感同时防止在背光暗影处死黑
    });

    const mGeo = new THREE.CylinderGeometry(markerRadius, markerRadius, 0.10, 24);
    mGeo.rotateX(Math.PI / 2);

    const count = Math.max(1, numMarkers || 3);
    for (let k = 0; k < count; k++) {
      const ang = (k * Math.PI * 2) / count;
      const mx = Math.cos(ang) * markerDist;
      const my = Math.sin(ang) * markerDist;

      // 正面 Marker 销柱
      const markerF = new THREE.Mesh(mGeo, mMat);
      markerF.position.set(mx, my, thickness / 2 + 0.03);
      mesh.add(markerF);

      // 背面对应 Marker 销柱
      const markerB = new THREE.Mesh(mGeo, mMat);
      markerB.position.set(mx, my, -thickness / 2 - 0.03);
      mesh.add(markerB);
    }
  }

  return mesh;
}

// 制作内外双齿齿圈：外有 60 个外齿(m=0.10)，内有 54 个内齿(m=0.09)，环壁兼具紧凑轻量与高抗扭刚度
function createDualGearedRingMesh(intTeeth, intPitchRadius, extTeeth, extPitchRadius, thickness, material) {
  const shape = new THREE.Shape();
  const mExt = (2 * extPitchRadius) / extTeeth;
  const mInt = (2 * intPitchRadius) / intTeeth;
  const haExt = 1.0 * mExt;
  const hfExt = 1.2 * mExt;
  const haInt = 1.0 * mInt;
  const hfInt = 1.2 * mInt;

  // 1. 外齿轮廓（外向凸出）
  const rOuterMax = extPitchRadius + haExt;
  const rOuterRoot = extPitchRadius - hfExt;
  const extStep = (Math.PI * 2) / extTeeth;

  for (let i = 0; i < extTeeth; i++) {
    const a0 = i * extStep;
    const a1 = a0 + extStep * 0.24;
    const a2 = a0 + extStep * 0.44;
    const a3 = a0 + extStep * 0.68;

    const p0x = Math.cos(a0) * rOuterRoot;
    const p0y = Math.sin(a0) * rOuterRoot;
    const p1x = Math.cos(a1) * rOuterMax;
    const p1y = Math.sin(a1) * rOuterMax;
    const p2x = Math.cos(a2) * rOuterMax;
    const p2y = Math.sin(a2) * rOuterMax;
    const p3x = Math.cos(a3) * rOuterRoot;
    const p3y = Math.sin(a3) * rOuterRoot;

    if (i === 0) shape.moveTo(p0x, p0y);
    else shape.lineTo(p0x, p0y);
    shape.lineTo(p1x, p1y);
    shape.lineTo(p2x, p2y);
    shape.lineTo(p3x, p3y);
  }
  shape.closePath();

  // 2. 内齿轮廓（内孔路径顺时针，具有充裕的环形实体壁厚）
  const hole = new THREE.Path();
  const rIntRoot = intPitchRadius + hfInt; // 内齿齿根（靠外）
  const rIntTip = intPitchRadius - haInt;  // 内齿齿尖（向内深入）
  const intStep = (Math.PI * 2) / intTeeth;

  for (let j = 0; j < intTeeth; j++) {
    const b0 = j * intStep;
    const b1 = b0 + intStep * 0.24;
    const b2 = b0 + intStep * 0.44;
    const b3 = b0 + intStep * 0.68;

    const q0x = Math.cos(b0) * rIntRoot;
    const q0y = Math.sin(b0) * rIntRoot;
    const q1x = Math.cos(b1) * rIntTip;
    const q1y = Math.sin(b1) * rIntTip;
    const q2x = Math.cos(b2) * rIntTip;
    const q2y = Math.sin(b2) * rIntTip;
    const q3x = Math.cos(b3) * rIntRoot;
    const q3y = Math.sin(b3) * rIntRoot;

    if (j === 0) hole.moveTo(q0x, q0y);
    else hole.lineTo(q0x, q0y);
    hole.lineTo(q1x, q1y);
    hole.lineTo(q2x, q2y);
    hole.lineTo(q3x, q3y);
  }
  hole.closePath();
  shape.holes.push(hole);

  const extrudeSettings = {
    depth: thickness,
    bevelEnabled: true,
    bevelSegments: 2,
    steps: 1,
    bevelSize: 0.02,
    bevelThickness: 0.02
  };

  const geometry = new THREE.ExtrudeGeometry(shape, extrudeSettings);
  geometry.center();
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

// 后轮真实通风打孔刹车盘
function createBrakeDiscMesh(radius, thickness) {
  const discShape = new THREE.Shape();
  discShape.absarc(0, 0, radius, 0, Math.PI * 2, false);

  const centerHole = new THREE.Path();
  centerHole.absarc(0, 0, radius * 0.48, 0, Math.PI * 2, true);
  discShape.holes.push(centerHole);

  const numRings = 2;
  const holesPerRing = 12;
  for (let r = 0; r < numRings; r++) {
    const ventRadius = radius * (0.64 + r * 0.18);
    const holeR = 0.18;
    const angOffset = (r * Math.PI) / holesPerRing;
    for (let h = 0; h < holesPerRing; h++) {
      const ang = (h * Math.PI * 2) / holesPerRing + angOffset;
      const hx = Math.cos(ang) * ventRadius;
      const hy = Math.sin(ang) * ventRadius;
      const ventHole = new THREE.Path();
      ventHole.absarc(hx, hy, holeR, 0, Math.PI * 2, true);
      discShape.holes.push(ventHole);
    }
  }

  const extrudeSettings = {
    depth: thickness,
    bevelEnabled: true,
    bevelSegments: 1,
    steps: 1,
    bevelSize: 0.02,
    bevelThickness: 0.02
  };

  const geometry = new THREE.ExtrudeGeometry(discShape, extrudeSettings);
  geometry.center();
  const brakeMat = new THREE.MeshStandardMaterial({
    color: 0x94a3b8,
    metalness: 0.95,
    roughness: 0.28
  });
  return new THREE.Mesh(geometry, brakeMat);
}

// 程序化生成摩托车现代跑车真空胎贴图 (极简现代刀锋流线槽，仅8组疏朗大气，彻底消除蛇皮碎纹)
function createTireTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 1024;
  canvas.height = 512;
  const ctx = canvas.getContext('2d');

  // 1. 哑光高级质感深黑底色
  ctx.fillStyle = '#0a0d12';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // 2. 现代超级跑车半热熔胎经典风格：仅 8 组优雅舒朗的大气刀锋主排水槽，胎冠中心保持平整半热熔光头带
  const numPatterns = 8;
  const stepX = canvas.width / numPatterns;

  for (let i = 0; i < numPatterns; i++) {
    const baseX = i * stepX;

    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 14;
    ctx.lineCap = 'round';

    // 组 1: 左侧流线型斜向刀锋槽 (从 Y=244 优雅向外撇向胎肩 Y=195)
    ctx.beginPath();
    ctx.moveTo(baseX + 20, 244);
    ctx.quadraticCurveTo(baseX + 60, 226, baseX + 100, 195);
    ctx.stroke();

    // 组 2: 右侧流线型斜向刀锋槽 (左右错开交替，从 Y=268 撇向胎肩 Y=317)
    ctx.beginPath();
    ctx.moveTo(baseX + stepX * 0.5 + 20, 268);
    ctx.quadraticCurveTo(baseX + stepX * 0.5 + 60, 286, baseX + stepX * 0.5 + 100, 317);
    ctx.stroke();

    // 凹槽内壁极微弱质感反光 (仅在槽内单侧淡淡勾勒，绝无双边斑马线杂乱感)
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(baseX + 22, 243);
    ctx.quadraticCurveTo(baseX + 62, 225, baseX + 98, 196);
    ctx.moveTo(baseX + stepX * 0.5 + 22, 269);
    ctx.quadraticCurveTo(baseX + stepX * 0.5 + 62, 287, baseX + stepX * 0.5 + 98, 316);
    ctx.stroke();
  }

  const tex = new THREE.CanvasTexture(canvas);
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  tex.offset.set(0, 0.5);
  return tex;
}

// 共享链条基础几何体单例
let sharedChainPlateGeo = null;
let sharedChainRollerGeo = null;
let sharedChainPinOuterGeo = null;
let sharedChainPinInnerGeo = null;

function getChainGeometries() {
  if (sharedChainPlateGeo) return;
  const pitch = CHAIN_PITCH;
  const pinR = 0.11;
  const rollerR = 0.21;
  const innerGap = 0.46;
  const outerGap = 0.72;

  const plateShape = new THREE.Shape();
  const w = 0.42;
  plateShape.moveTo(-pitch / 2, -w / 2);
  plateShape.lineTo(pitch / 2, -w / 2);
  plateShape.absarc(pitch / 2, 0, w / 2, -Math.PI / 2, Math.PI / 2, false);
  plateShape.lineTo(-pitch / 2, w / 2);
  plateShape.absarc(-pitch / 2, 0, w / 2, Math.PI / 2, -Math.PI / 2, false);

  const h1 = new THREE.Path();
  h1.absarc(-pitch / 2, 0, pinR * 0.95, 0, Math.PI * 2, true);
  const h2 = new THREE.Path();
  h2.absarc(pitch / 2, 0, pinR * 0.95, 0, Math.PI * 2, true);
  plateShape.holes.push(h1, h2);

  const extrudeSettings = {
    depth: 0.08,
    bevelEnabled: true,
    bevelSegments: 1,
    steps: 1,
    bevelSize: 0.012,
    bevelThickness: 0.012
  };
  sharedChainPlateGeo = new THREE.ExtrudeGeometry(plateShape, extrudeSettings);
  sharedChainPlateGeo.center();

  sharedChainRollerGeo = new THREE.CylinderGeometry(rollerR, rollerR, innerGap * 0.96, 16);
  sharedChainRollerGeo.rotateX(Math.PI / 2);

  sharedChainPinOuterGeo = new THREE.CylinderGeometry(pinR, pinR, outerGap + 0.16, 14);
  sharedChainPinOuterGeo.rotateX(Math.PI / 2);

  sharedChainPinInnerGeo = new THREE.CylinderGeometry(pinR, pinR, innerGap + 0.16, 14);
  sharedChainPinInnerGeo.rotateX(Math.PI / 2);
}

function createRollerLinkMesh(isOuter, plateMat, pinMat) {
  getChainGeometries();
  const linkGroup = new THREE.Group();
  const pitch = CHAIN_PITCH;
  const innerGap = 0.46;
  const outerGap = 0.72;
  const zSpread = isOuter ? outerGap : innerGap;

  const plateLeft = new THREE.Mesh(sharedChainPlateGeo, plateMat);
  plateLeft.position.z = zSpread / 2;
  linkGroup.add(plateLeft);

  const plateRight = new THREE.Mesh(sharedChainPlateGeo, plateMat);
  plateRight.position.z = -zSpread / 2;
  linkGroup.add(plateRight);

  // 每个链节仅保留前销轴孔 (-pitch/2) 处的滚子与贯通销轴；
  // 后销轴孔由相邻下一链节的前铰接构件自然穿接，彻底消除 100% 几何重叠与共面 Z-fighting
  const roller = new THREE.Mesh(sharedChainRollerGeo, pinMat);
  roller.position.set(-pitch / 2, 0, 0);
  linkGroup.add(roller);

  const pinMesh = new THREE.Mesh(sharedChainPinOuterGeo, pinMat);
  pinMesh.position.set(-pitch / 2, 0, 0);
  linkGroup.add(pinMesh);

  return linkGroup;
}

let scene, camera, renderer, controls;
let assembly, iceGroup, carrierAssembly, ringGearMesh, sunGear, mg1Group, mg2Group, mg2Pinion, sumGear, frontSprocket, wheelGroup, outShaftGroup, chainGroup, rearSprocketMesh, chainHitProxy;
let icePrimaryGear;
const planetMeshes = [];
const chainLinks = [];
let numLinks = NUM_CHAIN_LINKS;
let chainPathGeom = null;
let currentKine = null;

const interactiveMeshes = [];
const partsToExplode = [];

function isDescendant(parent, child) {
  let cur = child;
  while (cur) {
    if (cur === parent) return true;
    cur = cur.parent;
  }
  return false;
}

// 链条两轮公切线与包络几何精密预计算（严密外公切线几何闭环）
function computeChainGeometry() {
  const x1 = POS_SUM_X, y1 = POS_SUM_Y, r1 = R_FRONT_SPROCKET; // 12T 小链轮 (r1)
  const x2 = POS_REAR_WHEEL_X, y2 = POS_REAR_WHEEL_Y, r2 = R_REAR_SPROCKET; // 44T 大链盘 (r2)
  const dx = x2 - x1;
  const dy = y2 - y1;
  const d = Math.sqrt(dx * dx + dy * dy);
  const baseAngle = Math.atan2(dy, dx);

  // 外公切线法线与两轮连心线法向的偏移角 (r2 > r1)
  const alpha = Math.asin(Math.max(-1, Math.min(1, (r2 - r1) / d)));
  const topTangentLen = Math.sqrt(Math.max(0, d * d - (r2 - r1) * (r2 - r1)));

  // 上下公切线的外法线极角：
  // 上切线法线朝上偏左：baseAngle + PI/2 + alpha
  // 下切线法线朝下偏左：baseAngle - PI/2 - alpha
  const aTop = baseAngle + Math.PI / 2 + alpha;
  const aBot = baseAngle - Math.PI / 2 - alpha;

  // 小轮 (前) 接触包角必定小于 180° (PI - 2*alpha)
  // 大轮 (后) 接触包角必定大于 180° (PI + 2*alpha)
  const arcFront = r1 * (Math.PI - 2 * alpha);
  const arcRear = r2 * (Math.PI + 2 * alpha);
  const totalLen = topTangentLen * 2 + arcFront + arcRear;

  // 预计算上/下公切线世界切点坐标与切向角 (避免动画每帧对 120 节链条重复进行数千次三角函数运算)
  const xTopStart = x2 + Math.cos(aTop) * r2;
  const yTopStart = y2 + Math.sin(aTop) * r2;
  const xTopEnd = x1 + Math.cos(aTop) * r1;
  const yTopEnd = y1 + Math.sin(aTop) * r1;
  const rotTop = Math.atan2(yTopEnd - yTopStart, xTopEnd - xTopStart);

  const xBotStart = x1 + Math.cos(aBot) * r1;
  const yBotStart = y1 + Math.sin(aBot) * r1;
  const xBotEnd = x2 + Math.cos(aBot) * r2;
  const yBotEnd = y2 + Math.sin(aBot) * r2;
  const rotBot = Math.atan2(yBotEnd - yBotStart, xBotEnd - xBotStart);

  return {
    x1, y1, r1, x2, y2, r2, d, baseAngle, alpha,
    topLen: topTangentLen,
    arcFront,
    botLen: topTangentLen,
    arcRear,
    totalLen,
    aTop, aBot,
    xTopStart, yTopStart, xTopEnd, yTopEnd, rotTop,
    xBotStart, yBotStart, xBotEnd, yBotEnd, rotBot
  };
}

// 创建轻量化链条射线碰撞代理网格 (仅约 380 个低阶三角面，替代遍历 120 组链节、数百个复杂 Extrude 网格)
function createChainHitProxy(g) {
  const points = [];
  const nTangent = 8;
  for (let i = 0; i <= nTangent; i++) {
    const t = i / nTangent;
    points.push(new THREE.Vector3(
      g.xTopStart + t * (g.xTopEnd - g.xTopStart),
      g.yTopStart + t * (g.yTopEnd - g.yTopStart),
      0
    ));
  }
  const nFront = 10;
  for (let i = 1; i <= nFront; i++) {
    const t = i / nFront;
    const ang = g.aTop + t * (Math.PI - 2 * g.alpha);
    points.push(new THREE.Vector3(
      g.x1 + Math.cos(ang) * g.r1,
      g.y1 + Math.sin(ang) * g.r1,
      0
    ));
  }
  for (let i = 1; i <= nTangent; i++) {
    const t = i / nTangent;
    points.push(new THREE.Vector3(
      g.xBotStart + t * (g.xBotEnd - g.xBotStart),
      g.yBotStart + t * (g.yBotEnd - g.yBotStart),
      0
    ));
  }
  const nRear = 14;
  for (let i = 1; i < nRear; i++) {
    const t = i / nRear;
    const ang = g.aBot + t * (Math.PI + 2 * g.alpha);
    points.push(new THREE.Vector3(
      g.x2 + Math.cos(ang) * g.r2,
      g.y2 + Math.sin(ang) * g.r2,
      0
    ));
  }

  const curve = new THREE.CatmullRomCurve3(points, true);
  const tubeGeo = new THREE.TubeGeometry(curve, 32, 0.45, 6, true);
  const proxyMat = new THREE.MeshBasicMaterial({
    colorWrite: false,
    depthWrite: false
  });
  return new THREE.Mesh(tubeGeo, proxyMat);
}

function initSceneAndModels() {
  const container = document.getElementById('canvas-container');
  const width = container.clientWidth || window.innerWidth;
  const height = container.clientHeight || window.innerHeight;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a1120);

  camera = new THREE.PerspectiveCamera(38, width / height, 0.1, 2000);
  camera.position.set(24, 18, 36);

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: "high-performance" });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.35;
  container.appendChild(renderer.domElement);

  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.06;
  controls.target.set(10, 3, 0);
  controls.maxPolarAngle = Math.PI / 2 + 0.08;

  // 根据视口屏幕宽度自动计算并应用最佳等比缩放
  updateResponsiveScale();

  // 工业级高亮布光
  const ambientLight = new THREE.AmbientLight(0xffffff, 1.35);
  scene.add(ambientLight);

  const mainSun = new THREE.DirectionalLight(0xfffdf5, 1.85);
  mainSun.position.set(30, 45, 35);
  mainSun.castShadow = true;
  mainSun.shadow.mapSize.width = 2048;
  mainSun.shadow.mapSize.height = 2048;
  mainSun.shadow.bias = -0.0001;
  // 拓展阴影视锥体，完整覆盖曲轴发动机前端(X=-12)至后轮全宽总成(X=36)
  mainSun.shadow.camera.left = -18;
  mainSun.shadow.camera.right = 38;
  mainSun.shadow.camera.top = 22;
  mainSun.shadow.camera.bottom = -16;
  mainSun.shadow.camera.near = 10;
  mainSun.shadow.camera.far = 120;
  scene.add(mainSun);

  const fillLightFront = new THREE.DirectionalLight(0x38bdf8, 1.0);
  fillLightFront.position.set(-25, 20, 30);
  scene.add(fillLightFront);

  const backLight = new THREE.DirectionalLight(0xa78bfa, 0.9);
  backLight.position.set(20, 15, -35);
  scene.add(backLight);

  const bottomBounceLight = new THREE.DirectionalLight(0xe2e8f0, 0.65);
  bottomBounceLight.position.set(0, -25, 0);
  scene.add(bottomBounceLight);

  // Floor Grid (与 160/60-R17 真实后轮外半径 11.30 紧密贴合: 3.0 - 11.30 = -8.30)
  const grid = new THREE.GridHelper(100, 100, 0x334155, 0x1e293b);
  grid.position.y = -8.30;
  scene.add(grid);

  // 材质库
  const tireTexture = createTireTexture();
  const materials = {
    aluminumCase: new THREE.MeshStandardMaterial({ color: 0xd8dee9, metalness: 0.85, roughness: 0.30 }),
    aluminumMachined: new THREE.MeshStandardMaterial({ color: 0xe5e9f0, metalness: 0.90, roughness: 0.18 }),
    iceHead: new THREE.MeshStandardMaterial({ color: 0xb48ead, metalness: 0.82, roughness: 0.28 }),
    carrier: new THREE.MeshStandardMaterial({ color: 0x0284c7, metalness: 0.84, roughness: 0.25 }),
    carrierPlate: new THREE.MeshStandardMaterial({ color: 0x38bdf8, metalness: 0.86, roughness: 0.22 }),
    sun: new THREE.MeshStandardMaterial({ color: 0xef4444, metalness: 0.85, roughness: 0.25 }),
    planet: new THREE.MeshStandardMaterial({ color: 0xf1f5f9, metalness: 0.96, roughness: 0.13 }),
    ring: new THREE.MeshStandardMaterial({ color: 0x10b981, metalness: 0.85, roughness: 0.25 }),
    mg1Body: new THREE.MeshStandardMaterial({ color: 0xef4444, metalness: 0.85, roughness: 0.25 }),
    mg2Body: new THREE.MeshStandardMaterial({ color: 0xfbbf24, metalness: 0.80, roughness: 0.28 }),
    mg2Pinion: new THREE.MeshStandardMaterial({ color: 0xf59e0b, metalness: 0.86, roughness: 0.24 }),
    sumShaft: new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.92, roughness: 0.20 }),
    steelShaft: new THREE.MeshStandardMaterial({ color: 0xd1d5db, metalness: 0.95, roughness: 0.14 }),
    bearingSteel: new THREE.MeshStandardMaterial({ color: 0xe2e8f0, metalness: 0.92, roughness: 0.16 }),
    chainOuter: new THREE.MeshStandardMaterial({ color: 0xf97316, metalness: 0.85, roughness: 0.28 }),
    chainInner: new THREE.MeshStandardMaterial({ color: 0x64748b, metalness: 0.90, roughness: 0.30 }),
    chainPin: new THREE.MeshStandardMaterial({ color: 0x0f172a, metalness: 0.95, roughness: 0.15 }),
    chainSprocket: new THREE.MeshStandardMaterial({ color: 0xe2e8f0, metalness: 0.88, roughness: 0.24 }),
    axleBlock: new THREE.MeshStandardMaterial({ color: 0x0284c7, metalness: 0.82, roughness: 0.28 }),
    rubber: new THREE.MeshStandardMaterial({
      map: tireTexture,
      bumpMap: tireTexture,
      bumpScale: 0.22,
      color: 0x0a0c10,
      roughness: 0.82,
      metalness: 0.08
    }),
    wheelRim: new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.92, roughness: 0.22 })
  };

  assembly = new THREE.Group();
  scene.add(assembly);

  // --- A. 500cc 并列双缸发动机 (ICE) ---
  iceGroup = new THREE.Group();
  iceGroup.position.set(POS_ICE_X, POS_ICE_Y, 0);

  const crankcaseMainGeo = new THREE.CylinderGeometry(2.70, 2.70, 6.4, 32);
  crankcaseMainGeo.rotateX(Math.PI / 2);
  const crankcaseMain = new THREE.Mesh(crankcaseMainGeo, materials.aluminumCase);
  crankcaseMain.position.set(0, 0, -0.2);
  crankcaseMain.castShadow = true;
  iceGroup.add(crankcaseMain);

  const sumpGeo = new THREE.BoxGeometry(4.2, 1.8, 5.8);
  const sump = new THREE.Mesh(sumpGeo, materials.aluminumCase);
  sump.position.set(0.2, -2.5, -0.2);
  sump.rotation.z = -0.20;
  iceGroup.add(sump);

  const crankDeckSkirtGeo = new THREE.BoxGeometry(4.8, 2.2, 6.2);
  const crankDeckSkirt = new THREE.Mesh(crankDeckSkirtGeo, materials.aluminumCase);
  crankDeckSkirt.rotation.z = 25 * (Math.PI / 180);
  crankDeckSkirt.position.set(-0.55, 1.25, -0.2);
  iceGroup.add(crankDeckSkirt);

  const cylinderBank = new THREE.Group();
  cylinderBank.rotation.z = 25 * (Math.PI / 180);
  cylinderBank.position.set(-2.15 * Math.sin(25 * Math.PI / 180), 2.15 * Math.cos(25 * Math.PI / 180), -0.2);

  const deckGeo = new THREE.BoxGeometry(5.0, 0.45, 6.2);
  const deckMesh = new THREE.Mesh(deckGeo, materials.aluminumCase);
  deckMesh.position.y = 0.225;
  cylinderBank.add(deckMesh);

  const zBores = [-1.45, 1.45];
  const sharedFinGeo = new THREE.CylinderGeometry(2.35, 2.35, 0.14, 32);
  zBores.forEach((zb) => {
    const singleCylGeo = new THREE.CylinderGeometry(1.95, 2.05, 5.4, 32);
    const singleCyl = new THREE.Mesh(singleCylGeo, materials.aluminumCase);
    singleCyl.position.set(0, 3.15, zb);
    singleCyl.castShadow = true;
    cylinderBank.add(singleCyl);

    for (let f = 0; f < 9; f++) {
      const fin = new THREE.Mesh(sharedFinGeo, materials.aluminumMachined);
      fin.position.set(0, 1.35 + f * 0.50, zb);
      cylinderBank.add(fin);
    }

    const plugGeo = new THREE.CylinderGeometry(0.42, 0.42, 0.6, 20);
    const plug = new THREE.Mesh(plugGeo, materials.steelShaft);
    plug.position.set(0, 6.55, zb);
    cylinderBank.add(plug);
  });

  const headGeo = new THREE.BoxGeometry(5.1, 1.6, 6.4);
  const headMesh = new THREE.Mesh(headGeo, materials.iceHead);
  headMesh.position.y = 6.15;
  cylinderBank.add(headMesh);

  const camCoverGeo = new THREE.CylinderGeometry(0.8, 0.8, 6.2, 24);
  camCoverGeo.rotateX(Math.PI / 2);
  const camCoverF = new THREE.Mesh(camCoverGeo, materials.aluminumMachined);
  camCoverF.position.set(-1.1, 7.05, 0);
  cylinderBank.add(camCoverF);

  const camCoverR = new THREE.Mesh(camCoverGeo, materials.aluminumMachined);
  camCoverR.position.set(1.1, 7.05, 0);
  cylinderBank.add(camCoverR);

  iceGroup.add(cylinderBank);

  const crankSnoutGeo = new THREE.CylinderGeometry(1.3, 1.3, 1.6, 32);
  crankSnoutGeo.rotateX(Math.PI / 2);
  const crankSnout = new THREE.Mesh(crankSnoutGeo, materials.aluminumCase);
  crankSnout.position.set(0, 0, 2.2);
  iceGroup.add(crankSnout);

  const crankShaftGeo = new THREE.CylinderGeometry(0.62, 0.62, 2.4, 24);
  crankShaftGeo.rotateX(Math.PI / 2);
  const crankShaft = new THREE.Mesh(crankShaftGeo, materials.steelShaft);
  crankShaft.position.set(0, 0, 3.1);
  iceGroup.add(crankShaft);

  // 曲轴输出齿轮 (40T实心盘面，带3组同色系深钛钢旋转定位 Marker)
  icePrimaryGear = createExternalGearMesh(Z_ICE, R_ICE, 0.85, materials.aluminumMachined, false, 0, true, 0.18, 1.45, 0x64748b, 3);
  icePrimaryGear.position.set(0, 0, 4.0);
  iceGroup.add(icePrimaryGear);
  assembly.add(iceGroup);

  // --- B. MG1 调速电机 ---
  mg1Group = new THREE.Group();
  mg1Group.position.set(POS_PSD_X, POS_PSD_Y, -1.0);

  // MG1 额定 10kW / 峰值 20kW，作为直径基准: R=2.45，轴向有效长度 4.4
  const mg1Geo = new THREE.CylinderGeometry(2.45, 2.45, 4.4, 32);
  mg1Geo.rotateX(Math.PI / 2);
  const mg1Housing = new THREE.Mesh(mg1Geo, materials.mg1Body);
  mg1Housing.castShadow = true;
  mg1Group.add(mg1Housing);

  const sharedMg1RibGeo = new THREE.CylinderGeometry(2.53, 2.53, 0.16, 32);
  sharedMg1RibGeo.rotateX(Math.PI / 2);
  for (let m = 0; m < 5; m++) {
    const rib = new THREE.Mesh(sharedMg1RibGeo, materials.aluminumMachined);
    rib.position.set(0, 0, -1.6 + m * 0.8);
    mg1Group.add(rib);
  }

  // MG1 / 太阳轮贯通轴：贯穿行星架中心轴孔，由齿轮轴孔内的套接轴承支承，左端伸出输入大齿轮端面；
  // 右端同样穿出电机壳体后端面 0.5，用以接入变速箱壳体支承轴承（轴承本体无需绘制）
  const mg1ShaftGeo = new THREE.CylinderGeometry(0.52, 0.52, 8.65, 24); // 组内 Z: -2.70 ~ 5.95
  mg1ShaftGeo.rotateX(Math.PI / 2);
  const mg1Shaft = new THREE.Mesh(mg1ShaftGeo, materials.steelShaft);
  mg1Shaft.position.set(0, 0, 1.625);
  mg1Group.add(mg1Shaft);

  // 贯通轴前端轴承锁紧螺母
  const shaftNutGeo = new THREE.CylinderGeometry(0.80, 0.80, 0.30, 6);
  shaftNutGeo.rotateX(Math.PI / 2);
  const shaftNut = new THREE.Mesh(shaftNutGeo, materials.steelShaft);
  shaftNut.position.set(0, 0, 6.05);
  mg1Group.add(shaftNut);
  assembly.add(mg1Group);

  // 太阳轮 (18T, 内部模数 0.09, R=0.81)
  // 太阳轮中心挖空至 R=0.52，正好等于 MG1 电机轴直径 1.04 的一半，轴贯穿其中且端面处严丝合缝
  sunGear = createExternalGearMesh(Z_SUN, R_SUN, 0.88, materials.sun, true, 0.52);
  sunGear.position.set(POS_PSD_X, POS_PSD_Y, 2.4);
  assembly.add(sunGear);

  // --- C. 行星架总成 (68T 大齿轮 + 刚性传动筒 + 镂空双挡板 + 3个 18T 行星轮) ---
  carrierAssembly = new THREE.Group();
  carrierAssembly.position.set(POS_PSD_X, POS_PSD_Y, 0);

  // 行星架输入大齿轮 (68T，轴孔 R=1.07 与轴承外径仅留 0.02 装配间隙，端面处无可见缝隙)
  const carrierInputGear = createExternalGearMesh(Z_CARRIER_IN, R_CARRIER_IN, 0.85, materials.carrier, true, 1.07, true, 0.24, 2.70, 0x38bdf8, 3);
  carrierInputGear.position.set(0, 0, 4.0);
  carrierAssembly.add(carrierInputGear);

  // 压装于轴孔内的深沟球轴承 (外径适度扩大至 1.05，为标准截面系列；内孔仍套装 R=0.52 贯通轴)
  const bearingShape = new THREE.Shape();
  bearingShape.absarc(0, 0, 1.05, 0, Math.PI * 2, false);
  const bearingBore = new THREE.Path();
  bearingBore.absarc(0, 0, 0.54, 0, Math.PI * 2, true);
  bearingShape.holes.push(bearingBore);
  const carrierBearingGeo = new THREE.ExtrudeGeometry(bearingShape, {
    depth: 0.84,
    bevelEnabled: true,
    bevelSegments: 1,
    steps: 1,
    bevelSize: 0.02,
    bevelThickness: 0.02,
    curveSegments: 48
  });
  carrierBearingGeo.center();
  const carrierBearing = new THREE.Mesh(carrierBearingGeo, materials.bearingSteel);
  carrierBearing.position.set(0, 0, 4.0);
  carrierAssembly.add(carrierBearing);

  const carrierSpiderGroup = new THREE.Group();
  carrierSpiderGroup.position.set(0, 0, 2.4);

  const planetCenterDist = R_SUN + R_PLANET; // 0.81 + 0.81 = 1.62

  const flangeShape = new THREE.Shape();
  flangeShape.absarc(0, 0, 2.3, 0, Math.PI * 2, false);
  const flangeHole = new THREE.Path();
  flangeHole.absarc(0, 0, 1.05, 0, Math.PI * 2, true);
  flangeShape.holes.push(flangeHole);

  // 挡板开设 3 个行星销轴安装通孔，销轴贯穿显露
  for (let i = 0; i < 3; i++) {
    const pinAngle = (i * Math.PI * 2) / 3;
    const px = Math.cos(pinAngle) * planetCenterDist;
    const py = Math.sin(pinAngle) * planetCenterDist;
    const pinHole = new THREE.Path();
    pinHole.absarc(px, py, 0.21, 0, Math.PI * 2, true);
    flangeShape.holes.push(pinHole);
  }

  const sharedFlangeGeo = new THREE.ExtrudeGeometry(flangeShape, {
    depth: 0.14,
    bevelEnabled: true,
    bevelSegments: 1,
    steps: 1,
    bevelSize: 0.02,
    bevelThickness: 0.02
  });
  sharedFlangeGeo.center();

  // 适当加大挡板间距 (Z=±0.55, 内侧净间距 0.96)，避免紧夹厚度 0.78 的行星齿轮造成干涉
  const outerFlange = new THREE.Mesh(sharedFlangeGeo, materials.carrierPlate);
  outerFlange.position.set(0, 0, 0.55);
  carrierSpiderGroup.add(outerFlange);

  const innerFlange = new THREE.Mesh(sharedFlangeGeo, materials.carrierPlate);
  innerFlange.position.set(0, 0, -0.55);
  carrierSpiderGroup.add(innerFlange);

  const pinGeo = new THREE.CylinderGeometry(0.20, 0.20, 1.36, 24);
  pinGeo.rotateX(Math.PI / 2);

  // 销轴端盖/轴承限位凸缘几何体 (凸显于挡板外表面)
  const pinCapGeo = new THREE.CylinderGeometry(0.27, 0.27, 0.06, 24);
  pinCapGeo.rotateX(Math.PI / 2);

  for (let i = 0; i < 3; i++) {
    const pinAngle = (i * Math.PI * 2) / 3;
    const px = Math.cos(pinAngle) * planetCenterDist;
    const py = Math.sin(pinAngle) * planetCenterDist;

    // 贯穿行星齿轮与内外双挡板的实心销轴
    const pin = new THREE.Mesh(pinGeo, materials.steelShaft);
    pin.position.set(px, py, 0);
    carrierSpiderGroup.add(pin);

    // 外侧挡板凸显的销轴端盖
    const capOuter = new THREE.Mesh(pinCapGeo, materials.steelShaft);
    capOuter.position.set(px, py, 0.63);
    carrierSpiderGroup.add(capOuter);

    // 内侧挡板凸显的销轴端盖
    const capInner = new THREE.Mesh(pinCapGeo, materials.steelShaft);
    capInner.position.set(px, py, -0.63);
    carrierSpiderGroup.add(capInner);

    // 行星齿轮 (厚度适度收放至 0.78，与双挡板内侧保持舒适空隙，彻底消除挤夹干涉)
    const planetMesh = createExternalGearMesh(Z_PLANET, R_PLANET, 0.78, materials.planet, false, 0);
    planetMesh.position.set(px, py, 0);
    carrierSpiderGroup.add(planetMesh);
    planetMeshes.push({ mesh: planetMesh, index: i, angleOffset: pinAngle });
  }

  carrierAssembly.add(carrierSpiderGroup);

  // 行星架刚性连接三柱：精确位于 3 个行星销轴所在位置 (R=1.62, 0°/120°/240°)，
  // 柱径 R=0.35 明显大于销轴 R=0.20，可旋入紧固并将输入大齿轮刚性联结至双挡板
  // 轴向: Z=2.93~3.63，下端嵌入外侧挡板 (前端面 Z=3.04) 内 0.11，上端嵌入输入齿轮背面 (Z=3.555) 内 0.075
  const sharedStrutGeo = new THREE.CylinderGeometry(0.35, 0.35, 0.70, 24);
  sharedStrutGeo.rotateX(Math.PI / 2);
  for (let i = 0; i < 3; i++) {
    const strutAngle = (i * Math.PI * 2) / 3;
    const sx = Math.cos(strutAngle) * planetCenterDist;
    const sy = Math.sin(strutAngle) * planetCenterDist;
    const strut = new THREE.Mesh(sharedStrutGeo, materials.carrier);
    strut.position.set(sx, sy, 3.28);
    strut.castShadow = true;
    carrierAssembly.add(strut);
  }

  assembly.add(carrierAssembly);

  // --- D. 一体化轻量高刚度双齿齿圈 (内齿 54T, 外齿 60T) ---
  ringGearMesh = createDualGearedRingMesh(Z_RING_INT, R_RING_INT, Z_RING_EXT, R_RING_EXT, 0.88, materials.ring);
  ringGearMesh.position.set(POS_PSD_X, POS_PSD_Y, 2.4);
  assembly.add(ringGearMesh);

  // --- E. MG2 主驱动电机 ---
  mg2Group = new THREE.Group();
  mg2Group.position.set(POS_MG2_X, POS_MG2_Y, 0);

  // MG2 额定 15kW / 峰值 30kW，功率为 MG1 的 1.5 倍:
  // 按峰值功率 ∝ 有效铁芯体积 ∝ r²·L (相同磁热负荷与相近峰值转速)，同长度 4.4 下 R = 2.45·√1.5 ≈ 3.00
  const mg2Geo = new THREE.CylinderGeometry(3.00, 3.00, 4.4, 32);
  mg2Geo.rotateX(Math.PI / 2);
  const mg2Housing = new THREE.Mesh(mg2Geo, materials.mg2Body);
  mg2Housing.position.set(0, 0, -1.0);
  mg2Housing.castShadow = true;
  mg2Group.add(mg2Housing);

  const sharedMg2RibGeo = new THREE.CylinderGeometry(3.08, 3.08, 0.18, 32);
  sharedMg2RibGeo.rotateX(Math.PI / 2);
  for (let m = 0; m < 6; m++) {
    const rib = new THREE.Mesh(sharedMg2RibGeo, materials.aluminumMachined);
    rib.position.set(0, 0, -2.5 + m * 0.62);
    mg2Group.add(rib);
  }

  // MG2 贯通轴：左端穿出壳体并经小齿轮驱动副轴；右端同样穿出电机壳体后端面 0.5，
  // 用以接入变速箱壳体支承轴承（轴承本体无需绘制）
  const mg2ShaftGeo = new THREE.CylinderGeometry(0.52, 0.52, 6.5, 24); // 组内 Z: -3.70 ~ 2.80
  mg2ShaftGeo.rotateX(Math.PI / 2);
  const mg2Shaft = new THREE.Mesh(mg2ShaftGeo, materials.steelShaft);
  mg2Shaft.position.set(0, 0, -0.45);
  mg2Group.add(mg2Shaft);

  // MG2 驱动小齿轮 (22T实心盘面，带单组深青铜/钛金旋转定位 Marker)
  mg2Pinion = createExternalGearMesh(Z_MG2_PINION, R_MG2, 0.85, materials.mg2Pinion, false, 0, true, 0.10, 0.74, 0x78350f, 1);
  mg2Pinion.position.set(0, 0, 2.4);
  mg2Group.add(mg2Pinion);
  assembly.add(mg2Group);

  // --- F. 副轴动力汇总总成 (大齿轮 60T、阶梯轴与 12T 小链轮完全刚性一体化) ---
  outShaftGroup = new THREE.Group();
  outShaftGroup.position.set(POS_SUM_X, POS_SUM_Y, 0);

  // 1. 副轴汇总大齿轮 (60T实心盘面)，带3组同色系深钛钢旋转定位 Marker
  sumGear = createExternalGearMesh(Z_SUM_GEAR, R_SUM, 0.88, materials.aluminumMachined, false, 0, true, 0.20, 2.05, 0x64748b, 3);
  sumGear.position.set(0, 0, 2.4);
  outShaftGroup.add(sumGear);

  const sumFlangeGeo = new THREE.CylinderGeometry(1.4, 1.6, 0.4, 32);
  sumFlangeGeo.rotateX(Math.PI / 2);
  const sumFlange = new THREE.Mesh(sumFlangeGeo, materials.steelShaft);
  sumFlange.position.set(0, 0, 2.85);
  outShaftGroup.add(sumFlange);

  // 2. 贯穿阶梯传动实心轴
  const sumShaftRodGeo = new THREE.CylinderGeometry(0.72, 0.72, 2.1, 24);
  sumShaftRodGeo.rotateX(Math.PI / 2);
  const sumShaftRod = new THREE.Mesh(sumShaftRodGeo, materials.steelShaft);
  sumShaftRod.position.set(0, 0, 3.45);
  outShaftGroup.add(sumShaftRod);

  // 3. 终传 12T 小链轮，与副轴完全一体化花键紧固
  frontSprocket = createExternalGearMesh(Z_FRONT_SPROCKET, R_FRONT_SPROCKET, 0.36, materials.chainSprocket, false, 0);
  frontSprocket.position.set(0, 0, 4.2);
  outShaftGroup.add(frontSprocket);

  // 4. 小链轮锁紧轴端压板螺栓
  const lockNutGeo = new THREE.CylinderGeometry(0.55, 0.55, 0.28, 6);
  lockNutGeo.rotateX(Math.PI / 2);
  const lockNut = new THREE.Mesh(lockNutGeo, materials.steelShaft);
  lockNut.position.set(0, 0, 4.45);
  outShaftGroup.add(lockNut);

  assembly.add(outShaftGroup);

  // --- G. 全尺寸 500cc 摩托车后轮总成 (严格 1:1 还原真实 160/60-R17 跑车胎与 17寸轮辋规格) ---
  wheelGroup = new THREE.Group();
  wheelGroup.position.set(POS_REAR_WHEEL_X, POS_REAR_WHEEL_Y, 0);

  // 160/60-R17 真实跑车胎: 外径 624mm (R_outer=11.30), 断面宽 160mm (W=5.80)
  const tireGeo = new THREE.TorusGeometry(9.56, 1.74, 32, 80);
  const tire = new THREE.Mesh(tireGeo, materials.rubber);
  tire.scale.set(1.0, 1.0, 1.67);
  tire.castShadow = true;
  wheelGroup.add(tire);

  // 轮胎内圈完全封闭贴合衬垫 (彻底封死圆环管内径拱起造成的空隙，实现轮胎内侧与轮毂 100% 紧密无缝贴合)
  const tireLinerGeo = new THREE.CylinderGeometry(8.05, 8.05, 5.75, 48, 1, true);
  tireLinerGeo.rotateX(Math.PI / 2);
  const tireLiner = new THREE.Mesh(tireLinerGeo, materials.rubber);
  wheelGroup.add(tireLiner);

  // 17 英寸高刚性轻量铝合金锻造实心轮辋 (全宽 5.80，外翻轮唇外径 8.50，完全包裹扣合胎圈，彻底消除缝隙与单薄感)
  const rimPoints = [
    new THREE.Vector2(8.50, -2.88), // 外侧凸出加厚翻边轮唇 (高出轮胎内圈并紧紧扣住胎侧)
    new THREE.Vector2(8.40, -2.55),
    new THREE.Vector2(8.10, -2.35), // 胎圈座支撑过盈贴合面
    new THREE.Vector2(7.50, -1.30),
    new THREE.Vector2(7.35, 0.0),   // 中央加强沉槽
    new THREE.Vector2(7.50, 1.30),
    new THREE.Vector2(8.10, 2.35),  // 胎圈座支撑过盈贴合面
    new THREE.Vector2(8.40, 2.55),
    new THREE.Vector2(8.50, 2.88),  // 内侧凸出加厚翻边轮唇 (全宽 5.76，与轮胎侧面绝对严密贴合)
    new THREE.Vector2(7.80, 2.65),  // 实体轮圈侧壁
    new THREE.Vector2(7.10, 2.20),  // 实体厚壁内表面 (厚度达 0.40 单位)
    new THREE.Vector2(7.00, 0.0),
    new THREE.Vector2(7.10, -2.20),
    new THREE.Vector2(7.80, -2.65)
  ];
  const rimGeo = new THREE.LatheGeometry(rimPoints, 48);
  rimGeo.rotateX(Math.PI / 2);
  const rim = new THREE.Mesh(rimGeo, materials.wheelRim);
  rim.castShadow = true;
  rim.receiveShadow = true;
  wheelGroup.add(rim);

  const hubGeo = new THREE.CylinderGeometry(1.6, 1.6, 5.5, 24);
  hubGeo.rotateX(Math.PI / 2);
  const hubMesh = new THREE.Mesh(hubGeo, materials.wheelRim);
  wheelGroup.add(hubMesh);

  // 5 根高强度铝合金轮辐，自中心轮毂(R=1.6)强固延伸贯穿融入轮辋实体厚壁内表面(R=7.10)
  const spokeLen = 5.50;
  const spokeRadiusMid = 4.35;
  const sharedSpokeGeo = new THREE.BoxGeometry(0.75, spokeLen, 0.95);
  for (let s = 0; s < 5; s++) {
    const spokeAng = (s * Math.PI * 2) / 5;
    const spoke = new THREE.Mesh(sharedSpokeGeo, materials.wheelRim);
    spoke.position.set(
      Math.cos(spokeAng) * spokeRadiusMid,
      Math.sin(spokeAng) * spokeRadiusMid,
      0
    );
    spoke.rotation.z = spokeAng - Math.PI / 2;
    wheelGroup.add(spoke);
  }

  const sprocketCarrierGeo = new THREE.CylinderGeometry(2.6, 2.8, 1.5, 24);
  sprocketCarrierGeo.rotateX(Math.PI / 2);
  const sprocketCarrier = new THREE.Mesh(sprocketCarrierGeo, materials.aluminumMachined);
  sprocketCarrier.position.set(0, 0, 3.45);
  wheelGroup.add(sprocketCarrier);

  // 后轮 44T 链盘 (实心盘面，带4组同色系暗铬钢旋转定位铆钉 Marker)
  rearSprocketMesh = createExternalGearMesh(Z_REAR_SPROCKET, R_REAR_SPROCKET, 0.36, materials.chainSprocket, true, 2.2, true, 0.22, 3.15, 0x475569, 4);
  rearSprocketMesh.position.set(0, 0, 4.2);
  wheelGroup.add(rearSprocketMesh);

  const brakeDisc = createBrakeDiscMesh(4.2, 0.18);
  brakeDisc.position.set(0, 0, -3.4);
  wheelGroup.add(brakeDisc);

  // 刹车盘安装座总成：轮毂(R=1.6) -> 锥形过渡鼓 -> 法兰盘 -> 6 组贯穿螺栓，
  // 将通风碟盘与轮毂/轮轴做实刚性连接，彻底消除碟盘悬空的中心空洞
  const discDrumGeo = new THREE.CylinderGeometry(1.62, 2.10, 0.65, 32); // 小端朝 +Z 接轮毂，大端朝 -Z 接法兰
  discDrumGeo.rotateX(Math.PI / 2);
  const discDrum = new THREE.Mesh(discDrumGeo, materials.aluminumMachined);
  discDrum.position.set(0, 0, -2.85); // Z: -3.175 ~ -2.525，覆盖并与轮毂端面(-2.75)接实
  discDrum.castShadow = true;
  wheelGroup.add(discDrum);

  const discFlangeGeo = new THREE.CylinderGeometry(2.35, 2.35, 0.16, 32);
  discFlangeGeo.rotateX(Math.PI / 2);
  const discFlange = new THREE.Mesh(discFlangeGeo, materials.aluminumMachined);
  discFlange.position.set(0, 0, -3.27); // Z: -3.35 ~ -3.19，紧贴并微压入碟盘内侧面(-3.29)
  discFlange.castShadow = true;
  wheelGroup.add(discFlange);

  // 6 组法兰螺栓：贯穿法兰盘与碟盘内圈，头部凸出于碟盘外端面(-3.51)，形成可见的紧固连接
  const discBoltGeo = new THREE.CylinderGeometry(0.19, 0.19, 0.44, 16);
  discBoltGeo.rotateX(Math.PI / 2);
  const discBoltHeadGeo = new THREE.CylinderGeometry(0.28, 0.28, 0.07, 6);
  discBoltHeadGeo.rotateX(Math.PI / 2);
  const discBoltRadius = 2.28; // 落在碟盘实体环面 2.016~2.508 之间，避开通风孔
  for (let b = 0; b < 6; b++) {
    const boltAng = (b * Math.PI * 2) / 6;
    const bx = Math.cos(boltAng) * discBoltRadius;
    const by = Math.sin(boltAng) * discBoltRadius;

    const bolt = new THREE.Mesh(discBoltGeo, materials.steelShaft);
    bolt.position.set(bx, by, -3.38); // Z: -3.60 ~ -3.16
    wheelGroup.add(bolt);

    const boltHead = new THREE.Mesh(discBoltHeadGeo, materials.steelShaft);
    boltHead.position.set(bx, by, -3.605); // Z: -3.64 ~ -3.57
    wheelGroup.add(boltHead);
  }

  const rearAxleGeo = new THREE.CylinderGeometry(0.55, 0.55, 12.8, 24);
  rearAxleGeo.rotateX(Math.PI / 2);
  const rearAxle = new THREE.Mesh(rearAxleGeo, materials.steelShaft);
  wheelGroup.add(rearAxle);

  const nutGeo = new THREE.CylinderGeometry(0.85, 0.85, 0.6, 6);
  nutGeo.rotateX(Math.PI / 2);
  const axleNut = new THREE.Mesh(nutGeo, materials.steelShaft);
  axleNut.position.set(0, 0, 6.35);
  wheelGroup.add(axleNut);

  const axleHead = new THREE.Mesh(nutGeo, materials.steelShaft);
  axleHead.position.set(0, 0, -6.35);
  wheelGroup.add(axleHead);

  // --- H. 摩托车铝合金后摇臂总成 (自适应精确后轴位置) ---
  const swingarmGroup = new THREE.Group();
  const armMat = new THREE.MeshStandardMaterial({ color: 0x64748b, metalness: 0.85, roughness: 0.35 });

  const pivotX = 11.8;
  const armLength = POS_REAR_WHEEL_X - pivotX;
  const armMidX = (pivotX + POS_REAR_WHEEL_X) / 2;
  const armGeo = new THREE.BoxGeometry(armLength, 1.35, 1.0);

  const armZ = 5.3;
  const leftArm = new THREE.Mesh(armGeo, armMat);
  leftArm.position.set(armMidX, 3.0, armZ);
  swingarmGroup.add(leftArm);

  const rightArm = new THREE.Mesh(armGeo, armMat);
  rightArm.position.set(armMidX, 3.0, -armZ);
  swingarmGroup.add(rightArm);

  const pivotGeo = new THREE.CylinderGeometry(0.85, 0.85, armZ * 2 + 1.0, 24);
  pivotGeo.rotateX(Math.PI / 2);
  const pivot = new THREE.Mesh(pivotGeo, armMat);
  pivot.position.set(pivotX, 2.8, 0.0);
  swingarmGroup.add(pivot);

  const braceGeo = new THREE.BoxGeometry(1.6, 1.2, (armZ - 0.5) * 2);
  const brace = new THREE.Mesh(braceGeo, armMat);
  brace.position.set(pivotX + 1.8, 2.9, 0.0);
  swingarmGroup.add(brace);

  assembly.add(swingarmGroup);
  assembly.add(wheelGroup);

  // --- I. 真实 520 滚子链条 (严格 120 节闭环张紧，零累积误差) ---
  chainPathGeom = computeChainGeometry();
  numLinks = NUM_CHAIN_LINKS;

  chainGroup = new THREE.Group();
  chainGroup.position.set(0, 0, 4.2);

  for (let c = 0; c < numLinks; c++) {
    const isOuter = (c % 2 === 0);
    const link = createRollerLinkMesh(isOuter, isOuter ? materials.chainOuter : materials.chainInner, materials.chainPin);
    chainGroup.add(link);
    chainLinks.push(link);
  }

  // 轻量化高响应度射线检测代理 (仅约 380 个低阶三角面，代替对 120 组链节、数百个 Extrude 齿形零件的深度递归遍历)
  chainHitProxy = createChainHitProxy(chainPathGeom);
  chainGroup.add(chainHitProxy);

  assembly.add(chainGroup);

  // --- 全套齿轮解析相位精密对齐 ---
  // 齿形几何规律：齿顶中心位于 0.34 * step，齿槽底中心位于 0.84 * step (-0.16 * step)

  // 1. 齿圈外齿 (60T) 与 副轴汇总大齿轮 (60T) 空间切向基准
  const phi_ring_to_sum = Math.atan2(POS_SUM_Y - POS_PSD_Y, POS_SUM_X - POS_PSD_X);
  const step_ring_ext = (Math.PI * 2) / Z_RING_EXT;
  const step_sum = (Math.PI * 2) / Z_SUM_GEAR;
  initialPhase.ring = phi_ring_to_sum - 0.34 * step_ring_ext;
  initialPhase.sumShaft = (phi_ring_to_sum + Math.PI) - 0.84 * step_sum;

  // 2. 行星排内部 (54T内齿圈、行星架、18T行星轮、18T太阳轮) 严格闭环对齐
  const step_ring_int = (Math.PI * 2) / Z_RING_INT;
  const step_sun = (Math.PI * 2) / Z_SUN;
  const step_planet = (Math.PI * 2) / Z_PLANET;

  // 行星架转角与齿圈内齿槽几何相位严格锁死绑定
  initialPhase.carrier = initialPhase.ring + 0.84 * step_ring_int;

  // 太阳轮转角严格以行星架空间角度为参考基准，确保三颗行星轮连线上太阳轮均为齿槽
  initialPhase.sun = initialPhase.carrier - 0.84 * step_sun;

  // 行星轮在朝向太阳轮处呈齿顶 (0.34 * step_planet)；因 Z_PLANET=18 为偶数齿，背向太阳轮外侧同为齿顶入齿圈内齿槽
  planetMeshes.forEach(p => {
    initialPhase.planets[p.index] = (p.angleOffset + Math.PI) - 0.34 * step_planet;
  });

  // 3. 曲轴主动齿轮 (40T) 与 行星架输入大齿轮 (68T) 互补齿位严密对齐 (齿顶0.34与齿槽0.84互补闭环)
  const phi_carrier_to_ice = Math.atan2(POS_ICE_Y - POS_PSD_Y, POS_ICE_X - POS_PSD_X);
  const step_ice = (Math.PI * 2) / Z_ICE;
  const step_carrier_in = (Math.PI * 2) / Z_CARRIER_IN;
  let angleOnCarrierIn = (phi_carrier_to_ice - initialPhase.carrier) % step_carrier_in;
  if (angleOnCarrierIn < 0) angleOnCarrierIn += step_carrier_in;
  const u_carrier_in = angleOnCarrierIn / step_carrier_in;
  let u_ice = (1.18 - u_carrier_in) % 1.0;
  if (u_ice < 0) u_ice += 1.0;
  initialPhase.ice = (phi_carrier_to_ice + Math.PI) - u_ice * step_ice;

  // 4. MG2 小齿轮 (22T) 与 副轴大齿轮 (60T) 互补齿位严密对齐
  const phi_sum_to_mg2 = Math.atan2(POS_MG2_Y - POS_SUM_Y, POS_MG2_X - POS_SUM_X);
  const step_mg2 = (Math.PI * 2) / Z_MG2_PINION;
  let sumToothAngleAtMg2 = (phi_sum_to_mg2 - initialPhase.sumShaft) % step_sum;
  if (sumToothAngleAtMg2 < 0) sumToothAngleAtMg2 += step_sum;
  const u_sum = sumToothAngleAtMg2 / step_sum;
  let u_mg2 = (1.18 - u_sum) % 1.0;
  if (u_mg2 < 0) u_mg2 += 1.0;
  initialPhase.mg2 = (phi_sum_to_mg2 + Math.PI) - u_mg2 * step_mg2;

  // 应用初始咬合解析相位
  icePrimaryGear.rotation.z = initialPhase.ice;
  carrierAssembly.rotation.z = initialPhase.carrier;
  ringGearMesh.rotation.z = initialPhase.ring;
  outShaftGroup.rotation.z = initialPhase.sumShaft;
  mg2Pinion.rotation.z = initialPhase.mg2;
  sunGear.rotation.z = initialPhase.sun;
  planetMeshes.forEach(p => {
    p.mesh.rotation.z = initialPhase.planets[p.index];
  });

  // 5. 后轮大链盘与链条下切点初始咬合解析相位精密锁死
  const s_rear_entry = chainPathGeom.topLen + chainPathGeom.arcFront + chainPathGeom.botLen;
  const step_rear = (Math.PI * 2) / Z_REAR_SPROCKET;
  const stepFront = (Math.PI * 2) / Z_FRONT_SPROCKET;
  const actualAtS0 = (chainPathGeom.aTop - chainPathGeom.topLen / R_FRONT_SPROCKET - initialPhase.sumShaft);
  let angleDiff = (0.34 * stepFront - actualAtS0) % stepFront;
  if (angleDiff < 0) angleDiff += stepFront;
  const phaseAlignOffset = angleDiff * R_FRONT_SPROCKET;

  initialPhase.rearSprocket = chainPathGeom.aBot + (phaseAlignOffset - s_rear_entry) / R_REAR_SPROCKET - 0.34 * step_rear;
  if (rearSprocketMesh) {
    rearSprocketMesh.rotation.z = initialPhase.rearSprocket;
  }

  // 交互部件元数据（精简核心参数与传动关系）
  interactiveMeshes.push(
    { mesh: icePrimaryGear, title: "曲轴输出齿轮 (40T)", desc: "40T (m=0.12)，驱动行星架输入大齿轮" },
    { mesh: iceGroup, title: "500cc 发动机 (ICE)", desc: "双缸 DOHC，出轴直驱 40T 主动齿轮" },
    { mesh: carrierInputGear, title: "行星架输入大齿轮 (68T)", desc: "68T (m=0.12)，初级减速比 1.70" },
    { mesh: outerFlange, title: "行星架承托挡板", desc: "刚性夹持 3 个 18T 行星轮" },
    { mesh: sunGear, title: "行星排太阳轮 (18T)", desc: "18T (m=0.09)，直连 MG1 进行调速分流" },
    { mesh: ringGearMesh, title: "双齿齿圈 (内54T/外60T)", desc: "内齿咬合行星轮，外齿啮合副轴大齿轮" },
    { mesh: mg1Housing, title: "MG1 电机 (调速/发电)", desc: "同轴直连太阳轮，实现 e-CVT 调速" },
    { mesh: mg2Housing, title: "MG2 主驱动电机", desc: "大扭矩主驱电机，直驱副轴" },
    { mesh: mg2Pinion, title: "MG2 驱动齿轮 (22T)", desc: "22T (m=0.10)，减速比 2.73 驱动副轴" },
    { mesh: sumGear, title: "副轴汇总大齿轮 (60T)", desc: "60T (m=0.10)，汇聚齿圈与 MG2 动力" },
    { mesh: frontSprocket, title: "终传小链轮 (12T)", desc: "12T，与后链盘构成 3.67 终传比" },
    { mesh: chainHitProxy, title: "520 滚子链条 (120节)", desc: "标准 520 规格，传递终传动力" },
    { mesh: rearSprocketMesh, title: "后轮终传大链盘 (44T)", desc: "44T，终传比 3.67，驱动后轮" },
    { mesh: rearAxle, title: "后轮穿心轴", desc: "高强度合金轴，刚性支承后轮" },
    { mesh: wheelGroup, title: "160/60-R17 后轮总成", desc: "17寸铝合金轮辋与跑车宽胎" },
    { mesh: swingarmGroup, title: "铝合金后摇臂", desc: "双臂悬架结构，支承后轮轴心" }
  );
  planetMeshes.forEach((p, idx) => {
    interactiveMeshes.unshift({ mesh: p.mesh, title: `行星轮 #${idx + 1} (18T)`, desc: "18T (m=0.09)，公转自转分流动力" });
  });

  // 爆炸图设置（行星排与电机各层分解展示，链轮与链条保持位置不动）
  // 太阳轮与 MG1 电机为同轴刚性直连，分解位移必须完全一致，始终保持套在 MG1 电机轴上
  const EXPLODE_MG1_DELTA = -10;
  partsToExplode.push(
    { group: carrierAssembly, axis: 'z', delta: 12 },
    { group: ringGearMesh, axis: 'z', delta: 6 },
    { group: mg1Group, axis: 'z', delta: EXPLODE_MG1_DELTA },
    { group: sunGear, axis: 'z', delta: EXPLODE_MG1_DELTA },
    { group: mg2Group, axis: 'y', delta: 6 }
  );
  partsToExplode.forEach(item => {
    item.origPos = item.group.position.clone();
  });

  // 初始化运动学并渲染首次 UI
  currentKine = calculateKinematics(currentSpeed, currentIceRpm);
  updateUI(currentKine);

  setupInteractions();
  window.addEventListener('resize', onWindowResize, false);
  animate();
}

function updateChainLinks(distanceTraveled) {
  if (!chainPathGeom) return;
  const g = chainPathGeom;

  // 链条与小链轮齿槽完全居中闭环解析：
  // 每个链节由前后相距 pitch 的两滚子组成，要使双滚子落入相邻齿槽弧底(0.84*step)，
  // 链节中心在小链轮上必须精准对齐齿顶中心(0.34*stepFront)。
  const stepFront = (Math.PI * 2) / Z_FRONT_SPROCKET;
  const targetRelativeAngle = 0.34 * stepFront;
  const actualAtS0 = (g.aTop - g.topLen / R_FRONT_SPROCKET - initialPhase.sumShaft);
  let angleDiff = (targetRelativeAngle - actualAtS0) % stepFront;
  if (angleDiff < 0) angleDiff += stepFront;
  const phaseAlignOffset = angleDiff * R_FRONT_SPROCKET;

  for (let k = 0; k < numLinks; k++) {
    // 严格以 CHAIN_PITCH 为物理间距 (g.totalLen === numLinks * CHAIN_PITCH 严丝合缝)
    const nominalOffset = k * CHAIN_PITCH;
    let s = (nominalOffset + distanceTraveled + phaseAlignOffset) % g.totalLen;
    if (s < 0) s += g.totalLen;

    let px = 0, py = 0, rotZ = 0;

    // 段 1: 上公切线（后轮切点向小链轮切点）
    if (s < g.topLen) {
      const frac = s / g.topLen;
      px = g.xTopStart + frac * (g.xTopEnd - g.xTopStart);
      py = g.yTopStart + frac * (g.yTopEnd - g.yTopStart);
      rotZ = g.rotTop;
    }
    // 段 2: 小链轮外包圆弧（从 aTop 逆时针扫过 PI - 2*alpha 至 aBot）
    else if (s < g.topLen + g.arcFront) {
      const frac = (s - g.topLen) / g.arcFront;
      const currentAng = g.aTop + frac * (Math.PI - 2 * g.alpha);
      px = g.x1 + Math.cos(currentAng) * g.r1;
      py = g.y1 + Math.sin(currentAng) * g.r1;
      rotZ = currentAng + Math.PI / 2;
    }
    // 段 3: 下公切线（小链轮切点向后轮切点）
    else if (s < g.topLen + g.arcFront + g.botLen) {
      const frac = (s - g.topLen - g.arcFront) / g.botLen;
      px = g.xBotStart + frac * (g.xBotEnd - g.xBotStart);
      py = g.yBotStart + frac * (g.yBotEnd - g.yBotStart);
      rotZ = g.rotBot;
    }
    // 段 4: 后轮大链盘外包圆弧（从 aBot 逆时针扫过 PI + 2*alpha 至 aTop）
    else {
      const frac = (s - g.topLen - g.arcFront - g.botLen) / g.arcRear;
      const currentAng = g.aBot + frac * (Math.PI + 2 * g.alpha);
      px = g.x2 + Math.cos(currentAng) * g.r2;
      py = g.y2 + Math.sin(currentAng) * g.r2;
      rotZ = currentAng + Math.PI / 2;
    }

    chainLinks[k].position.set(px, py, 0);
    chainLinks[k].rotation.z = rotZ;
  }
}

// 仅在滑块输入或初始化时更新 DOM，避免 60fps~144fps 频繁重排
function updateUI(kine) {
  if (!kine) return;
  document.getElementById('lbl-speed').innerText = Math.round(currentSpeed);
  document.getElementById('lbl-ice').innerText = Math.round(currentIceRpm);

  document.getElementById('val-mg1-rpm').innerText = Math.round(kine.mg1Rpm);
  document.getElementById('val-mg2-rpm').innerText = Math.round(kine.mg2Rpm);
}

let raycastPending = false;
let cachedMouseX = 0, cachedMouseY = 0;
let raycaster = null;
let mouse = null;
let tooltip = null;
let tooltipTitle = null;
let tooltipDesc = null;
let cachedHitCandidates = null;

function performRaycast() {
  if (!raycaster || !mouse || !tooltip || !cachedHitCandidates) return;
  raycaster.setFromCamera(mouse, camera);
  const intersects = raycaster.intersectObjects(cachedHitCandidates, true);

  if (intersects.length > 0) {
    const hitObj = intersects[0].object;
    const found = interactiveMeshes.find(item => item.mesh === hitObj || isDescendant(item.mesh, hitObj));
    if (found) {
      tooltipTitle.innerText = found.title;
      tooltipDesc.innerText = found.desc;

      // 视口边界夹紧，避免边缘溢出
      const tipW = 240;
      const tipH = 75;
      const clampedX = Math.max(tipW / 2 + 12, Math.min(window.innerWidth - tipW / 2 - 12, cachedMouseX));
      const clampedY = Math.max(tipH + 12, Math.min(window.innerHeight - 12, cachedMouseY));

      tooltip.style.left = `${clampedX}px`;
      tooltip.style.top = `${clampedY}px`;
      tooltip.style.opacity = '1';
      return;
    }
  }
  tooltip.style.opacity = '0';
}

function setupInteractions() {
  raycaster = new THREE.Raycaster();
  mouse = new THREE.Vector2();
  tooltip = document.getElementById('tooltip');
  tooltipTitle = document.getElementById('tooltip-title');
  tooltipDesc = document.getElementById('tooltip-desc');
  cachedHitCandidates = interactiveMeshes.map(item => item.mesh);

  function hideTooltip() {
    if (tooltip) tooltip.style.opacity = '0';
    raycastPending = false;
  }

  // 节流鼠标与指针移动，将射线计算转移到渲染帧中
  function onPointerMove(e) {
    // 触屏滑动调整视角或多点触控时忽略射线拾取，避免 Tooltip 遮挡闪烁
    if (e.pointerType === 'touch') {
      hideTooltip();
      return;
    }
    cachedMouseX = e.clientX;
    cachedMouseY = e.clientY;
    mouse.x = (e.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(e.clientY / window.innerHeight) * 2 + 1;
    raycastPending = true;
  }

  window.addEventListener('pointermove', onPointerMove, { passive: true });
  window.addEventListener('pointerleave', hideTooltip);
  window.addEventListener('blur', hideTooltip);
  if (controls) {
    controls.addEventListener('start', hideTooltip);
  }

  const speedSlider = document.getElementById('slider-speed');
  const iceSlider = document.getElementById('slider-ice');

  speedSlider.addEventListener('input', (e) => {
    currentSpeed = parseFloat(e.target.value);
    currentKine = calculateKinematics(currentSpeed, currentIceRpm);
    updateUI(currentKine);
  });

  iceSlider.addEventListener('input', (e) => {
    currentIceRpm = parseFloat(e.target.value);
    currentKine = calculateKinematics(currentSpeed, currentIceRpm);
    updateUI(currentKine);
  });

  const btnViewLeft = document.getElementById('btn-view-left');
  const btnViewTop = document.getElementById('btn-view-top');
  const btnViewIso = document.getElementById('btn-view-iso');
  const viewBtns = [btnViewLeft, btnViewTop, btnViewIso];

  function setActiveViewBtn(activeBtn) {
    viewBtns.forEach(btn => {
      if (btn === activeBtn) {
        btn.className = "px-3 py-1.5 text-xs rounded-xl bg-sky-600/30 text-sky-300 transition font-medium border border-sky-500/40";
      } else {
        btn.className = "px-3 py-1.5 text-xs rounded-xl bg-slate-800/80 hover:bg-sky-600/40 text-slate-200 transition font-medium border border-slate-700/50";
      }
    });
  }

  btnViewLeft.onclick = () => {
    setActiveViewBtn(btnViewLeft);
    moveCameraTo(new THREE.Vector3(12, 4, 38), new THREE.Vector3(12, 4, 0));
  };
  btnViewTop.onclick = () => {
    setActiveViewBtn(btnViewTop);
    moveCameraTo(new THREE.Vector3(12, 42, 0.1), new THREE.Vector3(12, 3, 0));
  };
  btnViewIso.onclick = () => {
    setActiveViewBtn(btnViewIso);
    moveCameraTo(new THREE.Vector3(24, 18, 30), new THREE.Vector3(10, 3, 0));
  };

  const btnExplode = document.getElementById('btn-explode');
  btnExplode.onclick = () => {
    isExploded = !isExploded;
    btnExplode.innerText = isExploded ? "复原总成" : "爆炸图";
    btnExplode.className = isExploded
      ? "px-3 py-1.5 text-xs rounded-xl bg-amber-500 text-slate-950 font-bold transition shadow-lg"
      : "px-3 py-1.5 text-xs rounded-xl bg-amber-500/20 hover:bg-amber-500/30 text-amber-300 transition font-medium border border-amber-500/30";
  };
}

let currentCameraAnimId = null;
function moveCameraTo(pos, target) {
  if (currentCameraAnimId) {
    cancelAnimationFrame(currentCameraAnimId);
    currentCameraAnimId = null;
  }
  const startPos = camera.position.clone();
  const startTarget = controls.target.clone();
  const duration = 600;
  const startTime = performance.now();

  function stepCamera(now) {
    const elapsed = now - startTime;
    const progress = Math.min(elapsed / duration, 1.0);
    const ease = 0.5 - Math.cos(progress * Math.PI) / 2;

    camera.position.lerpVectors(startPos, pos, ease);
    controls.target.lerpVectors(startTarget, target, ease);
    controls.update();

    if (progress < 1.0) {
      currentCameraAnimId = requestAnimationFrame(stepCamera);
    } else {
      currentCameraAnimId = null;
    }
  }
  currentCameraAnimId = requestAnimationFrame(stepCamera);
}

// 根据屏幕宽度与宽高比自动等比自适应缩放整个 3D 动力学模型视图
let prevBaseScale = null;

function calculateBaseScale(w, h) {
  const aspect = w / h;
  const BASE_WIDTH = 1440;
  let scale = 1.0;
  if (w < BASE_WIDTH) {
    scale = Math.pow(w / BASE_WIDTH, 0.72);
  } else {
    scale = Math.min(1.15, 1.0 + (w - BASE_WIDTH) * 0.0001);
  }
  if (aspect < 1.45) {
    const aspectFactor = Math.max(0.65, aspect / 1.45);
    scale *= aspectFactor;
  }
  return Math.max(0.38, Math.min(1.18, scale));
}

function updateResponsiveScale() {
  if (!camera) return;
  const container = document.getElementById('canvas-container');
  const w = container ? (container.clientWidth || window.innerWidth) : window.innerWidth;
  const h = container ? (container.clientHeight || window.innerHeight) : window.innerHeight;
  const aspect = w / h;

  const newBaseScale = calculateBaseScale(w, h);

  if (prevBaseScale === null) {
    // 首次载入初始化基准缩放
    camera.zoom = newBaseScale;
    prevBaseScale = newBaseScale;
  } else {
    // 视口尺寸变化时按基准比例缩放，完整保留用户交互中通过鼠标滚轮自定义的缩放倍率
    const scaleRatio = newBaseScale / prevBaseScale;
    camera.zoom *= scaleRatio;
    prevBaseScale = newBaseScale;
  }

  camera.aspect = aspect;
  camera.updateProjectionMatrix();

  if (controls) {
    controls.zoom0 = camera.zoom;
  }
}

function onWindowResize() {
  const container = document.getElementById('canvas-container');
  const w = container ? (container.clientWidth || window.innerWidth) : window.innerWidth;
  const h = container ? (container.clientHeight || window.innerHeight) : window.innerHeight;

  updateResponsiveScale();

  if (renderer) {
    renderer.setSize(w, h);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  }
}

let clock = new THREE.Clock();

function animate() {
  requestAnimationFrame(animate);

  const dt = Math.min(clock.getDelta(), 0.1);
  if (!currentKine) {
    currentKine = calculateKinematics(currentSpeed, currentIceRpm);
  }
  const kine = currentKine;

  // 帧级别节流射线检测
  if (raycastPending) {
    performRaycast();
    raycastPending = false;
  }

  const targetExplode = isExploded ? 1.0 : 0.0;
  explodeFactor += (targetExplode - explodeFactor) * 0.08;
  partsToExplode.forEach(item => {
    if (item.axis === 'z') {
      item.group.position.z = item.origPos.z + item.delta * explodeFactor;
    } else if (item.axis === 'y') {
      item.group.position.y = item.origPos.y + item.delta * explodeFactor;
    }
  });

  const RAD_PER_SEC = (Math.PI * 2) / 60;
  // 显示转速缩放为实际转速的 1/10
  const speedScale = 0.028;

  if (!isExploded || explodeFactor < 0.2) {
    // 1. 后轮 (逆时针正向旋转)
    const wheelDelta = kine.wheelRpm * RAD_PER_SEC * dt * speedScale;
    angles.wheel += wheelDelta;
    wheelGroup.rotation.z = angles.wheel;

    // 2. 副轴总成 (副轴大齿轮、实心轴、前小链轮完全刚性一体化逆时针旋转)
    const sumShaftDelta = kine.sumShaftRpm * RAD_PER_SEC * dt * speedScale;
    angles.sumShaft += sumShaftDelta;
    outShaftGroup.rotation.z = initialPhase.sumShaft + angles.sumShaft;

    // 链条线位移与小链轮分度圆转角严格绝对刚性绑定（无浮点累加漂移）
    updateChainLinks(angles.sumShaft * R_FRONT_SPROCKET);

    // 3. MG2 小齿轮 (顺时针外啮合)
    const mg2Delta = kine.mg2Rpm * RAD_PER_SEC * dt * speedScale;
    angles.mg2 -= mg2Delta;
    mg2Pinion.rotation.z = initialPhase.mg2 + angles.mg2;

    // 4. 齿圈 (顺时针外啮合)
    const ringDelta = kine.ringRpm * RAD_PER_SEC * dt * speedScale;
    angles.ring -= ringDelta;
    ringGearMesh.rotation.z = initialPhase.ring + angles.ring;

    // 5. 行星架与曲轴主齿轮
    const carrierDelta = kine.carrierRpm * RAD_PER_SEC * dt * speedScale;
    angles.carrier -= carrierDelta;
    carrierAssembly.rotation.z = initialPhase.carrier + angles.carrier;

    const iceDelta = (kine.carrierRpm * I_ICE_C) * RAD_PER_SEC * dt * speedScale;
    angles.ice += iceDelta;
    if (icePrimaryGear) {
      icePrimaryGear.rotation.z = initialPhase.ice + angles.ice;
    }

    // 6. 太阳轮与 MG1 电机
    const sunDelta = kine.mg1Rpm * RAD_PER_SEC * dt * speedScale;
    angles.sun -= sunDelta;
    sunGear.rotation.z = initialPhase.sun + angles.sun;
    mg1Group.rotation.z = initialPhase.sun + angles.sun;

    // 行星轮在动参考系下的真实相对自转动力学：
    // 相对角速度增量 = - (ω_sun - ω_carrier) * (Z_sun / Z_planet)
    const planetRelDelta = - ((-sunDelta) - (-carrierDelta)) * (Z_SUN / Z_PLANET);
    angles.planetRel += planetRelDelta;

    planetMeshes.forEach(p => {
      // 行星架转角为 -angles.carrier，叠加自转增量
      p.mesh.rotation.z = initialPhase.planets[p.index] + angles.planetRel;
    });
  }

  if (controls) controls.update();
  if (renderer && scene && camera) renderer.render(scene, camera);
}

function ensureDependencies(callback) {
  if (window.THREE && THREE.OrbitControls) {
    callback();
    return;
  }
  const cdnList = [
    "lib/three.min.js",
    "https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js",
    "https://unpkg.com/three@0.128.0/build/three.min.js",
    "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"
  ];
  const ctrlCdnList = [
    "lib/OrbitControls.js",
    "https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js",
    "https://unpkg.com/three@0.128.0/examples/js/controls/OrbitControls.js",
    "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/examples/js/controls/OrbitControls.js"
  ];
  let idx = 0;
  function tryNext() {
    if (idx >= cdnList.length) {
      console.error("Three.js 加载失败，请检查网络连接");
      const alertBox = document.createElement('div');
      alertBox.className = "absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 px-6 py-4 rounded-2xl glass-panel text-rose-400 z-50 text-center";
      alertBox.innerText = "Three.js 资源加载失败，请检查网络连接或刷新页面重试。";
      document.body.appendChild(alertBox);
      return;
    }
    const s = document.createElement('script');
    const curIdx = idx++;
    s.src = cdnList[curIdx];
    s.onload = () => {
      const ctrlScript = document.createElement('script');
      ctrlScript.src = ctrlCdnList[curIdx] || ctrlCdnList[0];
      ctrlScript.onload = () => callback();
      ctrlScript.onerror = () => tryNext();
      document.head.appendChild(ctrlScript);
    };
    s.onerror = tryNext;
    document.head.appendChild(s);
  }
  tryNext();
}

window.onload = function () {
  ensureDependencies(() => {
    initSceneAndModels();
  });
};
