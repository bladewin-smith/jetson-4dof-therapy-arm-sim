const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;
const EPS = 1e-9;

export const DEFAULT_CONFIG = Object.freeze({
  links: Object.freeze({
    baseHeight: 0.18,
    upperArm: 0.32,
    forearm: 0.28,
    tool: 0.16,
  }),
  jointLimitsDeg: Object.freeze([
    Object.freeze([-170, 170]),
    Object.freeze([-75, 105]),
    Object.freeze([-135, 135]),
    Object.freeze([-120, 120]),
  ]),
  directions: Object.freeze([1, 1, 1, 1]),
  safety: Object.freeze({
    minSingularValue: 0.025,
    maxConditionNumber: 60,
    minJointMarginDeg: 4,
    baseKeepoutRadius: 0.075,
    floorHeight: 0.015,
  }),
  therapyZone: Object.freeze({
    x: Object.freeze([0.20, 0.56]),
    y: Object.freeze([-0.28, 0.28]),
    z: Object.freeze([0.18, 0.58]),
  }),
});

export function createConfig(overrides = {}) {
  return {
    links: { ...DEFAULT_CONFIG.links, ...(overrides.links || {}) },
    jointLimitsDeg: (overrides.jointLimitsDeg || DEFAULT_CONFIG.jointLimitsDeg).map((v) => [...v]),
    directions: [...(overrides.directions || DEFAULT_CONFIG.directions)],
    safety: { ...DEFAULT_CONFIG.safety, ...(overrides.safety || {}) },
    therapyZone: {
      x: [...(overrides.therapyZone?.x || DEFAULT_CONFIG.therapyZone.x)],
      y: [...(overrides.therapyZone?.y || DEFAULT_CONFIG.therapyZone.y)],
      z: [...(overrides.therapyZone?.z || DEFAULT_CONFIG.therapyZone.z)],
    },
  };
}

export const degToRad = (value) => value * DEG;
export const radToDeg = (value) => value * RAD;
export const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

export function normalizeRad(value) {
  let result = value;
  while (result > Math.PI) result -= Math.PI * 2;
  while (result <= -Math.PI) result += Math.PI * 2;
  return result;
}

export function commandsToModelRad(commandsDeg, config) {
  return commandsDeg.map((value, index) => degToRad(value * config.directions[index]));
}

export function modelRadToCommands(modelRad, config) {
  return modelRad.map((value, index) => radToDeg(value) * config.directions[index]);
}

export function forwardKinematics(commandsDeg, config) {
  return forwardKinematicsModel(commandsToModelRad(commandsDeg, config), config);
}

export function forwardKinematicsModel(modelRad, config) {
  const [q1, q2, q3, q4] = modelRad;
  const { baseHeight: h, upperArm: l1, forearm: l2, tool: l3 } = config.links;
  const a2 = q2;
  const a3 = q2 + q3;
  const a4 = q2 + q3 + q4;
  const er = { x: Math.cos(q1), y: Math.sin(q1) };

  const shoulder = { x: 0, y: 0, z: h };
  const elbow = {
    x: l1 * Math.cos(a2) * er.x,
    y: l1 * Math.cos(a2) * er.y,
    z: h + l1 * Math.sin(a2),
  };
  const wrist = {
    x: (l1 * Math.cos(a2) + l2 * Math.cos(a3)) * er.x,
    y: (l1 * Math.cos(a2) + l2 * Math.cos(a3)) * er.y,
    z: h + l1 * Math.sin(a2) + l2 * Math.sin(a3),
  };
  const radius = l1 * Math.cos(a2) + l2 * Math.cos(a3) + l3 * Math.cos(a4);
  const end = {
    x: radius * er.x,
    y: radius * er.y,
    z: h + l1 * Math.sin(a2) + l2 * Math.sin(a3) + l3 * Math.sin(a4),
  };

  return {
    modelRad: [...modelRad],
    commandsDeg: modelRadToCommands(modelRad, config),
    points: [{ x: 0, y: 0, z: 0 }, shoulder, elbow, wrist, end],
    pose: {
      ...end,
      radialDistance: Math.hypot(end.x, end.y),
      yawDeg: radToDeg(q1),
      pitchDeg: radToDeg(normalizeRad(a4)),
    },
  };
}

export function positionJacobian(modelRad, config) {
  const [q1, q2, q3, q4] = modelRad;
  const { upperArm: l1, forearm: l2, tool: l3 } = config.links;
  const a3 = q2 + q3;
  const a4 = a3 + q4;
  const r = l1 * Math.cos(q2) + l2 * Math.cos(a3) + l3 * Math.cos(a4);
  const dr2 = -l1 * Math.sin(q2) - l2 * Math.sin(a3) - l3 * Math.sin(a4);
  const dr3 = -l2 * Math.sin(a3) - l3 * Math.sin(a4);
  const dr4 = -l3 * Math.sin(a4);
  const dz2 = l1 * Math.cos(q2) + l2 * Math.cos(a3) + l3 * Math.cos(a4);
  const dz3 = l2 * Math.cos(a3) + l3 * Math.cos(a4);
  const dz4 = l3 * Math.cos(a4);
  const c = Math.cos(q1);
  const s = Math.sin(q1);

  return [
    [-r * s, dr2 * c, dr3 * c, dr4 * c],
    [r * c, dr2 * s, dr3 * s, dr4 * s],
    [0, dz2, dz3, dz4],
  ];
}

function multiplyByTranspose(matrix) {
  return matrix.map((rowA) => matrix.map((rowB) => rowA.reduce((sum, value, i) => sum + value * rowB[i], 0)));
}

export function eigenvaluesSymmetric3(input) {
  const a = input.map((row) => [...row]);
  for (let iteration = 0; iteration < 18; iteration += 1) {
    let p = 0;
    let q = 1;
    let max = Math.abs(a[0][1]);
    for (const [i, j] of [[0, 2], [1, 2]]) {
      if (Math.abs(a[i][j]) > max) {
        max = Math.abs(a[i][j]);
        p = i;
        q = j;
      }
    }
    if (max < 1e-14) break;
    const angle = 0.5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]);
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    for (let k = 0; k < 3; k += 1) {
      if (k === p || k === q) continue;
      const akp = a[k][p];
      const akq = a[k][q];
      a[k][p] = a[p][k] = c * akp - s * akq;
      a[k][q] = a[q][k] = s * akp + c * akq;
    }
    const app = a[p][p];
    const aqq = a[q][q];
    const apq = a[p][q];
    a[p][p] = c * c * app - 2 * s * c * apq + s * s * aqq;
    a[q][q] = s * s * app + 2 * s * c * apq + c * c * aqq;
    a[p][q] = a[q][p] = 0;
  }
  return [a[0][0], a[1][1], a[2][2]].sort((x, y) => y - x);
}

export function singularityMetrics(modelRad, config) {
  const jacobian = positionJacobian(modelRad, config);
  const eigenvalues = eigenvaluesSymmetric3(multiplyByTranspose(jacobian));
  const singularValues = eigenvalues.map((value) => Math.sqrt(Math.max(0, value)));
  const [maximum, , minimum] = singularValues;
  return {
    jacobian,
    singularValues,
    minSingularValue: minimum,
    maxSingularValue: maximum,
    conditionNumber: minimum < EPS ? Infinity : maximum / minimum,
    manipulability: singularValues.reduce((product, value) => product * value, 1),
  };
}

function distancePointToVerticalAxis(point) {
  return Math.hypot(point.x, point.y);
}

export function evaluateSafety(modelRad, config) {
  const fk = forwardKinematicsModel(modelRad, config);
  const metrics = singularityMetrics(modelRad, config);
  const modelDeg = modelRad.map(radToDeg);
  const marginsDeg = modelDeg.map((angle, index) => {
    const [min, max] = config.jointLimitsDeg[index];
    return Math.min(angle - min, max - angle);
  });
  const withinLimits = marginsDeg.every((value) => value >= -1e-7);
  const jointMarginDeg = Math.min(...marginsDeg);
  const aboveFloor = fk.points.slice(2).every((point) => point.z >= config.safety.floorHeight);
  const baseClear = fk.points.slice(2).every((point) => (
    point.z >= config.links.baseHeight || distancePointToVerticalAxis(point) >= config.safety.baseKeepoutRadius
  ));
  const singularitySafe = (
    metrics.minSingularValue >= config.safety.minSingularValue
    && metrics.conditionNumber <= config.safety.maxConditionNumber
  );
  const marginSafe = jointMarginDeg >= config.safety.minJointMarginDeg;
  const reasons = [];
  if (!withinLimits) reasons.push("关节限位超出");
  else if (!marginSafe) reasons.push("关节限位余量不足");
  if (!singularitySafe) reasons.push("接近位置奇异点");
  if (!aboveFloor) reasons.push("连杆低于安全地面");
  if (!baseClear) reasons.push("连杆进入底座禁入区");

  return {
    safe: withinLimits && marginSafe && singularitySafe && aboveFloor && baseClear,
    withinLimits,
    marginSafe,
    singularitySafe,
    aboveFloor,
    baseClear,
    jointMarginDeg,
    marginsDeg,
    reasons,
    metrics,
    fk,
  };
}

function angleCandidates(preferredDeg, optimize) {
  if (!optimize) return [preferredDeg];
  const result = [preferredDeg];
  for (let offset = 5; offset <= 180; offset += 5) {
    if (preferredDeg - offset >= -180) result.push(preferredDeg - offset);
    if (preferredDeg + offset <= 180) result.push(preferredDeg + offset);
  }
  return result;
}

function limitEquivalentAngle(angleRad, limitDeg) {
  const candidates = [angleRad, angleRad + 2 * Math.PI, angleRad - 2 * Math.PI];
  const feasible = candidates.filter((candidate) => {
    const deg = radToDeg(candidate);
    return deg >= limitDeg[0] - 1e-8 && deg <= limitDeg[1] + 1e-8;
  });
  if (feasible.length) return feasible.sort((a, b) => Math.abs(a) - Math.abs(b))[0];
  return normalizeRad(angleRad);
}

export function evaluateJointPath(startCommandsDeg, endCommandsDeg, config, steps = 36) {
  let minimumSigma = Infinity;
  let maximumCondition = 0;
  let minimumMargin = Infinity;
  let firstUnsafeIndex = -1;
  const samples = [];

  for (let index = 0; index <= steps; index += 1) {
    const t = index / steps;
    const command = startCommandsDeg.map((start, joint) => start + (endCommandsDeg[joint] - start) * t);
    const result = evaluateSafety(commandsToModelRad(command, config), config);
    minimumSigma = Math.min(minimumSigma, result.metrics.minSingularValue);
    maximumCondition = Math.max(maximumCondition, result.metrics.conditionNumber);
    minimumMargin = Math.min(minimumMargin, result.jointMarginDeg);
    if (!result.safe && firstUnsafeIndex < 0) firstUnsafeIndex = index;
    if (index % Math.max(1, Math.floor(steps / 12)) === 0 || index === steps) {
      samples.push({ t, point: result.fk.pose, safe: result.safe });
    }
  }
  return {
    safe: firstUnsafeIndex < 0,
    firstUnsafeIndex,
    minimumSigma,
    maximumCondition,
    minimumMargin,
    samples,
  };
}

export function solveInverse(target, config, options = {}) {
  const desiredPitchDeg = options.desiredPitchDeg ?? 0;
  const optimizePitch = options.optimizePitch ?? true;
  const currentCommandsDeg = options.currentCommandsDeg || [0, 0, 0, 0];
  const checkPath = options.checkPath ?? true;
  const { baseHeight: h, upperArm: l1, forearm: l2, tool: l3 } = config.links;
  const targetRadius = Math.hypot(target.x, target.y);
  const baseYaw = Math.atan2(target.y, target.x);
  const solutions = [];

  for (const pitchDeg of angleCandidates(desiredPitchDeg, optimizePitch)) {
    const pitch = degToRad(pitchDeg);
    for (const radialSign of [1, -1]) {
      const q1Raw = radialSign === 1 ? baseYaw : normalizeRad(baseYaw + Math.PI);
      const q1 = limitEquivalentAngle(q1Raw, config.jointLimitsDeg[0]);
      const signedRadius = targetRadius * radialSign;
      const wristR = signedRadius - l3 * Math.cos(pitch);
      const wristZ = target.z - h - l3 * Math.sin(pitch);
      const cosineElbow = (wristR ** 2 + wristZ ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2);
      if (cosineElbow < -1 - 1e-8 || cosineElbow > 1 + 1e-8) continue;
      const elbowMagnitude = Math.acos(clamp(cosineElbow, -1, 1));

      for (const elbowSign of [1, -1]) {
        const q3 = elbowMagnitude * elbowSign;
        const q2 = Math.atan2(wristZ, wristR) - Math.atan2(l2 * Math.sin(q3), l1 + l2 * Math.cos(q3));
        const q4 = pitch - q2 - q3;
        const raw = [q1, q2, q3, q4];
        const modelRad = raw.map((angle, index) => limitEquivalentAngle(angle, config.jointLimitsDeg[index]));
        const commandsDeg = modelRadToCommands(modelRad, config);
        const safety = evaluateSafety(modelRad, config);
        if (!safety.withinLimits) continue;
        const residual = Math.hypot(
          safety.fk.pose.x - target.x,
          safety.fk.pose.y - target.y,
          safety.fk.pose.z - target.z,
        );
        if (residual > 1e-5) continue;
        const path = checkPath ? evaluateJointPath(currentCommandsDeg, commandsDeg, config, 24) : null;
        const movement = commandsDeg.reduce((sum, angle, index) => sum + Math.abs(angle - currentCommandsDeg[index]), 0);
        const pitchDeviation = Math.abs(normalizeRad(pitch - degToRad(desiredPitchDeg))) * RAD;
        const score = (
          (path?.safe ? 5000 : 0)
          + (safety.safe ? 2000 : 0)
          + Math.min(safety.metrics.minSingularValue, 0.2) * 2000
          + Math.min(safety.jointMarginDeg, 60) * 5
          - movement * 0.35
          - pitchDeviation * 0.25
        );
        solutions.push({
          modelRad,
          commandsDeg,
          safety,
          path,
          score,
          pitchDeg: radToDeg(normalizeRad(pitch)),
          elbow: elbowSign > 0 ? "down" : "up",
          baseBranch: radialSign,
          residual,
        });
      }
    }
  }

  solutions.sort((a, b) => b.score - a.score);
  const safeSolutions = solutions.filter((solution) => solution.safety.safe && (!checkPath || solution.path?.safe));
  return {
    reachable: solutions.length > 0,
    safeReachable: safeSolutions.length > 0,
    chosen: safeSolutions[0] || solutions[0] || null,
    solutions,
    safeSolutionCount: safeSolutions.length,
  };
}

function halton(index, base) {
  let result = 0;
  let fraction = 1 / base;
  let i = index;
  while (i > 0) {
    result += fraction * (i % base);
    i = Math.floor(i / base);
    fraction /= base;
  }
  return result;
}

export function sampleWorkspace(config, count = 1600) {
  const points = [];
  const bases = [2, 3, 5, 7];
  for (let index = 1; index <= count; index += 1) {
    const modelDeg = config.jointLimitsDeg.map(([min, max], joint) => min + (max - min) * halton(index, bases[joint]));
    const modelRad = modelDeg.map(degToRad);
    const result = evaluateSafety(modelRad, config);
    points.push({
      x: result.fk.pose.x,
      y: result.fk.pose.y,
      z: result.fk.pose.z,
      safe: result.safe,
      sigma: result.metrics.minSingularValue,
    });
  }
  return points;
}

export function inTherapyZone(point, zone) {
  return (
    point.x >= zone.x[0] && point.x <= zone.x[1]
    && point.y >= zone.y[0] && point.y <= zone.y[1]
    && point.z >= zone.z[0] && point.z <= zone.z[1]
  );
}

export function configurationSummary(config) {
  return {
    links: { ...config.links },
    jointLimitsDeg: config.jointLimitsDeg.map((limit) => [...limit]),
    directions: [...config.directions],
    safety: { ...config.safety },
    therapyZone: {
      x: [...config.therapyZone.x],
      y: [...config.therapyZone.y],
      z: [...config.therapyZone.z],
    },
  };
}

