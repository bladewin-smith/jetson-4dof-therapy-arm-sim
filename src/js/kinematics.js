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
  jointAxes: Object.freeze(["yaw", "pitch", "pitch", "pitch"]),
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
    jointAxes: [...(overrides.jointAxes || DEFAULT_CONFIG.jointAxes)],
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

function multiplyMatrix3(a, b) {
  return a.map((row) => [0, 1, 2].map((column) => row.reduce((sum, value, index) => sum + value * b[index][column], 0)));
}

function multiplyMatrixVector(matrix, vector) {
  return matrix.map((row) => row.reduce((sum, value, index) => sum + value * vector[index], 0));
}

function rotationMatrix(axis, angle) {
  const [x, y, z] = axis;
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const t = 1 - c;
  return [
    [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
    [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
    [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
  ];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function localAxis(type) {
  return type === "yaw" ? [0, 0, 1] : [0, -1, 0];
}

export function forwardKinematicsModel(modelRad, config) {
  const { baseHeight: h, upperArm: l1, forearm: l2, tool: l3 } = config.links;
  const shoulder = { x: 0, y: 0, z: h };
  const points = [{ x: 0, y: 0, z: 0 }, shoulder];
  const jointOrigins = [];
  const jointAxesWorld = [];
  const linkLengths = [0, l1, l2, l3];
  let position = [0, 0, h];
  let orientation = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

  for (let index = 0; index < 4; index += 1) {
    const axis = localAxis(config.jointAxes[index]);
    jointOrigins.push([...position]);
    jointAxesWorld.push(multiplyMatrixVector(orientation, axis));
    orientation = multiplyMatrix3(orientation, rotationMatrix(axis, modelRad[index]));
    const length = linkLengths[index];
    if (length > EPS) {
      const offset = multiplyMatrixVector(orientation, [length, 0, 0]);
      position = position.map((value, axisIndex) => value + offset[axisIndex]);
      points.push({ x: position[0], y: position[1], z: position[2] });
    }
  }

  const end = { x: position[0], y: position[1], z: position[2] };
  const toolDirection = multiplyMatrixVector(orientation, [1, 0, 0]);

  return {
    modelRad: [...modelRad],
    commandsDeg: modelRadToCommands(modelRad, config),
    points,
    jointOrigins,
    jointAxesWorld,
    orientation,
    pose: {
      ...end,
      radialDistance: Math.hypot(end.x, end.y),
      yawDeg: radToDeg(Math.atan2(toolDirection[1], toolDirection[0])),
      pitchDeg: radToDeg(Math.atan2(toolDirection[2], Math.hypot(toolDirection[0], toolDirection[1]))),
    },
  };
}

export function positionJacobian(modelRad, config) {
  const fk = forwardKinematicsModel(modelRad, config);
  const end = [fk.pose.x, fk.pose.y, fk.pose.z];
  const columns = fk.jointOrigins.map((origin, index) => {
    const radius = end.map((value, axisIndex) => value - origin[axisIndex]);
    return cross(fk.jointAxesWorld[index], radius);
  });
  return [0, 1, 2].map((row) => columns.map((column) => column[row]));
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

function solveClassicInverse(target, config, options = {}) {
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

function invertMatrix3(matrix) {
  const [[a, b, c], [d, e, f], [g, h, i]] = matrix;
  const determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (Math.abs(determinant) < 1e-16) return null;
  return [
    [(e * i - f * h) / determinant, (c * h - b * i) / determinant, (b * f - c * e) / determinant],
    [(f * g - d * i) / determinant, (a * i - c * g) / determinant, (c * d - a * f) / determinant],
    [(d * h - e * g) / determinant, (b * g - a * h) / determinant, (a * e - b * d) / determinant],
  ];
}

function targetError(target, fk) {
  return [target.x - fk.pose.x, target.y - fk.pose.y, target.z - fk.pose.z];
}

function vectorNorm(vector) {
  return Math.hypot(...vector);
}

function clampModelToLimits(modelRad, config) {
  return modelRad.map((angle, index) => clamp(
    angle,
    degToRad(config.jointLimitsDeg[index][0]),
    degToRad(config.jointLimitsDeg[index][1]),
  ));
}

function solveDampedLeastSquares(target, seed, config) {
  let modelRad = clampModelToLimits(seed, config);
  let best = { modelRad: [...modelRad], residual: Infinity, iterations: 0 };
  for (let iteration = 0; iteration < 180; iteration += 1) {
    const fk = forwardKinematicsModel(modelRad, config);
    const error = targetError(target, fk);
    const residual = vectorNorm(error);
    if (residual < best.residual) best = { modelRad: [...modelRad], residual, iterations: iteration };
    if (residual < 5e-5) break;

    const jacobian = positionJacobian(modelRad, config);
    const normal = multiplyByTranspose(jacobian);
    const damping = 0.012 + Math.min(0.055, residual * 0.08);
    for (let axis = 0; axis < 3; axis += 1) normal[axis][axis] += damping ** 2;
    const inverse = invertMatrix3(normal);
    if (!inverse) break;
    const taskStep = multiplyMatrixVector(inverse, error);
    let delta = [0, 1, 2, 3].map((joint) => (
      jacobian[0][joint] * taskStep[0]
      + jacobian[1][joint] * taskStep[1]
      + jacobian[2][joint] * taskStep[2]
    ));
    const deltaNorm = vectorNorm(delta);
    if (deltaNorm > 0.28) delta = delta.map((value) => value * 0.28 / deltaNorm);

    let accepted = false;
    for (let lineSearch = 0; lineSearch < 6; lineSearch += 1) {
      const scale = 0.9 / (2 ** lineSearch);
      const candidate = clampModelToLimits(modelRad.map((value, index) => value + delta[index] * scale), config);
      const candidateResidual = vectorNorm(targetError(target, forwardKinematicsModel(candidate, config)));
      if (candidateResidual < residual - 1e-10) {
        modelRad = candidate;
        accepted = true;
        break;
      }
    }
    if (!accepted) break;
  }
  return best;
}

function numericSeeds(config, currentCommandsDeg, multiStart) {
  const seeds = [commandsToModelRad(currentCommandsDeg, config)];
  const center = config.jointLimitsDeg.map(([min, max]) => degToRad((min + max) / 2));
  seeds.push(center);
  if (multiStart) {
    const bases = [2, 3, 5, 7];
    for (let index = 1; index <= 46; index += 1) {
      seeds.push(config.jointLimitsDeg.map(([min, max], joint) => (
        degToRad(min + (max - min) * halton(index, bases[joint]))
      )));
    }
  }
  return seeds;
}

function solveNumericInverse(target, config, options = {}) {
  const currentCommandsDeg = options.currentCommandsDeg || [0, 0, 0, 0];
  const checkPath = options.checkPath ?? true;
  const multiStart = options.optimizePitch ?? true;
  const solutions = [];

  for (const seed of numericSeeds(config, currentCommandsDeg, multiStart)) {
    const result = solveDampedLeastSquares(target, seed, config);
    if (result.residual > 8e-4) continue;
    if (solutions.some((solution) => vectorNorm(solution.modelRad.map((value, index) => value - result.modelRad[index])) < 0.025)) continue;
    const modelRad = result.modelRad;
    const commandsDeg = modelRadToCommands(modelRad, config);
    const safety = evaluateSafety(modelRad, config);
    const path = checkPath ? evaluateJointPath(currentCommandsDeg, commandsDeg, config, 24) : null;
    const movement = commandsDeg.reduce((sum, angle, index) => sum + Math.abs(angle - currentCommandsDeg[index]), 0);
    const score = (
      (path?.safe ? 5000 : 0)
      + (safety.safe ? 2000 : 0)
      + Math.min(safety.metrics.minSingularValue, 0.2) * 2000
      + Math.min(safety.jointMarginDeg, 60) * 5
      - movement * 0.35
      - result.residual * 10000
    );
    solutions.push({
      modelRad,
      commandsDeg,
      safety,
      path,
      score,
      pitchDeg: safety.fk.pose.pitchDeg,
      elbow: "numeric",
      baseBranch: 0,
      residual: result.residual,
      iterations: result.iterations,
      method: "numeric-dls",
    });
  }

  solutions.sort((a, b) => b.score - a.score);
  const safeSolutions = solutions.filter((solution) => solution.safety.safe && (!checkPath || solution.path?.safe));
  return {
    reachable: solutions.length > 0,
    safeReachable: safeSolutions.length > 0,
    chosen: safeSolutions[0] || solutions[0] || null,
    solutions,
    safeSolutionCount: safeSolutions.length,
    method: "numeric-dls",
  };
}

function usesClassicAxisLayout(config) {
  return config.jointAxes.join(",") === "yaw,pitch,pitch,pitch";
}

export function solveInverse(target, config, options = {}) {
  if (usesClassicAxisLayout(config)) {
    const result = solveClassicInverse(target, config, options);
    result.method = "analytic";
    result.solutions.forEach((solution) => { solution.method = "analytic"; });
    return result;
  }
  return solveNumericInverse(target, config, options);
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
    jointAxes: [...config.jointAxes],
    directions: [...config.directions],
    safety: { ...config.safety },
    therapyZone: {
      x: [...config.therapyZone.x],
      y: [...config.therapyZone.y],
      z: [...config.therapyZone.z],
    },
  };
}

