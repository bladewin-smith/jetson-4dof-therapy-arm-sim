const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;
const EPS = 1e-9;

export const DEFAULT_CONFIG = Object.freeze({
  links: Object.freeze({
    baseHeight: 0.18,
    upperArm: 0.32,
    forearm: 0.28,
    tool: 0.16,
    terminal: 0.12,
  }),
  linearAxis: Object.freeze({
    minPosition: 0,
    maxPosition: 0.50,
    position: 0.22,
  }),
  toolAssembly: Object.freeze({
    verticalDirection: -1,
  }),
  jointLimitsDeg: Object.freeze([
    Object.freeze([-170, 170]),
    Object.freeze([-75, 105]),
    Object.freeze([-135, 135]),
    Object.freeze([-120, 120]),
  ]),
  jointAxes: Object.freeze(["yaw", "pitch", "yaw", "pitch"]),
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
  const jointAxes = [...(overrides.jointAxes || DEFAULT_CONFIG.jointAxes)];
  jointAxes[2] = "yaw";
  jointAxes[3] = "pitch";
  return {
    links: { ...DEFAULT_CONFIG.links, ...(overrides.links || {}) },
    linearAxis: { ...DEFAULT_CONFIG.linearAxis, ...(overrides.linearAxis || {}) },
    toolAssembly: { ...DEFAULT_CONFIG.toolAssembly, ...(overrides.toolAssembly || {}) },
    jointLimitsDeg: (overrides.jointLimitsDeg || DEFAULT_CONFIG.jointLimitsDeg).map((v) => [...v]),
    jointAxes,
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

export function shoulderHeight(config) {
  return config.links.baseHeight + config.linearAxis.position;
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
  const { upperArm: l1, forearm: l2, tool: verticalLength, terminal: terminalLength } = config.links;
  const h = shoulderHeight(config);
  const shoulder = { x: 0, y: 0, z: h };
  const points = [{ x: 0, y: 0, z: 0 }, shoulder];
  const jointOrigins = [];
  const jointAxesWorld = [];
  const linkLengths = [l1, l2];
  let position = [0, 0, h];
  let orientation = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

  for (let index = 0; index < 2; index += 1) {
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

  const wrist = [...position];
  jointOrigins.push([...wrist]);
  jointAxesWorld.push([0, 0, 1]);
  orientation = rotationMatrix([0, 0, 1], modelRad[2]);
  const verticalOffset = [0, 0, config.toolAssembly.verticalDirection * verticalLength];
  position = position.map((value, index) => value + verticalOffset[index]);
  points.push({ x: position[0], y: position[1], z: position[2] });
  jointOrigins.push([...position]);
  const terminalPitchAxis = localAxis("pitch");
  jointAxesWorld.push(multiplyMatrixVector(orientation, terminalPitchAxis));
  orientation = multiplyMatrix3(orientation, rotationMatrix(terminalPitchAxis, modelRad[3]));
  const terminalOffset = multiplyMatrixVector(orientation, [terminalLength, 0, 0]);
  position = position.map((value, index) => value + terminalOffset[index]);
  points.push({ x: position[0], y: position[1], z: position[2] });

  const end = { x: position[0], y: position[1], z: position[2] };
  const toolDirection = multiplyMatrixVector(orientation, [1, 0, 0]);

  return {
    modelRad: [...modelRad],
    commandsDeg: modelRadToCommands(modelRad, config),
    points,
    jointOrigins,
    jointAxesWorld,
    wrist,
    terminalOffset,
    verticalOffset,
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
  const columns = fk.jointOrigins.map((origin, index) => {
    const wristContribution = index < 2
      ? cross(fk.jointAxesWorld[index], fk.wrist.map((value, axisIndex) => value - origin[axisIndex]))
      : [0, 0, 0];
    const terminalContribution = index >= 2
      ? cross(fk.jointAxesWorld[index], fk.terminalOffset)
      : [0, 0, 0];
    return wristContribution.map((value, axisIndex) => value + terminalContribution[axisIndex]);
  });
  const allColumns = [[0, 0, 1], ...columns];
  return [0, 1, 2].map((row) => allColumns.map((column) => column[row]));
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
  const characteristicLength = config.links.upperArm + config.links.forearm + config.links.terminal;
  const normalizedJacobian = jacobian.map((row) => row.map((value, column) => (
    column === 0 ? value * characteristicLength : value
  )));
  const eigenvalues = eigenvaluesSymmetric3(multiplyByTranspose(normalizedJacobian));
  const singularValues = eigenvalues.map((value) => Math.sqrt(Math.max(0, value)));
  const [maximum, , minimum] = singularValues;
  return {
    jacobian,
    normalizedJacobian,
    characteristicLength,
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
  const slideMargin = Math.min(
    config.linearAxis.position - config.linearAxis.minPosition,
    config.linearAxis.maxPosition - config.linearAxis.position,
  );
  const withinSlideLimits = slideMargin >= -1e-9;
  const aboveFloor = fk.points.slice(2).every((point) => point.z >= config.safety.floorHeight);
  const baseClear = fk.points.slice(2).every((point) => (
    point.z >= shoulderHeight(config) || distancePointToVerticalAxis(point) >= config.safety.baseKeepoutRadius
  ));
  const singularitySafe = (
    metrics.minSingularValue >= config.safety.minSingularValue
    && metrics.conditionNumber <= config.safety.maxConditionNumber
  );
  const marginSafe = jointMarginDeg >= config.safety.minJointMarginDeg;
  const reasons = [];
  if (!withinLimits) reasons.push("关节限位超出");
  else if (!marginSafe) reasons.push("关节限位余量不足");
  if (!withinSlideLimits) reasons.push("升降轴超出行程");
  if (!singularitySafe) reasons.push("接近位置奇异点");
  if (!aboveFloor) reasons.push("连杆低于安全地面");
  if (!baseClear) reasons.push("连杆进入底座禁入区");

  return {
    safe: withinLimits && withinSlideLimits && marginSafe && singularitySafe && aboveFloor && baseClear,
    withinLimits,
    withinSlideLimits,
    marginSafe,
    singularitySafe,
    aboveFloor,
    baseClear,
    jointMarginDeg,
    slideMargin,
    marginsDeg,
    reasons,
    metrics,
    fk,
  };
}

export function evaluateJointPath(startCommandsDeg, endCommandsDeg, config, steps = 36, linearAxisRange = null) {
  let minimumSigma = Infinity;
  let maximumCondition = 0;
  let minimumMargin = Infinity;
  let firstUnsafeIndex = -1;
  const samples = [];

  for (let index = 0; index <= steps; index += 1) {
    const t = index / steps;
    const command = startCommandsDeg.map((start, joint) => start + (endCommandsDeg[joint] - start) * t);
    const sampleConfig = linearAxisRange
      ? createConfig({
        ...config,
        linearAxis: {
          ...config.linearAxis,
          position: linearAxisRange.start + (linearAxisRange.end - linearAxisRange.start) * t,
        },
      })
      : config;
    const result = evaluateSafety(commandsToModelRad(command, sampleConfig), sampleConfig);
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

    const jacobian = positionJacobian(modelRad, config).map((row) => row.slice(1));
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
  const multiStart = options.multiStart ?? options.optimizePitch ?? true;
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

function solveInverseAtFixedSlide(target, config, options = {}) {
  return solveNumericInverse(target, config, options);
}

export function solveInverse(target, config, options = {}) {
  if (!(options.optimizeSlide ?? false)) {
    const result = solveInverseAtFixedSlide(target, config, options);
    result.solutions.forEach((solution) => { solution.slidePosition = config.linearAxis.position; });
    if (result.chosen) result.chosen.slidePosition = config.linearAxis.position;
    return result;
  }

  const startSlide = config.linearAxis.position;
  const { minPosition, maxPosition } = config.linearAxis;
  const candidates = [startSlide];
  for (let index = 0; index <= 10; index += 1) {
    const position = minPosition + (maxPosition - minPosition) * index / 10;
    if (!candidates.some((value) => Math.abs(value - position) < 1e-8)) candidates.push(position);
  }
  candidates.sort((a, b) => Math.abs(a - startSlide) - Math.abs(b - startSlide));

  const solutions = [];
  for (const position of candidates) {
    const candidateConfig = createConfig({
      ...config,
      linearAxis: { ...config.linearAxis, position },
    });
    const fixed = solveInverseAtFixedSlide(target, candidateConfig, { ...options, checkPath: false });
    for (const solution of fixed.solutions) {
      solution.slidePosition = position;
      solution.path = (options.checkPath ?? true)
        ? evaluateJointPath(
          options.currentCommandsDeg || [0, 0, 0, 0],
          solution.commandsDeg,
          candidateConfig,
          24,
          { start: startSlide, end: position },
        )
        : null;
      solution.score += (solution.path?.safe ? 5000 : 0) - Math.abs(position - startSlide) * 120;
      solutions.push(solution);
    }
  }

  solutions.sort((a, b) => b.score - a.score);
  const checkPath = options.checkPath ?? true;
  const safeSolutions = solutions.filter((solution) => solution.safety.safe && (!checkPath || solution.path?.safe));
  return {
    reachable: solutions.length > 0,
    safeReachable: safeSolutions.length > 0,
    chosen: safeSolutions[0] || solutions[0] || null,
    solutions,
    safeSolutionCount: safeSolutions.length,
    method: "numeric-dls+linear-search",
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
    linearAxis: { ...config.linearAxis },
    toolAssembly: { ...config.toolAssembly },
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

