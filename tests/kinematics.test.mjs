import test from "node:test";
import assert from "node:assert/strict";
import {
  commandsToModelRad,
  createConfig,
  evaluateJointPath,
  evaluateSafety,
  forwardKinematics,
  forwardKinematicsModel,
  positionJacobian,
  singularityMetrics,
  solveInverse,
} from "../src/js/kinematics.js";

const close = (actual, expected, tolerance = 1e-8) => assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);

test("zero pose follows the positive X axis", () => {
  const config = createConfig();
  const fk = forwardKinematics([0, 0, 0, 0], config);
  close(fk.pose.x, config.links.upperArm + config.links.forearm + config.links.terminal);
  close(fk.pose.y, 0);
  close(
    fk.pose.z,
    config.links.baseHeight + config.linearAxis.position
      + config.toolAssembly.verticalDirection * config.links.tool,
  );
});

test("motor direction changes command-to-model mapping", () => {
  const config = createConfig({ directions: [1, -1, 1, -1] });
  const model = commandsToModelRad([10, 20, -30, 40], config);
  close(model[0] * 180 / Math.PI, 10);
  close(model[1] * 180 / Math.PI, -20);
  close(model[2] * 180 / Math.PI, -30);
  close(model[3] * 180 / Math.PI, -40);
});

test("numeric IK reproduces FK targets with fixed J3 yaw and J4 pitch", () => {
  const config = createConfig();
  const seeds = [
    [20, 30, -70, 25],
    [-35, 50, -85, 20],
    [65, 10, 75, -55],
  ];
  for (const commands of seeds) {
    const fk = forwardKinematics(commands, config);
    const result = solveInverse(fk.pose, config, {
      multiStart: false,
      currentCommandsDeg: commands,
      checkPath: false,
    });
    assert.equal(result.reachable, true);
    assert.ok(result.chosen.residual < 1e-7);
    close(result.chosen.safety.fk.pose.x, fk.pose.x, 1e-7);
    close(result.chosen.safety.fk.pose.y, fk.pose.y, 1e-7);
    close(result.chosen.safety.fk.pose.z, fk.pose.z, 1e-7);
  }
});

test("fully extended arm is detected as position singular", () => {
  const config = createConfig();
  const metrics = singularityMetrics(commandsToModelRad([0, 0, 0, 0], config), config);
  assert.ok(metrics.minSingularValue < 1e-8);
  assert.equal(metrics.conditionNumber, Infinity);
  const safety = evaluateSafety(commandsToModelRad([0, 0, 0, 0], config), config);
  assert.equal(safety.singularitySafe, false);
});

test("joint interpolation path reports aggregate metrics", () => {
  const config = createConfig();
  const result = evaluateJointPath([0, 30, -70, 25], [20, 42, -82, 18], config, 20);
  assert.equal(result.samples.length > 5, true);
  assert.ok(Number.isFinite(result.minimumSigma));
  assert.ok(Number.isFinite(result.minimumMargin));
  assert.ok(result.maximumCondition > 0);
});

test("unreachable far target returns no solution", () => {
  const config = createConfig();
  const result = solveInverse({ x: 2, y: 0, z: 2 }, config, { multiStart: true, checkPath: false });
  assert.equal(result.reachable, false);
  assert.equal(result.chosen, null);
});

test("configurable joint-axis Jacobian matches finite differences", () => {
  const config = createConfig({ jointAxes: ["yaw", "pitch", "yaw", "pitch"] });
  const model = commandsToModelRad([18, 37, -42, 28], config);
  const jacobian = positionJacobian(model, config);
  const epsilon = 1e-6;
  for (let joint = 0; joint < 4; joint += 1) {
    const plus = [...model];
    const minus = [...model];
    plus[joint] += epsilon;
    minus[joint] -= epsilon;
    const a = forwardKinematicsModel(plus, config).pose;
    const b = forwardKinematicsModel(minus, config).pose;
    const numeric = [(a.x - b.x), (a.y - b.y), (a.z - b.z)].map((value) => value / (2 * epsilon));
    for (let axis = 0; axis < 3; axis += 1) close(jacobian[axis][joint + 1], numeric[axis], 2e-6);
  }
});

test("numeric IK solves a mixed yaw-pitch axis layout", () => {
  const config = createConfig({ jointAxes: ["yaw", "pitch", "yaw", "pitch"] });
  const target = forwardKinematics([25, 35, -55, 40], config).pose;
  const result = solveInverse(target, config, {
    currentCommandsDeg: [0, 20, -40, 10],
    multiStart: true,
    checkPath: false,
  });
  assert.equal(result.method, "numeric-dls");
  assert.equal(result.reachable, true);
  assert.ok(result.chosen.residual < 8e-4);
});

test("J3 is forced to yaw above the fixed-pitch J4 node", () => {
  const config = createConfig({ jointAxes: ["pitch", "pitch", "pitch", "yaw"] });
  assert.deepEqual(config.jointAxes, ["pitch", "pitch", "yaw", "pitch"]);
  const neutral = forwardKinematics([20, -30, 0, 0], config);
  const yawed = forwardKinematics([20, -30, 90, 0], config);
  const pitched = forwardKinematics([20, -30, 90, 90], config);

  close(neutral.jointOrigins[2][0], yawed.jointOrigins[2][0]);
  close(neutral.jointOrigins[2][1], yawed.jointOrigins[2][1]);
  close(neutral.jointOrigins[2][2], yawed.jointOrigins[2][2]);
  assert.ok(Math.hypot(...neutral.wrist.map((value, index) => value - yawed.wrist[index])) > 0.1);

  close(yawed.wrist[0], pitched.wrist[0]);
  close(yawed.wrist[1], pitched.wrist[1]);
  close(yawed.wrist[2], pitched.wrist[2]);
  close(yawed.points.at(-2).x, pitched.points.at(-2).x);
  close(yawed.points.at(-2).y, pitched.points.at(-2).y);
  close(yawed.points.at(-2).z, pitched.points.at(-2).z);
  close(Math.hypot(...pitched.terminalOffset), config.links.terminal);
  assert.ok(Math.abs(yawed.pose.z - pitched.pose.z) > 0.05);
});

test("lift axis translates the full arm and contributes a Z Jacobian column", () => {
  const low = createConfig({ linearAxis: { position: 0.1 } });
  const high = createConfig({ linearAxis: { position: 0.3 } });
  const commands = [15, 35, -60, 20];
  const lowPose = forwardKinematics(commands, low).pose;
  const highPose = forwardKinematics(commands, high).pose;
  close(highPose.x, lowPose.x);
  close(highPose.y, lowPose.y);
  close(highPose.z - lowPose.z, 0.2);
  const jacobian = positionJacobian(commandsToModelRad(commands, high), high);
  close(jacobian[0][0], 0);
  close(jacobian[1][0], 0);
  close(jacobian[2][0], 1);
});

test("world-aligned tool segment stays parallel to the vertical rail", () => {
  const config = createConfig({ jointAxes: ["yaw", "pitch", "yaw", "pitch"] });
  const fk = forwardKinematics([42, -25, 55, 30], config);
  const wrist = fk.points.at(-3);
  const verticalEnd = fk.points.at(-2);
  close(verticalEnd.x, wrist.x);
  close(verticalEnd.y, wrist.y);
  close(verticalEnd.z - wrist.z, config.toolAssembly.verticalDirection * config.links.tool);
});

test("IK can search the lift-axis travel for a high target", () => {
  const config = createConfig({ linearAxis: { position: 0 } });
  const result = solveInverse({ x: 0.2, y: 0, z: 0.9 }, config, {
    currentCommandsDeg: [0, 20, -40, 20],
    multiStart: true,
    optimizeSlide: true,
    checkPath: true,
  });
  assert.equal(result.reachable, true);
  assert.equal(result.safeReachable, true);
  assert.ok(result.chosen.slidePosition > 0);
  assert.ok(result.chosen.residual < 5e-5);
  assert.equal(result.chosen.path.safe, true);
});

