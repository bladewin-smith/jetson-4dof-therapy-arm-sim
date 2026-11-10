import {
  createConfig,
  configurationSummary,
  evaluateJointPath,
  evaluateSafety,
  forwardKinematics,
  inTherapyZone,
  sampleWorkspace,
  solveInverse,
  commandsToModelRad,
} from "./kinematics.js";
import { ArmRenderer } from "./renderer.js";

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
const format = (value, digits = 3) => Number.isFinite(value) ? value.toFixed(digits) : "∞";

const state = {
  config: createConfig(),
  commands: [20, 60, -100, 25],
  target: { x: 0.46, y: 0.12, z: 0.38 },
  workspace: [],
  lastIK: null,
  scanResult: null,
};

const canvas = $("#arm-canvas");
const renderer = new ArmRenderer(canvas);

function setStatus(kind, title, detail) {
  const status = $("#status-card");
  status.className = `status-card ${kind}`;
  status.querySelector("strong").textContent = title;
  status.querySelector("span").textContent = detail;
}

function updateDirectionButtons() {
  $$("[data-direction]").forEach((button) => {
    const index = Number(button.dataset.direction);
    const direction = state.config.directions[index];
    button.textContent = direction > 0 ? "正向 +" : "反向 −";
    button.classList.toggle("reverse", direction < 0);
    button.setAttribute("aria-pressed", String(direction < 0));
  });
}

function updateAxisSelectors() {
  $$('[data-axis]').forEach((select) => {
    select.value = state.config.jointAxes[Number(select.dataset.axis)];
  });
}

function updateJointControls() {
  state.commands.forEach((value, index) => {
    const range = $(`#q${index + 1}`);
    const number = $(`#q${index + 1}-value`);
    const [modelMin, modelMax] = state.config.jointLimitsDeg[index];
    const direction = state.config.directions[index];
    const commandMin = direction > 0 ? modelMin : -modelMax;
    const commandMax = direction > 0 ? modelMax : -modelMin;
    range.min = commandMin;
    range.max = commandMax;
    number.min = commandMin;
    number.max = commandMax;
    range.value = value;
    number.value = value.toFixed(1);
  });
  updateSlideControls();
}

function updateSlideControls() {
  const range = $("#slide-position");
  const number = $("#slide-position-value");
  const { minPosition, maxPosition, position } = state.config.linearAxis;
  range.min = minPosition;
  range.max = maxPosition;
  number.min = minPosition;
  number.max = maxPosition;
  range.value = position;
  number.value = position.toFixed(3);
}

function clampCommandsToLimits() {
  state.commands = state.commands.map((value, index) => {
    const [modelMin, modelMax] = state.config.jointLimitsDeg[index];
    const direction = state.config.directions[index];
    const commandMin = direction > 0 ? modelMin : -modelMax;
    const commandMax = direction > 0 ? modelMax : -modelMin;
    return Math.max(commandMin, Math.min(commandMax, value));
  });
}

function updateZoneSummary() {
  const zone = state.config.therapyZone;
  $("#zone-summary").textContent = `X ${format(zone.x[0], 2)}–${format(zone.x[1], 2)} m · Y ${format(zone.y[0], 2)}–${format(zone.y[1], 2)} m · Z ${format(zone.z[0], 2)}–${format(zone.z[1], 2)} m`;
}

function populateConfigInputs() {
  const { links, linearAxis, toolAssembly, jointLimitsDeg: limits, therapyZone: zone, safety } = state.config;
  $("#link-base").value = links.baseHeight;
  $("#link-upper").value = links.upperArm;
  $("#link-forearm").value = links.forearm;
  $("#link-tool").value = links.tool;
  $("#link-terminal").value = links.terminal;
  $("#slide-min").value = linearAxis.minPosition;
  $("#slide-max").value = linearAxis.maxPosition;
  $("#vertical-direction").value = String(toolAssembly.verticalDirection);
  limits.forEach(([min, max], index) => {
    $(`#limit-q${index + 1}-min`).value = min;
    $(`#limit-q${index + 1}-max`).value = max;
  });
  for (const axis of ["x", "y", "z"]) {
    $(`#zone-${axis}-min`).value = zone[axis][0];
    $(`#zone-${axis}-max`).value = zone[axis][1];
  }
  $("#threshold-sigma").value = safety.minSingularValue;
  $("#threshold-condition").value = safety.maxConditionNumber;
  $("#threshold-margin").value = safety.minJointMarginDeg;
  updateZoneSummary();
}

function applyConfiguration() {
  const error = $("#config-error");
  error.textContent = "";
  error.classList.remove("ok");
  const values = {
    links: {
      baseHeight: Number($("#link-base").value),
      upperArm: Number($("#link-upper").value),
      forearm: Number($("#link-forearm").value),
      tool: Number($("#link-tool").value),
      terminal: Number($("#link-terminal").value),
    },
    linearAxis: {
      minPosition: Number($("#slide-min").value),
      maxPosition: Number($("#slide-max").value),
      position: state.config.linearAxis.position,
    },
    toolAssembly: {
      verticalDirection: Number($("#vertical-direction").value),
    },
    jointLimitsDeg: Array.from({ length: 4 }, (_, index) => [
      Number($(`#limit-q${index + 1}-min`).value),
      Number($(`#limit-q${index + 1}-max`).value),
    ]),
    therapyZone: Object.fromEntries(["x", "y", "z"].map((axis) => [axis, [
      Number($(`#zone-${axis}-min`).value),
      Number($(`#zone-${axis}-max`).value),
    ]])),
    safety: {
      minSingularValue: Number($("#threshold-sigma").value),
      maxConditionNumber: Number($("#threshold-condition").value),
      minJointMarginDeg: Number($("#threshold-margin").value),
    },
  };
  const allNumbers = [
    ...Object.values(values.links),
    ...Object.values(values.linearAxis),
    ...values.jointLimitsDeg.flat(),
    ...Object.values(values.therapyZone).flat(),
    ...Object.values(values.safety),
  ];
  if (!allNumbers.every(Number.isFinite)) {
    error.textContent = "所有参数都必须是有效数字。";
    return;
  }
  if (Object.values(values.links).some((value) => value <= 0)) {
    error.textContent = "连杆尺寸必须大于 0。";
    return;
  }
  if (values.linearAxis.minPosition >= values.linearAxis.maxPosition) {
    error.textContent = "升降轴行程下限必须小于上限。";
    return;
  }
  values.linearAxis.position = Math.max(
    values.linearAxis.minPosition,
    Math.min(values.linearAxis.maxPosition, values.linearAxis.position),
  );
  if (values.jointLimitsDeg.some(([min, max]) => min >= max) || Object.values(values.therapyZone).some(([min, max]) => min >= max)) {
    error.textContent = "每组下限必须小于上限。";
    return;
  }
  state.config = createConfig({
    links: values.links,
    linearAxis: values.linearAxis,
    toolAssembly: values.toolAssembly,
    jointLimitsDeg: values.jointLimitsDeg,
    jointAxes: state.config.jointAxes,
    directions: state.config.directions,
    therapyZone: values.therapyZone,
    safety: { ...state.config.safety, ...values.safety },
  });
  clampCommandsToLimits();
  state.workspace = sampleWorkspace(state.config, 1200);
  state.lastIK = null;
  state.scanResult = null;
  $("#coverage-card").hidden = true;
  updateJointControls();
  updateZoneSummary();
  render();
  error.textContent = "参数已应用。请重新运行治疗空间扫描。";
  error.classList.add("ok");
}

function render() {
  const safety = evaluateSafety(commandsToModelRad(state.commands, state.config), state.config);
  const pose = safety.fk.pose;
  const path = state.lastIK?.chosen?.path?.samples || [];
  renderer.update({
    fk: safety.fk,
    safe: safety.safe,
    rail: {
      bottom: 0,
      top: state.config.links.baseHeight + state.config.linearAxis.maxPosition,
      carriage: state.config.links.baseHeight + state.config.linearAxis.position,
    },
    target: state.target,
    zone: state.config.therapyZone,
    workspace: state.workspace,
    path,
  });

  $("#tcp-position").textContent = `${format(pose.x)} / ${format(pose.y)} / ${format(pose.z)} m`;
  $("#tcp-orientation").textContent = `偏航 ${format(pose.yawDeg, 1)}° · 俯仰 ${format(pose.pitchDeg, 1)}°`;
  $("#sigma-value").textContent = format(safety.metrics.minSingularValue, 4);
  $("#condition-value").textContent = format(safety.metrics.conditionNumber, 1);
  $("#margin-value").textContent = `${format(safety.jointMarginDeg, 1)}°`;
  $("#zone-value").textContent = inTherapyZone(pose, state.config.therapyZone) ? "内部" : "外部";
  $("#slide-value").textContent = `${format(state.config.links.baseHeight + state.config.linearAxis.position, 3)} m`;

  const meter = Math.min(100, Math.max(0, safety.metrics.minSingularValue / (state.config.safety.minSingularValue * 2.5) * 100));
  $("#sigma-meter").style.width = `${meter}%`;
  $("#sigma-meter").className = safety.singularitySafe ? "meter-fill safe" : "meter-fill danger";

  if (safety.safe) {
    setStatus("safe", "当前姿态通过基础安全检查", "升降行程、关节限位、位置雅可比、底座禁入区与地面约束均通过");
  } else {
    setStatus("danger", "当前姿态不安全", safety.reasons.join(" · ") || "未知约束失败");
  }
}

function bindJointInputs() {
  const updateSlide = (raw) => {
    const { minPosition, maxPosition } = state.config.linearAxis;
    state.config.linearAxis.position = Math.max(minPosition, Math.min(maxPosition, Number(raw)));
    updateSlideControls();
    state.workspace = sampleWorkspace(state.config, 1200);
    state.lastIK = null;
    state.scanResult = null;
    $("#coverage-card").hidden = true;
    render();
  };
  $("#slide-position").addEventListener("input", (event) => updateSlide(event.target.value));
  $("#slide-position-value").addEventListener("change", (event) => updateSlide(event.target.value));
  for (let index = 0; index < 4; index += 1) {
    const range = $(`#q${index + 1}`);
    const number = $(`#q${index + 1}-value`);
    const update = (raw) => {
      state.commands[index] = Number(raw);
      updateJointControls();
      state.lastIK = null;
      render();
    };
    range.addEventListener("input", () => update(range.value));
    number.addEventListener("change", () => update(number.value));
  }
  $$("[data-direction]").forEach((button) => {
    button.addEventListener("click", () => {
      const index = Number(button.dataset.direction);
      state.config.directions[index] *= -1;
      clampCommandsToLimits();
      updateDirectionButtons();
      updateJointControls();
      state.workspace = sampleWorkspace(state.config, 1200);
      state.lastIK = null;
      render();
    });
  });
  $$("[data-axis]").forEach((select) => {
    select.addEventListener("change", () => {
      const index = Number(select.dataset.axis);
      state.config.jointAxes[index] = select.value;
      state.workspace = sampleWorkspace(state.config, 1200);
      state.lastIK = null;
      state.scanResult = null;
      $("#coverage-card").hidden = true;
      render();
    });
  });
}

function readTarget() {
  state.target = {
    x: Number($("#target-x").value),
    y: Number($("#target-y").value),
    z: Number($("#target-z").value),
  };
}

function solveTarget() {
  readTarget();
  const multiStart = $("#multi-start").checked;
  const optimizeSlide = $("#auto-slide").checked;
  const result = solveInverse(state.target, state.config, {
    multiStart,
    optimizeSlide,
    currentCommandsDeg: state.commands,
    checkPath: true,
  });
  state.lastIK = result;

  if (!result.reachable) {
    render();
    setStatus("danger", "目标不可达", "当前关节轴组合、连杆长度和关节限位下没有收敛的逆解");
    return;
  }

  const chosen = result.chosen;
  state.config.linearAxis.position = chosen.slidePosition;
  state.commands = chosen.commandsDeg.map((value) => Number(value.toFixed(4)));
  state.workspace = sampleWorkspace(state.config, 1200);
  updateJointControls();
  render();
  if (result.safeReachable) {
    const method = "多初值数值 DLS";
    setStatus("safe", "已找到安全逆解与插值路径", `${method}；P1 ${format(chosen.slidePosition, 3)} m；${result.safeSolutionCount} 个候选通过；残差 ${format(chosen.residual * 1000, 2)} mm`);
  } else {
    setStatus("warn", "仅找到带风险的几何逆解", chosen.safety.reasons.join(" · ") || "关节插值路径穿过风险区");
  }
}

async function scanTherapyZone() {
  const button = $("#scan-zone");
  button.disabled = true;
  button.textContent = "扫描中…";
  const zone = state.config.therapyZone;
  const counts = { total: 0, reachable: 0, safe: 0 };
  const failures = [];
  const nx = 7;
  const ny = 7;
  const nz = 7;
  for (let ix = 0; ix < nx; ix += 1) {
    for (let iy = 0; iy < ny; iy += 1) {
      for (let iz = 0; iz < nz; iz += 1) {
        const target = {
          x: zone.x[0] + (zone.x[1] - zone.x[0]) * ix / (nx - 1),
          y: zone.y[0] + (zone.y[1] - zone.y[0]) * iy / (ny - 1),
          z: zone.z[0] + (zone.z[1] - zone.z[0]) * iz / (nz - 1),
        };
        const result = solveInverse(target, state.config, {
          multiStart: true,
          optimizeSlide: true,
          currentCommandsDeg: state.commands,
          checkPath: false,
        });
        counts.total += 1;
        if (result.reachable) counts.reachable += 1;
        if (result.solutions.some((solution) => solution.safety.safe)) counts.safe += 1;
        else if (failures.length < 24) failures.push(target);
      }
    }
    await new Promise((resolve) => requestAnimationFrame(resolve));
  }
  state.scanResult = { ...counts, failures };
  const percent = counts.safe / counts.total * 100;
  $("#coverage-value").textContent = `${format(percent, 1)}%`;
  $("#coverage-detail").textContent = `${counts.safe}/${counts.total} 个网格点在 P1 行程内存在安全姿态；几何可达 ${counts.reachable}/${counts.total}`;
  $("#coverage-bar").style.width = `${percent}%`;
  $("#coverage-card").hidden = false;
  button.disabled = false;
  button.textContent = "重新扫描治疗空间";
}

function exportReport() {
  const safety = evaluateSafety(commandsToModelRad(state.commands, state.config), state.config);
  const report = {
    generatedAt: new Date().toISOString(),
    disclaimer: "Feasibility simulation only. Not a medical-device safety certification.",
    configuration: configurationSummary(state.config),
    currentCommandsDeg: state.commands,
    currentPose: safety.fk.pose,
    currentSafety: {
      safe: safety.safe,
      reasons: safety.reasons,
      minSingularValue: safety.metrics.minSingularValue,
      conditionNumber: safety.metrics.conditionNumber,
      jointMarginDeg: safety.jointMarginDeg,
    },
    therapyZoneScan: state.scanResult,
  };
  const blob = new Blob([JSON.stringify(report, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = `therapy-arm-feasibility-${new Date().toISOString().slice(0, 10)}.json`;
  link.click();
  URL.revokeObjectURL(url);
}

function reset() {
  state.config = createConfig();
  state.commands = [20, 60, -100, 25];
  state.target = { x: 0.46, y: 0.12, z: 0.38 };
  state.lastIK = null;
  state.scanResult = null;
  $("#target-x").value = state.target.x;
  $("#target-y").value = state.target.y;
  $("#target-z").value = state.target.z;
  $("#multi-start").checked = true;
  $("#auto-slide").checked = true;
  $("#coverage-card").hidden = true;
  state.workspace = sampleWorkspace(state.config, 1200);
  populateConfigInputs();
  updateAxisSelectors();
  updateDirectionButtons();
  updateJointControls();
  render();
}

bindJointInputs();
$("#solve-ik").addEventListener("click", solveTarget);
$("#scan-zone").addEventListener("click", scanTherapyZone);
$("#export-report").addEventListener("click", exportReport);
$("#reset-all").addEventListener("click", reset);
$("#apply-config").addEventListener("click", applyConfiguration);
$("#target-form").addEventListener("submit", (event) => { event.preventDefault(); solveTarget(); });

state.workspace = sampleWorkspace(state.config, 1200);
populateConfigInputs();
updateAxisSelectors();
updateDirectionButtons();
updateJointControls();
render();

