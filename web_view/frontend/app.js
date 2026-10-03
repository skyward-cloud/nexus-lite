import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const $ = (id) => document.getElementById(id);

const el = {
  connected: $("s-connected"),
  armed: $("s-armed"),
  mode: $("s-mode"),
  life: $("s-life"),
  batt: $("s-batt"),
  fail: $("s-fail"),
  pos: $("s-pos"),
  vel: $("s-vel"),
  task: $("s-task"),
  err: $("s-err"),
  rth: $("s-rth"),
  cloudCount: $("cloud-count"),
  cloudHz: $("cloud-hz"),
  hint: $("cmd-hint"),
  takeoffZ: $("takeoff-z"),
  gotoX: $("goto-x"),
  gotoY: $("goto-y"),
  gotoZ: $("goto-z"),
  gotoYaw: $("goto-yaw"),
  nudgeStep: $("nudge-step"),
  nudgeYawDeg: $("nudge-yaw-deg"),
  hudPos: $("hud-pos"),
  hudMode: $("hud-mode"),
};

let api = null;
let lastCloudTs = 0;
let lastStatus = null;
let droneMesh = null;
let pointsObj = null;
let pointsGeo = null;
let posSeq = 0;
let nudgeTimer = null;
let activeNudge = null;

function fmt3(v) {
  return Number(v).toFixed(2);
}

function fmtVec(arr) {
  if (!arr || arr.length < 3) return "—";
  return `${fmt3(arr[0])}, ${fmt3(arr[1])}, ${fmt3(arr[2])}`;
}

function wrapPi(a) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

function yawFromQuat(q) {
  if (!q || q.length < 4) return 0;
  const qw = Number(q[0]);
  const qx = Number(q[1]);
  const qy = Number(q[2]);
  const qz = Number(q[3]);
  return Math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

function currentPose() {
  if (!lastStatus || !lastStatus.position) {
    return { x: 0, y: 0, z: 0, yaw: 0 };
  }
  return {
    x: Number(lastStatus.position[0]) || 0,
    y: Number(lastStatus.position[1]) || 0,
    z: Number(lastStatus.position[2]) || 0,
    yaw: yawFromQuat(lastStatus.attitude_quat),
  };
}

function setBool(node, yes, yesText, noText) {
  node.textContent = yes ? yesText : noText;
  node.className = yes ? "on" : "off";
}

function applyStatus(s) {
  lastStatus = s;
  setBool(el.connected, !!s.connected, "已连接", "断开");
  setBool(el.armed, !!s.armed, "解锁", "上锁");
  el.mode.textContent = s.mode || "—";
  el.life.textContent = s.lifecycle_state || "—";
  el.batt.textContent = `${(Number(s.battery_remaining || 0) * 100).toFixed(0)}%`;
  if (s.failsafe) {
    el.fail.textContent = s.failsafe_reason || "YES";
    el.fail.className = "off";
  } else {
    el.fail.textContent = "否";
    el.fail.className = "on";
  }
  el.pos.textContent = fmtVec(s.position);
  el.vel.textContent = fmtVec(s.velocity);
  el.task.textContent = s.active_task_id || "—";
  el.err.textContent = s.error || "—";
  const rth = [s.rth_phase, s.rth_detail].filter(Boolean).join(" · ");
  el.rth.textContent = rth || "—";
  el.hudPos.textContent = `pos ${fmtVec(s.position)}`;
  el.hudMode.textContent = `mode ${s.mode || "—"} · ${s.lifecycle_state || "—"}`;

  if (droneMesh && s.position) {
    droneMesh.position.set(
      Number(s.position[0]) || 0,
      Number(s.position[2]) || 0,
      -Number(s.position[1]) || 0
    );
    const q = s.attitude_quat;
    if (q && q.length === 4) {
      // ENU FLU quat (w,x,y,z) → Three Y-up 近似：绕 Y 映射
      const qw = Number(q[0]);
      const qx = Number(q[1]);
      const qy = Number(q[2]);
      const qz = Number(q[3]);
      droneMesh.quaternion.set(qx, qz, -qy, qw);
    }
  }
}

function applyCloud(cloud) {
  const xyz = cloud.xyz || [];
  const count = cloud.count || Math.floor(xyz.length / 3);
  el.cloudCount.textContent = String(count);
  const now = performance.now();
  if (lastCloudTs > 0) {
    const hz = 1000 / Math.max(1, now - lastCloudTs);
    el.cloudHz.textContent = `${hz.toFixed(1)} Hz`;
  }
  lastCloudTs = now;

  if (!pointsGeo || !pointsObj) return;

  // ENU (x,y,z) → Three (x, z, -y)
  const arr = new Float32Array(count * 3);
  for (let i = 0; i < count; i++) {
    const ix = i * 3;
    const x = xyz[ix] || 0;
    const y = xyz[ix + 1] || 0;
    const z = xyz[ix + 2] || 0;
    arr[ix] = x;
    arr[ix + 1] = z;
    arr[ix + 2] = -y;
  }
  pointsGeo.setAttribute("position", new THREE.BufferAttribute(arr, 3));
  pointsGeo.computeBoundingSphere();
  pointsObj.visible = count > 0;
}

async function pollLoop() {
  if (!api) return;
  try {
    const data = await api.poll();
    if (data && data.status) applyStatus(data.status);
    if (data && data.cloud) applyCloud(data.cloud);
  } catch (err) {
    // pywebview 偶发竞态，忽略单次失败
  }
  requestAnimationFrame(() => {
    setTimeout(pollLoop, 40);
  });
}

async function submit(taskType, payload, deadlineMs = 0) {
  if (!api) {
    el.hint.textContent = "后端未就绪";
    return;
  }
  el.hint.textContent = `发送 ${taskType}…`;
  const res = await api.submit_task(taskType, payload, deadlineMs, "");
  if (res && res.ok) {
    el.hint.textContent = `已发布 ${res.task_id}`;
  } else {
    el.hint.textContent = `失败: ${(res && res.error) || "unknown"}`;
  }
}

async function submitPosition(x, y, z, yaw, label = "pos") {
  posSeq += 1;
  if (!api) {
    el.hint.textContent = "后端未就绪";
    return;
  }
  const payload = {
    x,
    y,
    z,
    yaw,
    arrival_radius_m: 0.3,
    arrival_dz_m: 0.2,
    arrival_speed_mps: 0.5,
  };
  const tid = `web-${label}-${posSeq}`;
  el.hint.textContent = `指点 (${fmt3(x)}, ${fmt3(y)}, ${fmt3(z)}) yaw=${fmt3(yaw)}`;
  const res = await api.submit_task("POSITION_CONTROL", payload, 60000, tid);
  if (res && res.ok) {
    el.hint.textContent = `已发布 ${res.task_id}`;
  } else {
    el.hint.textContent = `失败: ${(res && res.error) || "unknown"}`;
  }
}

function fillPoseInputs() {
  const p = currentPose();
  el.gotoX.value = p.x.toFixed(2);
  el.gotoY.value = p.y.toFixed(2);
  el.gotoZ.value = p.z.toFixed(2);
  el.gotoYaw.value = p.yaw.toFixed(3);
  el.hint.textContent = "已填入当前位姿";
}

function applyNudge(dir) {
  const pose = currentPose();
  const step = Math.max(0.05, Number(el.nudgeStep.value) || 0.5);
  const yawStep = ((Number(el.nudgeYawDeg.value) || 15) * Math.PI) / 180;
  let { x, y, z, yaw } = pose;

  if (dir === "hover") {
    submitPosition(x, y, z, yaw, "hover");
    return;
  }

  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  // 与 keyboard teleop 一致：机体前/左（v_left 正=左）
  let fwd = 0;
  let left = 0;
  if (dir === "fwd") fwd = step;
  else if (dir === "back") fwd = -step;
  else if (dir === "left") left = step;
  else if (dir === "right") left = -step;
  else if (dir === "up") z += step;
  else if (dir === "down") z -= step;
  else if (dir === "yaw_l") yaw = wrapPi(yaw + yawStep);
  else if (dir === "yaw_r") yaw = wrapPi(yaw - yawStep);

  x += c * fwd - s * left;
  y += s * fwd + c * left;
  submitPosition(x, y, z, yaw, "nudge");
}

function stopNudgeHold() {
  if (nudgeTimer != null) {
    clearInterval(nudgeTimer);
    nudgeTimer = null;
  }
  if (activeNudge) {
    activeNudge.classList.remove("active");
    activeNudge = null;
  }
}

function startNudgeHold(btn, dir) {
  stopNudgeHold();
  activeNudge = btn;
  btn.classList.add("active");
  applyNudge(dir);
  // 按住连续下发（约 5Hz）
  nudgeTimer = setInterval(() => applyNudge(dir), 200);
}

function wireActions() {
  document.querySelectorAll("[data-action]").forEach((btn) => {
    btn.addEventListener("click", () => {
      const action = btn.getAttribute("data-action");
      const z = Number(el.takeoffZ.value) || 2.0;
      switch (action) {
        case "arm":
          submit("ARMING", { arm: true });
          break;
        case "disarm":
          submit("ARMING", { arm: false });
          break;
        case "takeoff": {
          const px = lastStatus && lastStatus.position ? Number(lastStatus.position[0]) : 0;
          const py = lastStatus && lastStatus.position ? Number(lastStatus.position[1]) : 0;
          const yaw = yawFromQuat(lastStatus && lastStatus.attitude_quat);
          submitPosition(px, py, z, yaw, "takeoff");
          break;
        }
        case "land":
          submit("MODE_SWITCH", { mode: "LAND" });
          break;
        case "hold":
          submit("MODE_SWITCH", { mode: "HOLD" });
          break;
        case "posctl":
          submit("MODE_SWITCH", { mode: "POSCTL" });
          break;
        case "offboard":
          submit("MODE_SWITCH", { mode: "OFFBOARD" });
          break;
        case "rth":
          submit("MODE_SWITCH", { mode: "RETURN_HOME" });
          break;
        case "kill":
          if (window.confirm("确认发送紧急停止？")) {
            submit("KILL_SWITCH", {});
          }
          break;
        case "fill-pose":
          fillPoseInputs();
          break;
        case "goto": {
          const gx = Number(el.gotoX.value);
          const gy = Number(el.gotoY.value);
          const gz = Number(el.gotoZ.value);
          const gyaw = Number(el.gotoYaw.value);
          if ([gx, gy, gz, gyaw].some((v) => Number.isNaN(v))) {
            el.hint.textContent = "指点坐标无效";
            break;
          }
          submitPosition(gx, gy, gz, gyaw, "goto");
          break;
        }
        default:
          break;
      }
    });
  });

  document.querySelectorAll("[data-nudge]").forEach((btn) => {
    const dir = btn.getAttribute("data-nudge");
    const onDown = (ev) => {
      ev.preventDefault();
      startNudgeHold(btn, dir);
    };
    const onUp = (ev) => {
      ev.preventDefault();
      stopNudgeHold();
    };
    btn.addEventListener("mousedown", onDown);
    btn.addEventListener("mouseup", onUp);
    btn.addEventListener("mouseleave", onUp);
    btn.addEventListener("touchstart", onDown, { passive: false });
    btn.addEventListener("touchend", onUp);
    btn.addEventListener("touchcancel", onUp);
  });

  window.addEventListener("mouseup", stopNudgeHold);
  window.addEventListener("blur", stopNudgeHold);
}

function initScene() {
  const canvas = $("scene");
  const renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    alpha: true,
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x000000, 0);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 500);
  camera.position.set(12, 8, 12);

  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.target.set(0, 1, 0);

  const ambient = new THREE.AmbientLight(0xb8c8d4, 0.55);
  scene.add(ambient);
  const dir = new THREE.DirectionalLight(0xe8f0f4, 0.85);
  dir.position.set(6, 12, 4);
  scene.add(dir);

  const grid = new THREE.GridHelper(40, 40, 0x2a3a44, 0x1a252c);
  grid.position.y = 0;
  scene.add(grid);

  // FLU：X 前(红) Y 左(绿) Z 上(蓝)。Three 为 Y-up，辅助轴绕 X 转 -90°
  const axes = new THREE.AxesHelper(2.5);
  axes.rotation.x = -Math.PI / 2;
  scene.add(axes);

  // 机体：简洁机臂十字
  const drone = new THREE.Group();
  const bodyMat = new THREE.MeshStandardMaterial({
    color: 0x2bb8a6,
    metalness: 0.2,
    roughness: 0.45,
  });
  const armMat = new THREE.MeshStandardMaterial({
    color: 0xd7e0e8,
    metalness: 0.1,
    roughness: 0.55,
  });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.12, 0.35), bodyMat);
  drone.add(body);
  const armGeo = new THREE.BoxGeometry(1.2, 0.04, 0.08);
  const arm1 = new THREE.Mesh(armGeo, armMat);
  const arm2 = new THREE.Mesh(armGeo, armMat);
  arm2.rotation.y = Math.PI / 2;
  drone.add(arm1, arm2);
  scene.add(drone);
  droneMesh = drone;

  pointsGeo = new THREE.BufferGeometry();
  const mat = new THREE.PointsMaterial({
    size: 0.06,
    color: 0x7fd4c8,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.85,
  });
  pointsObj = new THREE.Points(pointsGeo, mat);
  pointsObj.visible = false;
  scene.add(pointsObj);

  function resize() {
    const parent = canvas.parentElement;
    const w = parent.clientWidth || 1;
    const h = parent.clientHeight || 1;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }
  window.addEventListener("resize", resize);
  resize();

  const pulse = { t: 0 };
  function frame(t) {
    pulse.t = t * 0.001;
    controls.update();
    if (pointsObj && pointsObj.material) {
      pointsObj.material.opacity = 0.72 + 0.12 * Math.sin(pulse.t * 1.6);
    }
    if (droneMesh) {
      const s = 1 + 0.03 * Math.sin(pulse.t * 2.2);
      droneMesh.scale.setScalar(s);
    }
    renderer.render(scene, camera);
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
}

function waitApi() {
  return new Promise((resolve) => {
    const tick = () => {
      if (window.pywebview && window.pywebview.api) {
        resolve(window.pywebview.api);
        return;
      }
      setTimeout(tick, 50);
    };
    window.addEventListener("pywebviewready", () => {
      if (window.pywebview && window.pywebview.api) resolve(window.pywebview.api);
    });
    tick();
  });
}

async function boot() {
  initScene();
  wireActions();
  api = await waitApi();
  try {
    await api.ready();
    el.hint.textContent = "已连接后端，可下发任务";
  } catch (e) {
    el.hint.textContent = "API ready 失败";
  }
  pollLoop();
}

boot();
