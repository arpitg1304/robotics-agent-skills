#!/usr/bin/env node
// ARMATURE backend for the robotics agent harness.
//
// Speaks the harness backend protocol: one JSON request per line on stdin, one
// JSON response per line on stdout. Loads the simulator from a local ARMATURE
// checkout at runtime; no ARMATURE source is copied into this repository.
//
// Usage: node bridge.mjs <path-to-ARMATURE>
import fs from "node:fs/promises";
import path from "node:path";
import readline from "node:readline";
import vm from "node:vm";
import { createRequire } from "node:module";
import { pathToFileURL } from "node:url";
import { performance } from "node:perf_hooks";

const PROTOCOL = 1;
const CONTROL_HZ = 30;
const TRACE_STRIDE = 3;

const home = path.resolve(process.argv[2] || process.env.ARMATURE_HOME || "");

async function loadSimulator() {
  const require = createRequire(path.join(home, "package.json"));
  const esbuild = await import(pathToFileURL(require.resolve("esbuild")).href);
  const build = esbuild.build || esbuild.default.build;
  const bundle = await build({
    stdin: {
      contents:
        "import { RobotEnvironment } from './src/simulation/environment.js';" +
        "import { ROBOT_MODELS } from './src/robots/models.js';" +
        "globalThis.Environment = RobotEnvironment; globalThis.ROBOT_MODELS = ROBOT_MODELS;",
      resolveDir: home,
    },
    bundle: true,
    write: false,
    format: "iife",
    loader: { ".b64": "text" },
    logLevel: "silent",
  });
  // The simulator logs to console; stdout is reserved for protocol messages.
  const context = vm.createContext({
    console: { warn() {}, log() {}, error() {} },
    atob,
    TextDecoder,
    TextEncoder,
    performance,
  });
  vm.runInContext(
    (await fs.readFile(path.join(home, "vendor/three.min.js"), "utf8")) +
      bundle.outputFiles[0].text,
    context,
  );
  const pkg = JSON.parse(await fs.readFile(path.join(home, "package.json"), "utf8"));
  return {
    THREE: context.THREE,
    Environment: context.Environment,
    models: context.ROBOT_MODELS,
    version: pkg.version,
  };
}

// Deterministic noise so a seed reproduces a run exactly.
function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function gaussian(random) {
  const u = Math.max(random(), 1e-12);
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * random());
}

const sim = await loadSimulator();
const { THREE } = sim;

let rig = null; // rig manifest from `configure`
let env = null;
let episode = null; // per-episode state, null until `reset`

function fail(message) {
  throw Error(message);
}
function needEpisode() {
  if (!episode) fail("No episode: call reset first.");
  return episode;
}
function vector(value, length, what) {
  if (!Array.isArray(value) || value.length !== length || !value.every(Number.isFinite))
    fail(`${what} must be ${length} finite numbers.`);
  return value;
}

// --- peripherals -----------------------------------------------------------

function trackerSpec() {
  const spec = rig.peripherals?.tracker;
  if (!spec) return null;
  const mount = spec.mount || {};
  return {
    rateHz: spec.rate_hz ?? CONTROL_HZ,
    noiseM: spec.noise_m ?? 0,
    xyz: mount.xyz || [0, 0, 0],
    rpy: mount.rpy || [0, 0, 0],
  };
}

function sampleTracker() {
  const spec = trackerSpec();
  // world_T_tracker; readings are reported in the tracker's own frame.
  const mountQ = new THREE.Quaternion().setFromEuler(new THREE.Euler(...spec.rpy, "ZYX"));
  const inverse = mountQ.clone().invert();
  const origin = new THREE.Vector3(...spec.xyz);
  return {
    stamp: env.steps / CONTROL_HZ,
    frame: "tracker",
    objects: env.objects.map((body, index) => {
      const position = new THREE.Vector3(body.position.x, body.position.y, body.position.z)
        .sub(origin)
        .applyQuaternion(inverse);
      if (spec.noiseM)
        position.add(
          new THREE.Vector3(
            gaussian(episode.random),
            gaussian(episode.random),
            gaussian(episode.random),
          ).multiplyScalar(spec.noiseM),
        );
      const orientation = inverse
        .clone()
        .multiply(
          new THREE.Quaternion(
            body.quaternion.x,
            body.quaternion.y,
            body.quaternion.z,
            body.quaternion.w,
          ),
        );
      return {
        id: index,
        class: env.objectSpecs[index].id,
        position: position.toArray(),
        orientation: orientation.toArray(),
      };
    }),
  };
}

function refreshTracker(force = false) {
  const spec = trackerSpec();
  if (!spec) return;
  const period = Math.max(1, Math.round(CONTROL_HZ / spec.rateHz));
  if (force || env.steps % period === 0) episode.tracker = sampleTracker();
}

function describeDevices() {
  const model = env.model;
  const devices = {
    arm: {
      kind: "actuator",
      type: "serial_arm",
      model: model.label,
      interface: "joint_position",
      control_hz: CONTROL_HZ,
      joints: model.armNames.map((name, i) => ({
        name,
        unit: "rad",
        lower: env.arm[i].limit.lower,
        upper: env.arm[i].limit.upper,
      })),
      frames: {
        world: "right-handed, +Y up, metres; bench top at y = 0",
        base: { parent: "world", xyz: [model.baseX, 0, 0], note: "URDF base, +Z up" },
        tcp: model.tcp,
      },
      read: { position: "rad[]", velocity: "rad/s[]", tcp: "[x, y, z] in world, metres" },
      write: { position: "absolute joint targets, rad[]; servo is velocity limited" },
      services: {
        ik: {
          params: { position: "[x, y, z] in world", seed: "optional joint vector" },
          returns: { position: "joint vector", residual: "metres" },
          note: "Controller IK, position target with a fixed top-down tool orientation.",
        },
        fk: { params: { position: "joint vector" }, returns: { tcp: "[x, y, z] in world" } },
      },
    },
    gripper: {
      kind: "actuator",
      type: model.jawType === "revolute" ? "revolute_jaw" : "parallel_jaw",
      interface: "position",
      unit: model.jawType === "revolute" ? "rad" : "m",
      lower: model.jawLimit.lower,
      upper: model.jawLimit.upper,
      open: model.open,
      closed: model.close,
      speed: model.jawSpeed,
      read: {
        position: "current opening",
        object_detected: "true while both fingers touch the same object",
      },
      write: { position: "absolute opening target" },
    },
  };
  const tracker = trackerSpec();
  if (tracker)
    devices.tracker = {
      kind: "sensor",
      type: "object_tracker",
      rate_hz: tracker.rateHz,
      noise_m: tracker.noiseM,
      frame: "tracker",
      mount: { parent: "world", xyz: tracker.xyz, rpy: tracker.rpy, convention: "world_T_tracker; rpy applied Z-Y-X" },
      read: {
        stamp: "seconds of simulated time when the sample was taken",
        objects: "[{id, class, position [x, y, z], orientation [x, y, z, w]}] in the tracker frame",
      },
      note: "Simulated ideal tracker fed from simulator ground truth plus configured noise. Stands in for a perception stack; it is not a camera.",
    };
  return devices;
}

// --- evaluator (privileged; reported to the developer, never to the app) ----

const AT_TOLERANCE_M = 0.018; // matches ARMATURE's own placement tolerance
const STILL_SPEED = 0.035;
const STABLE_STEPS = 15;
const LIFT_MARGIN_M = 0.01;
const MAX_EVENTS = 300;

function indexOf(name, prefix, count) {
  const match = new RegExp(`^${prefix}_(\\d+)$`).exec(name ?? "");
  if (!match || Number(match[1]) >= count) fail(`Unknown ${prefix} '${name}' in task goal.`);
  return Number(match[1]);
}

// Goal relations: at(object_i, target_j) and on(object_i, object_j).
function parseGoal(goal) {
  return goal.map((item) => {
    const a = indexOf(item.a, "object", env.objects.length);
    if (item.rel === "at") return { rel: "at", a, b: indexOf(item.b, "target", env.goals.length) };
    if (item.rel === "on") return { rel: "on", a, b: indexOf(item.b, "object", env.objects.length) };
    fail(`Unknown goal relation '${item.rel}'. Supported: at, on.`);
  });
}

function relationName({ rel, a, b }) {
  return `${rel}(object_${a},${rel === "at" ? "target" : "object"}_${b})`;
}

function targetDistance(a, b) {
  const p = env.objects[a].position;
  const g = env.goals[b];
  return Math.hypot(p.x - g[0], p.y - g[1], p.z - g[2]);
}

function holds({ rel, a, b }) {
  if (rel === "at") return targetDistance(a, b) < AT_TOLERANCE_M;
  const scale = env.config.sizeScale;
  const top = env.objects[a].position;
  const bottom = env.objects[b].position;
  const gap = ((env.objectSpecs[a].height + env.objectSpecs[b].height) * scale) / 2;
  return (
    Math.abs(top.y - bottom.y - gap) < 0.008 &&
    Math.hypot(top.x - bottom.x, top.z - bottom.z) < (env.objectSpecs[b].grip * scale) / 2
  );
}

function heldObject() {
  if (!env.grasped) return -1;
  const tcp = env.tcp();
  let best = -1;
  let nearest = Infinity;
  env.objects.forEach((o, i) => {
    const d = Math.hypot(o.position.x - tcp.x, o.position.y - tcp.y, o.position.z - tcp.z);
    if (d < nearest) [nearest, best] = [d, i];
  });
  return best;
}

function currentRelations() {
  const relations = new Set();
  const held = heldObject();
  if (held >= 0) relations.add(`held(object_${held})`);
  if (env.config.task === "reach") return relations;
  env.objects.forEach((_, a) => {
    env.goals.forEach((_, b) => {
      if (holds({ rel: "at", a, b })) relations.add(relationName({ rel: "at", a, b }));
    });
    env.objects.forEach((_, b) => {
      if (a !== b && holds({ rel: "on", a, b })) relations.add(relationName({ rel: "on", a, b }));
    });
  });
  return relations;
}

function logEvent(event) {
  if (episode.events.length < MAX_EVENTS)
    episode.events.push({ step: env.steps, mark: episode.mark, ...event });
}

function recordStep() {
  const e = episode;
  const relations = currentRelations();
  for (const relation of relations) if (!e.relations.has(relation)) logEvent({ relation, holds: true });
  for (const relation of e.relations)
    if (!relations.has(relation)) {
      logEvent({ relation, holds: false, gripper_open: env.isOpen });
      const held = /^held\(object_(\d+)\)$/.exec(relation);
      if (held) {
        // Losing an object in mid-air while the gripper is still closed is a slip.
        const i = Number(held[1]);
        const rest = (env.objectSpecs[i].height * env.config.sizeScale) / 2;
        if (!env.isOpen && env.objects[i].position.y > rest + LIFT_MARGIN_M) e.slipped = true;
      } else if (e.goalNames.has(relation) && !e.relations.has(`held(${relation.slice(relation.indexOf("(") + 1, relation.indexOf(","))})`))
        // A satisfied goal relation broke while its object was not in the gripper.
        e.disturbed = relation;
    }
  e.relations = relations;

  const held = heldObject();
  if (held >= 0) {
    const rest = (env.objectSpecs[held].height * env.config.sizeScale) / 2;
    e.milestones[held].grasped = true;
    if (env.objects[held].position.y > rest + LIFT_MARGIN_M) e.milestones[held].lifted = true;
  }

  if (e.customGoal) {
    // The task defines success; ARMATURE's built-in criterion must not end the episode.
    env.terminated = false;
    const settled =
      e.goal.every(holds) &&
      env.isOpen &&
      env.objects.every((o) => o.velocity.length() < STILL_SPEED);
    e.stable = settled ? e.stable + 1 : 0;
    e.success = e.stable >= STABLE_STEPS;
  } else e.success = env.terminated;

  const box = rig.safety?.workspace;
  if (box && !e.success) {
    const tcp = env.tcp().toArray();
    const axis = tcp.findIndex((value, i) => value < box.min[i] || value > box.max[i]);
    if (axis >= 0) {
      e.violation = { step: env.steps, mark: e.mark, tcp, axis: "xyz"[axis], workspace: box };
      logEvent({ safety_violation: `tcp left the workspace along ${"xyz"[axis]}` });
    }
  }
  e.done = e.success || env.truncated || !!e.violation;
  if (env.steps % TRACE_STRIDE === 0 || e.done) traceRow();
}

function traceRow() {
  const e = episode;
  e.trace.push({
    step: env.steps,
    mark: e.mark,
    q: env.q.slice(),
    tcp: env.tcp().toArray(),
    objects: env.objects.map((o) => [o.position.x, o.position.y, o.position.z]),
    grasped: env.grasped,
  });
}

function report() {
  const e = episode;
  const reach = env.config.task === "reach";
  const goal = e.goal.map((relation) => ({
    relation: relationName(relation),
    holds: holds(relation),
    ...(relation.rel === "at" ? { error_m: targetDistance(relation.a, relation.b) } : {}),
  }));
  // Stage credit: each object is grasped, lifted, then left satisfying its goal relations.
  const milestones = e.milestones.map((m, i) => ({
    object: `object_${i}`,
    grasped: m.grasped,
    lifted: m.lifted,
    placed: e.goal.filter((r) => r.a === i).every(holds) && e.goal.some((r) => r.a === i),
  }));
  const stages = milestones.flatMap((m) => [m.grasped, m.lifted, m.placed]);
  // A task can succeed without every stage, e.g. a part that spawned on its target.
  const progress =
    e.success || reach || !stages.length ? Number(e.success) : stages.filter(Boolean).length / stages.length;
  const fell = env.objects.some((o) => o.position.y < -0.1);
  let outcome = "success";
  if (!e.success) {
    if (e.violation) outcome = "safety_violation";
    else if (reach) outcome = "off_target";
    else if (fell) outcome = "fell";
    else if (!milestones.some((m) => m.grasped)) outcome = "never_grasped";
    else if (e.slipped) outcome = "slipped";
    else if (!env.isOpen) outcome = "not_released";
    else if (e.disturbed) outcome = "disturbed";
    else outcome = "off_target";
  }
  const positionErrors = e.beliefs.filter((b) => b.error_m !== undefined).map((b) => b.error_m);
  return {
    success: e.success,
    outcome,
    progress,
    steps: env.steps,
    sim_seconds: env.steps / CONTROL_HZ,
    step_budget: env.config.maxSteps,
    out_of_budget: env.steps >= env.config.maxSteps && !e.success,
    goal,
    goal_error_m: reach ? [env.distance()] : goal.filter((g) => "error_m" in g).map((g) => g.error_m),
    milestones,
    safety_violation: e.violation,
    disturbed: e.disturbed,
    limit_clamps: e.limitClamps,
    last_mark: e.mark,
    // What the app claimed from its sensors, checked against simulator ground truth.
    beliefs: {
      checked: e.beliefs.length,
      wrong: e.beliefs.filter((b) => b.correct === false).length,
      max_position_error_m: positionErrors.length ? Math.max(...positionErrors) : null,
      worst: e.beliefs
        .filter((b) => b.correct === false || b.error_m > 0.005)
        .slice(0, 10),
    },
    events: e.events,
  };
}

// --- protocol handlers -----------------------------------------------------

const handlers = {
  hello() {
    return {
      protocol: PROTOCOL,
      backend: "armature",
      version: sim.version,
      home,
      robots: Object.keys(sim.models),
      control_hz: CONTROL_HZ,
      fidelity:
        "Velocity-limited kinematic servos with approximate collision proxies. Not a validated dynamics model; success here is not evidence of sim-to-real transfer.",
    };
  },

  configure({ rig: manifest }) {
    if (!manifest?.robot) fail("Rig manifest needs a robot.");
    if (!sim.models[manifest.robot])
      fail(`Unknown robot '${manifest.robot}'. Available: ${Object.keys(sim.models).join(", ")}`);
    const box = manifest.safety?.workspace;
    if (box) {
      vector(box.min, 3, "safety.workspace.min");
      vector(box.max, 3, "safety.workspace.max");
    }
    rig = manifest;
    env = new sim.Environment(THREE, manifest.robot);
    episode = null;
    return { devices: describeDevices(), safety: manifest.safety || null };
  },

  reset({ scene = {}, goal = null, seed = 1 }) {
    if (!env) fail("No rig: call configure first.");
    env.reset({ ...scene, seed });
    const reach = env.config.task === "reach";
    const relations = goal
      ? parseGoal(goal)
      : reach
        ? []
        : env.objects.map((_, i) => ({ rel: "at", a: i, b: i }));
    episode = {
      random: mulberry32(seed * 7919 + 17),
      target: env.q.slice(),
      mark: "start",
      goal: relations,
      goalNames: new Set(relations.map(relationName)),
      customGoal: !!goal,
      disturbed: null,
      relations: new Set(),
      milestones: env.objects.map(() => ({ grasped: false, lifted: false })),
      events: [],
      beliefs: [],
      trace: [],
      actions: [],
      marks: [[0, "start"]],
      stable: 0,
      success: false,
      done: false,
      slipped: false,
      violation: null,
      limitClamps: 0,
      tracker: null,
    };
    episode.relations = currentRelations();
    refreshTracker(true);
    traceRow();
    return {
      seed,
      control_hz: CONTROL_HZ,
      step_budget: env.config.maxSteps,
      // Public task layout: what a cell drawing or fixture survey would give you.
      layout: {
        frame: "world",
        goal: relations.map(({ rel, a, b }) => ({
          rel,
          a: `object_${a}`,
          b: `${rel === "at" ? "target" : "object"}_${b}`,
        })),
        targets: reach
          ? [{ name: "reach_target", position: env.reachGoal.slice() }]
          : env.goals.map((position, i) => ({
              name: `target_${i}`,
              position: position.slice(),
              note: "object centre when placed",
            })),
        parts: env.objectSpecs.map((spec, i) => ({
          name: `object_${i}`,
          tracker_id: i,
          class: spec.id,
          height_m: spec.height * env.config.sizeScale,
          grip_width_m: spec.grip * env.config.sizeScale,
        })),
        home_tcp: env.model.homePoint.slice(),
        clearance_height_m: env.model.high,
      },
    };
  },

  read({ device }) {
    needEpisode();
    const stamp = env.steps / CONTROL_HZ;
    if (device === "arm")
      return {
        stamp,
        position: env.q.slice(0, env.n),
        velocity: env.qdot.slice(0, env.n),
        tcp: env.tcp().toArray(),
      };
    if (device === "gripper")
      return { stamp, position: env.q[env.n], object_detected: env.grasped };
    if (device === "tracker" && trackerSpec()) return episode.tracker;
    fail(`Unknown device '${device}'.`);
  },

  write({ device, command = {} }) {
    const e = needEpisode();
    if (device === "arm") {
      const position = vector(command.position, env.n, "arm position");
      position.forEach((value, i) => {
        const { lower, upper } = env.arm[i].limit;
        if (value < lower || value > upper) e.limitClamps++;
        e.target[i] = value;
      });
      return {};
    }
    if (device === "gripper") {
      if (!Number.isFinite(command.position)) fail("gripper position must be a finite number.");
      e.target[env.n] = command.position;
      return {};
    }
    fail(`Device '${device}' does not accept commands.`);
  },

  call({ device, method, params = {} }) {
    needEpisode();
    if (device !== "arm") fail(`Device '${device}' has no services.`);
    if (method === "ik") {
      const position = vector(params.position, 3, "ik position");
      const seed = params.seed ? vector(params.seed, env.n, "ik seed") : env.q.slice(0, env.n);
      const solution = env.ik(new THREE.Vector3(...position), [...seed, env.q[env.n]]);
      return { position: solution.q.slice(0, env.n), residual: solution.residual };
    }
    if (method === "fk") {
      const position = vector(params.position, env.n, "fk position");
      const tcp = env.fk([...position, env.q[env.n]]).toArray();
      env.fk(); // restore the live joint state
      return { tcp };
    }
    fail(`Unknown arm service '${method}'.`);
  },

  step({ n = 1 }) {
    const e = needEpisode();
    if (e.done) fail("Episode ended; call reset.");
    for (let i = 0; i < n && !e.done; i++) {
      e.actions.push(e.target.slice());
      env.step(e.target.slice());
      recordStep();
      refreshTracker();
    }
    return { step: env.steps, stamp: env.steps / CONTROL_HZ, done: e.done };
  },

  mark({ label }) {
    const e = needEpisode();
    e.mark = String(label);
    e.marks.push([env.steps, e.mark]);
    return {};
  },

  // The app states what it believes; the answer is scored but never returned to it.
  claim({ kind, id = null, position = null }) {
    const e = needEpisode();
    const entry = { step: env.steps, mark: e.mark, kind, id };
    if (kind === "object_position") {
      if (!env.objects[id]) fail(`claim: unknown object id ${id}.`);
      const truth = env.objects[id].position;
      const p = vector(position, 3, "claimed position");
      entry.error_m = Math.hypot(p[0] - truth.x, p[1] - truth.y, p[2] - truth.z);
    } else if (kind === "holding") {
      entry.correct = heldObject() === (id ?? -1);
    } else fail(`Unknown claim kind '${kind}'. Supported: object_position, holding.`);
    if (e.beliefs.length < MAX_EVENTS) e.beliefs.push(entry);
    return {};
  },

  finish() {
    needEpisode();
    const result = {
      report: report(),
      trace: episode.trace,
      // Enough to re-simulate the episode elsewhere: the simulator is deterministic per seed.
      replay: {
        robot: env.model.id,
        config: JSON.parse(JSON.stringify(env.config)),
        control_hz: CONTROL_HZ,
        actions: episode.actions,
        marks: episode.marks,
      },
    };
    episode = null;
    return result;
  },
};

const lines = readline.createInterface({ input: process.stdin });
for await (const line of lines) {
  if (!line.trim()) continue;
  let response;
  try {
    const request = JSON.parse(line);
    const handler = handlers[request.op];
    if (!handler) fail(`Unknown op '${request.op}'.`);
    response = { ok: true, result: handler(request) };
  } catch (error) {
    response = { ok: false, error: error.message };
  }
  process.stdout.write(JSON.stringify(response) + "\n");
}
