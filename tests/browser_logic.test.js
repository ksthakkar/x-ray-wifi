"use strict";

const assert = require("node:assert/strict");
const geometry = require("../hub/serve/static/geometry.js");

const view = {
  viewer_position_m: [1.8, -2.0, 1.65],
  yaw_deg: 0,
  pitch_deg: 0,
  horizontal_fov_deg: 50,
  torso_height_m: 1.2,
  presence_threshold: 0.5,
  stale_after_ms: 1000,
};

const estimate = {
  present: 0.9,
  position_m: [1.8, 2.0],
  covariance: [[0.18, 0.03], [0.03, 0.44]],
  height_m: 1.65,
  quality: 0.8,
};

const center = geometry.projectPoint([1.8, 2.0, 1.65], view, 1920, 1080);
assert.equal(center.x, 960);
assert.equal(center.y, 540);

const projected = geometry.projectEstimate(estimate, view, 1920, 1080);
assert.ok(projected.ellipse.major > projected.ellipse.minor);
assert.ok(projected.ellipse.minor >= 0);

assert.equal(geometry.shouldRenderEstimate(estimate, 900, 0, view), true);
assert.equal(geometry.shouldRenderEstimate(estimate, 1001, 0, view), false);
assert.equal(
  geometry.shouldRenderEstimate({ ...estimate, present: 0.49 }, 10, 0, view),
  false
);

console.log("browser geometry and render-state tests passed");
