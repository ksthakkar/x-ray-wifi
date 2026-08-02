(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.XrwGeometry = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const DEG_TO_RAD = Math.PI / 180;
  const CHI_SQUARE_95_2D = 5.991;

  function projectPoint(point, view, width, height) {
    if (width <= 0 || height <= 0) throw new Error("viewport must be positive");
    const fov = view.horizontal_fov_deg;
    if (!(fov > 1 && fov < 179)) throw new Error("horizontal FOV is invalid");

    const yaw = view.yaw_deg * DEG_TO_RAD;
    const pitch = view.pitch_deg * DEG_TO_RAD;
    const sy = Math.sin(yaw), cy = Math.cos(yaw);
    const sp = Math.sin(pitch), cp = Math.cos(pitch);
    const right = [cy, -sy, 0];
    const forward = [sy * cp, cy * cp, sp];
    const up = [-sy * sp, -cy * sp, cp];
    const delta = point.map((value, index) => value - view.viewer_position_m[index]);
    const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const cameraX = dot(delta, right);
    const cameraY = dot(delta, up);
    const cameraZ = dot(delta, forward);
    if (cameraZ <= 1e-6) return null;

    const tanHalfHorizontal = Math.tan(fov * DEG_TO_RAD / 2);
    const tanHalfVertical = tanHalfHorizontal * height / width;
    const ndcX = cameraX / (cameraZ * tanHalfHorizontal);
    const ndcY = cameraY / (cameraZ * tanHalfVertical);
    return {
      x: (ndcX + 1) * width / 2,
      y: (1 - ndcY) * height / 2,
      depth_m: cameraZ,
    };
  }

  function screenCovariance(estimate, view, width, height, torsoHeight) {
    const x = estimate.position_m[0];
    const y = estimate.position_m[1];
    const step = 0.001;
    const center = projectPoint([x, y, torsoHeight], view, width, height);
    if (!center) return null;
    const xStep = projectPoint([x + step, y, torsoHeight], view, width, height);
    const yStep = projectPoint([x, y + step, torsoHeight], view, width, height);
    if (!xStep || !yStep) return null;

    const j00 = (xStep.x - center.x) / step;
    const j10 = (xStep.y - center.y) / step;
    const j01 = (yStep.x - center.x) / step;
    const j11 = (yStep.y - center.y) / step;
    const covariance = estimate.covariance;
    const c00 = covariance[0][0], c01 = covariance[0][1];
    const c10 = covariance[1][0], c11 = covariance[1][1];
    return {
      xx: j00 * (c00 * j00 + c01 * j01) + j01 * (c10 * j00 + c11 * j01),
      xy: j00 * (c00 * j10 + c01 * j11) + j01 * (c10 * j10 + c11 * j11),
      yy: j10 * (c00 * j10 + c01 * j11) + j11 * (c10 * j10 + c11 * j11),
    };
  }

  function ellipse95(covariance) {
    if (!covariance) return null;
    const trace = covariance.xx + covariance.yy;
    const discriminant = Math.sqrt(
      Math.max(0, (covariance.xx - covariance.yy) ** 2 + 4 * covariance.xy ** 2)
    );
    const majorVariance = Math.max(0, (trace + discriminant) / 2);
    const minorVariance = Math.max(0, (trace - discriminant) / 2);
    return {
      major: Math.sqrt(majorVariance * CHI_SQUARE_95_2D),
      minor: Math.sqrt(minorVariance * CHI_SQUARE_95_2D),
      angle: 0.5 * Math.atan2(2 * covariance.xy, covariance.xx - covariance.yy),
    };
  }

  function projectEstimate(estimate, view, width, height) {
    const torsoHeight = estimate.height_m == null
      ? view.torso_height_m
      : estimate.height_m;
    const center = projectPoint(
      [estimate.position_m[0], estimate.position_m[1], torsoHeight],
      view,
      width,
      height
    );
    if (!center) return null;
    return {
      center,
      ellipse: ellipse95(
        screenCovariance(estimate, view, width, height, torsoHeight)
      ),
    };
  }

  function shouldRenderEstimate(estimate, nowMs, lastMessageMs, view) {
    return Boolean(
      estimate &&
      estimate.present >= view.presence_threshold &&
      nowMs - lastMessageMs <= view.stale_after_ms
    );
  }

  return {
    CHI_SQUARE_95_2D,
    ellipse95,
    projectEstimate,
    projectPoint,
    screenCovariance,
    shouldRenderEstimate,
  };
});
