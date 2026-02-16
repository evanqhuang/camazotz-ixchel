/**
 * Speed colormap: warm gold (slow) through vivid orange to bright red (fast).
 * Three-stop piecewise linear interpolation for perceptual smoothness.
 */

const SLOW = { r: 1.0, g: 0.85, b: 0.0 };   // #FFD900
const MID  = { r: 1.0, g: 0.4,  b: 0.0 };   // #FF6600
const FAST = { r: 0.9, g: 0.1,  b: 0.0 };   // #E61A00

export function getSpeedColor(t) {
  const c = Math.max(0, Math.min(1, t));
  let from, to, local;
  if (c < 0.5) {
    from = SLOW; to = MID; local = c * 2;
  } else {
    from = MID; to = FAST; local = (c - 0.5) * 2;
  }
  return {
    r: from.r + local * (to.r - from.r),
    g: from.g + local * (to.g - from.g),
    b: from.b + local * (to.b - from.b)
  };
}

export function getSpeedColorCSS(t) {
  const color = getSpeedColor(t);
  return `rgb(${Math.floor(color.r * 255)}, ${Math.floor(color.g * 255)}, ${Math.floor(color.b * 255)})`;
}

export function getSpeedCSSStops() {
  return [
    { position: 0,   color: getSpeedColorCSS(0) },
    { position: 0.5, color: getSpeedColorCSS(0.5) },
    { position: 1,   color: getSpeedColorCSS(1) }
  ];
}
