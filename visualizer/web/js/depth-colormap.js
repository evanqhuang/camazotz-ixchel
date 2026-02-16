/**
 * Depth colormap: light blue (shallow) to dark purple (deep).
 */

const SHALLOW = { r: 0.6, g: 0.85, b: 1.0 };   // #99D8FF
const DEEP    = { r: 0.25, g: 0.0,  b: 0.5 };   // #3F007F

export function getDepthColor(t) {
  const c = Math.max(0, Math.min(1, t));
  return {
    r: SHALLOW.r + c * (DEEP.r - SHALLOW.r),
    g: SHALLOW.g + c * (DEEP.g - SHALLOW.g),
    b: SHALLOW.b + c * (DEEP.b - SHALLOW.b)
  };
}

export function getDepthColorCSS(t) {
  const color = getDepthColor(t);
  return `rgb(${Math.floor(color.r * 255)}, ${Math.floor(color.g * 255)}, ${Math.floor(color.b * 255)})`;
}

export function getDepthCSSStops() {
  return [
    { position: 0, color: getDepthColorCSS(0) },
    { position: 1, color: getDepthColorCSS(1) }
  ];
}
