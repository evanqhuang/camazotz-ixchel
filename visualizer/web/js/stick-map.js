import { convertValue, getUnit, getPrecision } from './units.js';
import { flagColor } from './flag-decoder.js';
import { getDepthColorCSS, getDepthCSSStops } from './depth-colormap.js';
import { getSpeedColorCSS, getSpeedCSSStops } from './speed-colormap.js';

/**
 * Stick map renderer for cave dive surveys.
 * Exports 2D plan-view visualizations as PNG with color-coded paths,
 * station markers, depth annotations, and statistics.
 */
export class StickMap {
  constructor(parsedData, stats, colorMode) {
    this.data = parsedData;
    this.stats = stats;
    this.colorMode = colorMode; // 'depth' | 'speed' | 'flags'

    // Canvas dimensions
    this.width = 3200;
    this.height = 2400;
    this.titleHeight = 120;
    this.footerHeight = 120;
    this.legendWidth = 400;
    this.mapWidth = this.width - this.legendWidth;
    this.mapHeight = this.height - this.titleHeight - this.footerHeight;

    // Prepare path data
    this._preparePathData();
    this._computeCumulativeDistances();
    this._computeSpeedData();
  }

  /**
   * Export the stick map as a PNG download.
   */
  export(filename = 'cave-stick-map.png', onComplete = null) {
    const canvas = document.createElement('canvas');
    canvas.width = this.width;
    canvas.height = this.height;
    const ctx = canvas.getContext('2d');

    this._renderBackground(ctx);
    this._renderTitleBar(ctx);
    this._renderMapGrid(ctx);
    this._renderPath(ctx);
    this._renderEntranceMarker(ctx);
    this._renderDepthAnnotations(ctx);
    this._renderScaleBar(ctx);
    this._renderNorthArrow(ctx);
    this._renderLegend(ctx);
    this._renderFooter(ctx);

    canvas.toBlob(blob => {
      if (!blob) {
        console.error('Failed to create PNG blob from canvas');
        if (onComplete) onComplete();
        return;
      }
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = filename;
      a.click();
      setTimeout(() => {
        URL.revokeObjectURL(url);
        if (onComplete) onComplete();
      }, 1000);
    }, 'image/png');
  }

  /**
   * Extract and simplify path coordinates from position data.
   */
  _preparePathData() {
    const positions = this.data.positions;
    const count = this.data.count;

    // Extract plan-view coordinates (px, py)
    const rawPoints = [];
    for (let i = 0; i < count; i++) {
      rawPoints.push({
        x: positions[i * 3],
        y: positions[i * 3 + 1],
        origIndex: i
      });
    }

    // Edge case: too few points
    if (count < 10) {
      this.pathPoints = rawPoints;
      this._computeExtents();
      return;
    }

    // Simplify using iterative RDP
    this.pathPoints = this._rdpSimplify(rawPoints);
    this._computeExtents();
  }

  /**
   * Iterative Ramer-Douglas-Peucker simplification.
   */
  _rdpSimplify(points) {
    const n = points.length;
    let xMin = Infinity, xMax = -Infinity, yMin = Infinity, yMax = -Infinity;
    for (let i = 0; i < n; i++) {
      if (points[i].x < xMin) xMin = points[i].x;
      if (points[i].x > xMax) xMax = points[i].x;
      if (points[i].y < yMin) yMin = points[i].y;
      if (points[i].y > yMax) yMax = points[i].y;
    }
    const extentX = xMax - xMin;
    const extentY = yMax - yMin;
    const epsilon = Math.max(extentX, extentY) / 2000;

    const keep = new Uint8Array(n);
    keep[0] = 1;
    keep[n - 1] = 1;

    const stack = [[0, n - 1]];

    while (stack.length > 0) {
      const [start, end] = stack.pop();
      if (end - start <= 1) continue;

      // Find point with maximum perpendicular distance
      let maxDist = 0;
      let maxIndex = start;

      const p1 = points[start];
      const p2 = points[end];
      const dx = p2.x - p1.x;
      const dy = p2.y - p1.y;
      const normSq = dx * dx + dy * dy;

      for (let i = start + 1; i < end; i++) {
        const p = points[i];
        let dist;

        if (normSq === 0) {
          const pdx = p.x - p1.x;
          const pdy = p.y - p1.y;
          dist = Math.sqrt(pdx * pdx + pdy * pdy);
        } else {
          const t = Math.max(0, Math.min(1, ((p.x - p1.x) * dx + (p.y - p1.y) * dy) / normSq));
          const projX = p1.x + t * dx;
          const projY = p1.y + t * dy;
          const pdx = p.x - projX;
          const pdy = p.y - projY;
          dist = Math.sqrt(pdx * pdx + pdy * pdy);
        }

        if (dist > maxDist) {
          maxDist = dist;
          maxIndex = i;
        }
      }

      if (maxDist > epsilon) {
        keep[maxIndex] = 1;
        stack.push([start, maxIndex]);
        stack.push([maxIndex, end]);
      }
    }

    const simplified = [];
    for (let i = 0; i < n; i++) {
      if (keep[i]) {
        simplified.push(points[i]);
      }
    }

    return simplified;
  }

  /**
   * Compute bounding box and transform parameters.
   */
  _computeExtents() {
    if (this.pathPoints.length === 0) {
      this.minX = 0;
      this.minY = 0;
      this.maxX = 1;
      this.maxY = 1;
      this.scale = 1;
      this.offsetX = 0;
      this.offsetY = 0;
      return;
    }

    this.minX = Infinity;
    this.maxX = -Infinity;
    this.minY = Infinity;
    this.maxY = -Infinity;
    for (const p of this.pathPoints) {
      if (p.x < this.minX) this.minX = p.x;
      if (p.x > this.maxX) this.maxX = p.x;
      if (p.y < this.minY) this.minY = p.y;
      if (p.y > this.maxY) this.maxY = p.y;
    }

    let extentX = this.maxX - this.minX;
    let extentY = this.maxY - this.minY;

    // Edge case: zero extent (straight line or single point)
    if (extentX < 1e-6) extentX = 1;
    if (extentY < 1e-6) extentY = 1;

    const plotWidth = this.mapWidth - 200;
    const plotHeight = this.mapHeight - 200;

    this.scale = Math.min(plotWidth / extentX, plotHeight / extentY) * 0.9;

    const centerDataX = (this.minX + this.maxX) / 2;
    const centerDataY = (this.minY + this.maxY) / 2;
    const centerCanvasX = this.legendWidth + this.mapWidth / 2;
    const centerCanvasY = this.titleHeight + this.mapHeight / 2;

    this.offsetX = centerCanvasX - centerDataX * this.scale;
    this.offsetY = centerCanvasY + centerDataY * this.scale; // Y-flip
  }

  /**
   * Transform data coordinates to canvas coordinates.
   */
  _toCanvas(x, y) {
    return {
      x: this.offsetX + x * this.scale,
      y: this.offsetY - y * this.scale // Y-flip
    };
  }

  /**
   * Compute cumulative distance array for station placement.
   */
  _computeCumulativeDistances() {
    const deltas = this.data.deltaDistances;
    const n = this.data.count;

    this.cumulativeDistances = new Float32Array(n);
    let sum = 0;
    for (let i = 0; i < n; i++) {
      this.cumulativeDistances[i] = sum;
      sum += deltas[i];
    }
  }

  /**
   * Compute speed for each data point.
   */
  _computeSpeedData() {
    const n = this.data.count;
    const times = this.data.timestamps;
    const deltas = this.data.deltaDistances;

    this.speeds = new Float32Array(n);

    for (let i = 1; i < n; i++) {
      const dt = (times[i] - times[i - 1]) / 1000; // milliseconds to seconds
      if (dt > 0) {
        this.speeds[i] = deltas[i] / dt;
      }
    }

    // Compute speed percentiles for color normalization (matches depth approach)
    const sorted = Float32Array.from(this.speeds).sort();
    this.speedP5 = sorted[Math.floor(n * 0.05)] ?? 0;
    this.speedP95 = sorted[Math.floor(n * 0.95)] ?? 0;
    this.speedRange = this.speedP95 - this.speedP5;
  }

  /**
   * Find data index at a given cumulative distance using binary search.
   */
  _findIndexAtDistance(targetDist) {
    const dists = this.cumulativeDistances;
    let left = 0;
    let right = dists.length - 1;

    while (left < right) {
      const mid = Math.floor((left + right) / 2);
      if (dists[mid] < targetDist) {
        left = mid + 1;
      } else {
        right = mid;
      }
    }

    return left;
  }



  /**
   * Get segment color based on color mode.
   */
  _getSegmentColor(index) {
    const positions = this.data.positions;
    const flags = this.data.flags;

    if (this.colorMode === 'depth') {
      const pz = positions[index * 3 + 2];
      const depth = Math.abs(pz);
      const range = this.stats.depthP95 - this.stats.depthP5;
      const t = range > 0 ? Math.max(0, Math.min(1, (depth - this.stats.depthP5) / range)) : 0;
      return getDepthColorCSS(t);
    } else if (this.colorMode === 'speed') {
      const speed = this.speeds[index];
      const t = this.speedRange > 0 ? Math.max(0, Math.min(1, (speed - this.speedP5) / this.speedRange)) : 0;
      return getSpeedColorCSS(t);
    } else if (this.colorMode === 'flags') {
      const flag = flags[index];
      const hex = flagColor(flag);
      const r = (hex >> 16) & 0xFF;
      const g = (hex >> 8) & 0xFF;
      const b = hex & 0xFF;
      return `rgb(${r}, ${g}, ${b})`;
    }

    return '#0066cc';
  }

  /**
   * Round to nice number for intervals.
   */
  _niceNumber(value) {
    if (value <= 0) return 1;
    const exponent = Math.floor(Math.log10(value));
    const fraction = value / Math.pow(10, exponent);
    let nice;
    if (fraction <= 1.5) nice = 1;
    else if (fraction <= 3.5) nice = 2;
    else if (fraction <= 7.5) nice = 5;
    else nice = 10;
    return nice * Math.pow(10, exponent);
  }

  /**
   * Format duration as M:SS or H:MM:SS.
   */
  _formatDuration(seconds) {
    const hours = Math.floor(seconds / 3600);
    const mins = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);

    if (hours > 0) {
      return `${hours}:${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
    } else {
      return `${mins}:${secs.toString().padStart(2, '0')}`;
    }
  }

  /**
   * Render white background.
   */
  _renderBackground(ctx) {
    ctx.save();
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, this.width, this.height);
    ctx.restore();
  }

  /**
   * Render title bar.
   */
  _renderTitleBar(ctx) {
    ctx.save();
    ctx.fillStyle = '#1a3a5c';
    ctx.fillRect(0, 0, this.width, this.titleHeight);

    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 48px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('Cave Dive Survey', this.width / 2, this.titleHeight / 2 - 10);

    ctx.fillStyle = 'rgba(255, 255, 255, 0.7)';
    ctx.font = '24px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    const subtitle = `Duration: ${this._formatDuration(this.stats.duration)} | Samples: ${this.stats.sampleCount.toLocaleString()}`;
    ctx.fillText(subtitle, this.width / 2, this.titleHeight / 2 + 30);
    ctx.restore();
  }

  /**
   * Render dotted grid at nice intervals.
   */
  _renderMapGrid(ctx) {
    ctx.save();
    const mapLeft = this.legendWidth;
    const mapTop = this.titleHeight;

    const targetSpacing = 80; // pixels
    const dataSpacing = targetSpacing / this.scale;
    const niceSpacing = this._niceNumber(dataSpacing);
    if (!isFinite(niceSpacing) || niceSpacing <= 0) { ctx.restore(); return; }

    const startX = Math.floor(this.minX / niceSpacing) * niceSpacing;
    const startY = Math.floor(this.minY / niceSpacing) * niceSpacing;

    ctx.strokeStyle = 'rgba(0, 102, 204, 0.08)';
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 4]);

    for (let x = startX; x <= this.maxX + niceSpacing; x += niceSpacing) {
      const canvasX = this.offsetX + x * this.scale;
      if (canvasX >= mapLeft && canvasX <= this.width) {
        ctx.beginPath();
        ctx.moveTo(canvasX, mapTop);
        ctx.lineTo(canvasX, mapTop + this.mapHeight);
        ctx.stroke();
      }
    }

    for (let y = startY; y <= this.maxY + niceSpacing; y += niceSpacing) {
      const canvasY = this.offsetY - y * this.scale;
      if (canvasY >= mapTop && canvasY <= mapTop + this.mapHeight) {
        ctx.beginPath();
        ctx.moveTo(mapLeft, canvasY);
        ctx.lineTo(this.width, canvasY);
        ctx.stroke();
      }
    }

    ctx.setLineDash([]);
    ctx.restore();
  }

  /**
   * Render color-coded path.
   */
  _renderPath(ctx) {
    ctx.save();
    if (this.pathPoints.length < 2) { ctx.restore(); return; }

    ctx.lineWidth = 10;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    for (let i = 1; i < this.pathPoints.length; i++) {
      const p1 = this.pathPoints[i - 1];
      const p2 = this.pathPoints[i];
      const c1 = this._toCanvas(p1.x, p1.y);
      const c2 = this._toCanvas(p2.x, p2.y);

      const dataIndex = p2.origIndex;
      ctx.strokeStyle = this._getSegmentColor(dataIndex);

      ctx.beginPath();
      ctx.moveTo(c1.x, c1.y);
      ctx.lineTo(c2.x, c2.y);
      ctx.stroke();
    }
    ctx.restore();
  }

  /**
   * Render entrance marker.
   */
  _renderEntranceMarker(ctx) {
    ctx.save();
    if (this.data.count === 0) { ctx.restore(); return; }

    const positions = this.data.positions;
    const x = positions[0];
    const y = positions[1];
    const c = this._toCanvas(x, y);

    ctx.fillStyle = '#0066cc';
    ctx.beginPath();
    ctx.moveTo(c.x, c.y - 20);
    ctx.lineTo(c.x - 15, c.y + 10);
    ctx.lineTo(c.x + 15, c.y + 10);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = '#1a3a5c';
    ctx.font = 'bold 24px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    ctx.fillText('ENTRY', c.x, c.y + 15);
    ctx.restore();
  }

  /**
   * Render depth annotations at extrema and regular intervals.
   */
  _renderDepthAnnotations(ctx) {
    ctx.save();
    const positions = this.data.positions;
    const n = this.data.count;
    if (n < 2) { ctx.restore(); return; }

    const annotations = [];
    const visited = new Uint8Array(n);

    // Find local extrema using windowed comparison
    const windowSize = 20;
    for (let i = windowSize; i < n - windowSize; i++) {
      const curr = Math.abs(positions[i * 3 + 2]);
      let windowMin = curr, windowMax = curr;
      for (let j = i - windowSize; j <= i + windowSize; j++) {
        if (j === i) continue;
        const d = Math.abs(positions[j * 3 + 2]);
        windowMin = Math.min(windowMin, d);
        windowMax = Math.max(windowMax, d);
      }
      const isMax = curr >= windowMax && (curr - windowMin) > 1.0;
      const isMin = curr <= windowMin && (windowMax - curr) > 1.0;
      if (isMax || isMin) {
        annotations.push({ index: i, depth: curr, type: isMax ? 'max' : 'min' });
        visited[i] = 1;
        // Skip ahead to avoid clustering annotations
        i += windowSize;
      }
    }

    // Gap-fill every ~50m
    const totalDist = this.stats.totalDistance;
    const fillInterval = 50;

    for (let dist = fillInterval; dist < totalDist; dist += fillInterval) {
      const index = this._findIndexAtDistance(dist);
      if (!visited[index]) {
        const depth = Math.abs(positions[index * 3 + 2]);
        annotations.push({ index, depth, type: 'fill' });
        visited[index] = 1;
      }
    }

    // Sort by index for alternating offsets
    annotations.sort((a, b) => a.index - b.index);

    ctx.strokeStyle = 'rgba(128, 128, 128, 0.5)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 18px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    const placedBoxes = [];

    annotations.forEach((ann, idx) => {
      ctx.save();
      const x = positions[ann.index * 3];
      const y = positions[ann.index * 3 + 1];
      const c = this._toCanvas(x, y);

      const offsetDir = (idx % 2 === 0) ? -1 : 1;
      const labelX = c.x + offsetDir * 100;
      const labelY = c.y - 40;

      // Measure label pill for collision detection
      const displayDepth = convertValue('depth', ann.depth);
      const unit = getUnit('depth');
      const precision = getPrecision('depth');
      const text = `${displayDepth.toFixed(precision)} ${unit}`;
      const metrics = ctx.measureText(text);
      const padding = 12;
      const pillWidth = metrics.width + padding * 2;
      const pillHeight = 30;

      const box = {
        left: labelX - pillWidth / 2 - 4,
        right: labelX + pillWidth / 2 + 4,
        top: labelY - pillHeight / 2 - 4,
        bottom: labelY + pillHeight / 2 + 4
      };
      const overlaps = placedBoxes.some(pb =>
        box.left < pb.right && box.right > pb.left &&
        box.top < pb.bottom && box.bottom > pb.top
      );
      if (overlaps) {
        ctx.restore();
        return;
      }
      placedBoxes.push(box);

      // Leader line (drawn only if label is not overlapping)
      ctx.beginPath();
      ctx.moveTo(c.x, c.y);
      ctx.lineTo(labelX, labelY);
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.strokeStyle = '#cccccc';
      ctx.lineWidth = 1;
      ctx.setLineDash([]);
      ctx.beginPath();
      this._roundRect(ctx, labelX - pillWidth / 2, labelY - pillHeight / 2, pillWidth, pillHeight, 15);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#1a3a5c';
      ctx.fillText(text, labelX, labelY);

      ctx.setLineDash([4, 4]);
      ctx.restore();
    });

    ctx.setLineDash([]);
    ctx.restore();
  }

  /**
   * Render scale bar in bottom-right of map area.
   */
  _renderScaleBar(ctx) {
    ctx.save();
    const targetPixels = 200;
    const dataLength = targetPixels / this.scale;
    const niceLength = this._niceNumber(dataLength);
    const barPixels = niceLength * this.scale;

    const x = this.width - 150;
    const y = this.titleHeight + this.mapHeight - 80;

    ctx.strokeStyle = '#1a3a5c';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(x - barPixels / 2, y);
    ctx.lineTo(x + barPixels / 2, y);
    ctx.stroke();

    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x - barPixels / 2, y - 8);
    ctx.lineTo(x - barPixels / 2, y + 8);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(x + barPixels / 2, y - 8);
    ctx.lineTo(x + barPixels / 2, y + 8);
    ctx.stroke();

    ctx.fillStyle = '#1a3a5c';
    ctx.font = 'bold 20px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';

    const displayLength = convertValue('distance', niceLength);
    const unit = getUnit('distance');
    const precision = getPrecision('distance');
    ctx.fillText(`${displayLength.toFixed(precision)} ${unit}`, x, y - 15);
    ctx.restore();
  }

  /**
   * Render north arrow in top-right of map area.
   */
  _renderNorthArrow(ctx) {
    ctx.save();
    const x = this.width - 100;
    const y = this.titleHeight + 100;

    ctx.fillStyle = '#1a3a5c';
    ctx.beginPath();
    ctx.moveTo(x, y - 40);
    ctx.lineTo(x - 15, y + 10);
    ctx.lineTo(x, y);
    ctx.lineTo(x + 15, y + 10);
    ctx.closePath();
    ctx.fill();

    ctx.font = 'bold 32px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    ctx.fillText('N', x, y + 15);
    ctx.restore();
  }

  /**
   * Render legend panel with color key and statistics.
   */
  _renderLegend(ctx) {
    ctx.save();
    const x = 20;
    let y = this.titleHeight + 40;

    ctx.fillStyle = '#1a3a5c';
    ctx.font = 'bold 36px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText('Legend', x, y);

    y += 60;

    // Color gradient or flag key
    if (this.colorMode === 'depth' || this.colorMode === 'speed') {
      this._renderGradientBar(ctx, x, y);
      y += 280;
    } else if (this.colorMode === 'flags') {
      this._renderFlagKey(ctx, x, y);
      y += 200;
    }

    // Symbols section
    y += 40;
    ctx.fillStyle = '#1a3a5c';
    ctx.font = 'bold 28px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.fillText('Symbols', x, y);
    y += 50;

    ctx.fillStyle = '#0066cc';
    ctx.beginPath();
    ctx.moveTo(x + 15, y - 10);
    ctx.lineTo(x, y + 10);
    ctx.lineTo(x + 30, y + 10);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = '#333333';
    ctx.font = '20px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.fillText('Entrance', x + 50, y);
    y += 60;

    // Dive statistics
    ctx.fillStyle = '#1a3a5c';
    ctx.font = 'bold 28px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.fillText('Dive Statistics', x, y);
    y += 50;

    ctx.fillStyle = '#333333';
    ctx.font = '20px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';

    const stats = [
      ['Distance', convertValue('distance', this.stats.totalDistance), getUnit('distance'), getPrecision('distance')],
      ['Max Depth', convertValue('depth', this.stats.maxDepth), getUnit('depth'), getPrecision('depth')],
      ['Duration', this._formatDuration(this.stats.duration), '', 0],
      ['Avg Speed', convertValue('speed', this.stats.avgSpeed), getUnit('speed'), getPrecision('speed')],
      ['Flagged', this.stats.flaggedPercent.toFixed(1), '%', 0],
      ['Samples', this.stats.sampleCount.toLocaleString(), '', 0]
    ];

    stats.forEach(([label, value, unit, precision]) => {
      const displayValue = typeof value === 'number' && precision > 0 ? value.toFixed(precision) : value;
      const text = unit ? `${label}: ${displayValue} ${unit}` : `${label}: ${displayValue}`;
      ctx.fillText(text, x, y);
      y += 35;
    });
    ctx.restore();
  }

  /**
   * Render vertical gradient bar for depth or speed mode.
   */
  _renderGradientBar(ctx, x, y) {
    ctx.save();
    const barWidth = 60;
    const barHeight = 200;

    const gradient = ctx.createLinearGradient(x, y, x, y + barHeight);

    if (this.colorMode === 'depth') {
      getDepthCSSStops().forEach(s => gradient.addColorStop(s.position, s.color));
    } else if (this.colorMode === 'speed') {
      getSpeedCSSStops().forEach(s => gradient.addColorStop(s.position, s.color));
    }

    ctx.fillStyle = gradient;
    ctx.fillRect(x, y, barWidth, barHeight);

    ctx.strokeStyle = '#cccccc';
    ctx.lineWidth = 1;
    ctx.strokeRect(x, y, barWidth, barHeight);

    ctx.fillStyle = '#333333';
    ctx.font = '18px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';

    if (this.colorMode === 'depth') {
      const minDepth = this.stats.depthP5;
      const maxDepth = this.stats.depthP95;
      const minDisplay = convertValue('depth', minDepth);
      const maxDisplay = convertValue('depth', maxDepth);
      const unit = getUnit('depth');
      const precision = getPrecision('depth');

      ctx.fillText(`${minDisplay.toFixed(precision)} ${unit}`, x + barWidth + 10, y);
      ctx.fillText(`${maxDisplay.toFixed(precision)} ${unit}`, x + barWidth + 10, y + barHeight);
    } else if (this.colorMode === 'speed') {
      const minDisplay = convertValue('speed', this.speedP5);
      const maxDisplay = convertValue('speed', this.speedP95);
      const unit = getUnit('speed');
      const precision = getPrecision('speed');

      ctx.fillText(`${minDisplay.toFixed(precision)} ${unit}`, x + barWidth + 10, y);
      ctx.fillText(`${maxDisplay.toFixed(precision)} ${unit}`, x + barWidth + 10, y + barHeight);
    }
    ctx.restore();
  }

  /**
   * Render flag severity key.
   */
  _renderFlagKey(ctx, x, y) {
    ctx.save();
    const items = [
      { label: 'OK', color: '#00994d' },
      { label: 'Warning', color: '#cc8800' },
      { label: 'Critical', color: '#cc0000' }
    ];

    ctx.font = '20px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';

    items.forEach((item, idx) => {
      const itemY = y + idx * 50;

      ctx.fillStyle = item.color;
      ctx.fillRect(x, itemY - 12, 30, 24);

      ctx.fillStyle = '#333333';
      ctx.fillText(item.label, x + 45, itemY);
    });
    ctx.restore();
  }

  /**
   * Helper for rounded rectangle (fallback for older browsers).
   */
  _roundRect(ctx, x, y, w, h, r) {
    if (typeof ctx.roundRect === 'function') {
      ctx.roundRect(x, y, w, h, r);
      return;
    }
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + w - r, y);
    ctx.arcTo(x + w, y, x + w, y + r, r);
    ctx.lineTo(x + w, y + h - r);
    ctx.arcTo(x + w, y + h, x + w - r, y + h, r);
    ctx.lineTo(x + r, y + h);
    ctx.arcTo(x, y + h, x, y + h - r, r);
    ctx.lineTo(x, y + r);
    ctx.arcTo(x, y, x + r, y, r);
    ctx.closePath();
  }

  /**
   * Render footer bar.
   */
  _renderFooter(ctx) {
    ctx.save();
    const y = this.height - this.footerHeight;

    ctx.fillStyle = '#f0f0f0';
    ctx.fillRect(0, y, this.width, this.footerHeight);

    ctx.fillStyle = '#666666';
    ctx.font = '24px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('Camazotz Ixchel Dive Visualizer', this.width / 2, y + this.footerHeight / 2);
    ctx.restore();
  }
}
