import { convertValue, getUnit, getPrecision, cycleUnit } from './units.js';

export function computeStats(parsedData) {
  const { timestamps, positions, deltaDistances, flags, count } = parsedData;

  if (count === 0) {
    throw new Error('No data points to compute stats');
  }

  const durationMs = timestamps[count - 1] - timestamps[0];
  const duration = durationMs / 1000.0;

  let totalDistance = 0;
  for (let i = 0; i < count; i++) {
    totalDistance += deltaDistances[i];
  }

  const minPos = [Infinity, Infinity, Infinity];
  const maxPos = [-Infinity, -Infinity, -Infinity];

  let maxDepth = 0;

  for (let i = 0; i < count; i++) {
    const px = positions[i * 3 + 0];
    const py = positions[i * 3 + 1];
    const pz = positions[i * 3 + 2];

    const vizX = px;
    const vizY = pz;
    const vizZ = py;

    minPos[0] = Math.min(minPos[0], vizX);
    minPos[1] = Math.min(minPos[1], vizY);
    minPos[2] = Math.min(minPos[2], vizZ);

    maxPos[0] = Math.max(maxPos[0], vizX);
    maxPos[1] = Math.max(maxPos[1], vizY);
    maxPos[2] = Math.max(maxPos[2], vizZ);

    maxDepth = Math.max(maxDepth, Math.abs(pz));
  }

  // Compute depth percentiles for color normalization
  const depths = new Float32Array(count);
  for (let i = 0; i < count; i++) {
    depths[i] = Math.abs(positions[i * 3 + 2]);
  }
  depths.sort();
  const depthP5 = depths[Math.floor(count * 0.05)] ?? 0;
  const depthP95 = depths[Math.floor(count * 0.95)] ?? maxDepth;

  const avgSpeed = duration > 0 ? totalDistance / duration : 0;

  let flaggedCount = 0;
  for (let i = 0; i < count; i++) {
    if (flags[i] !== 0x00) {
      flaggedCount++;
    }
  }
  const flaggedPercent = count > 0 ? (flaggedCount / count) * 100 : 0;

  return {
    duration,
    totalDistance,
    maxDepth,
    depthP5,
    depthP95,
    avgSpeed,
    flaggedPercent,
    boundingBox: {
      min: minPos,
      max: maxPos
    },
    sampleCount: count
  };
}

export function formatStats(stats) {
  const {
    duration,
    totalDistance,
    maxDepth,
    avgSpeed,
    flaggedPercent,
    sampleCount
  } = stats;

  const formatDuration = (seconds) => {
    const totalSeconds = Math.floor(seconds);
    const mins = Math.floor(totalSeconds / 60);
    const secs = totalSeconds % 60;
    return `${mins}:${secs.toString().padStart(2, '0')}`;
  };

  const flaggedClass = flaggedPercent > 50 ? 'critical' : flaggedPercent > 10 ? 'warning' : '';

  const distanceValue = convertValue('distance', totalDistance);
  const distanceUnit = getUnit('distance');
  const distancePrecision = getPrecision('distance');

  const depthValue = convertValue('depth', maxDepth);
  const depthUnit = getUnit('depth');
  const depthPrecision = getPrecision('depth');

  const speedValue = convertValue('speed', avgSpeed);
  const speedUnit = getUnit('speed');
  const speedPrecision = getPrecision('speed');

  return `
    <div class="stat-row">
      <span class="stat-label">Duration:</span>
      <span class="stat-value">${formatDuration(duration)}</span>
    </div>
    <div class="stat-row">
      <span class="stat-label">Distance:</span>
      <span class="stat-value"><span class="stat-number" data-stat="distance">${distanceValue.toFixed(distancePrecision)}</span> <span class="stat-unit" data-stat="distance">${distanceUnit}</span></span>
    </div>
    <div class="stat-row">
      <span class="stat-label">Max Depth:</span>
      <span class="stat-value"><span class="stat-number" data-stat="depth">${depthValue.toFixed(depthPrecision)}</span> <span class="stat-unit" data-stat="depth">${depthUnit}</span></span>
    </div>
    <div class="stat-row">
      <span class="stat-label">Avg Speed:</span>
      <span class="stat-value"><span class="stat-number" data-stat="speed">${speedValue.toFixed(speedPrecision)}</span> <span class="stat-unit" data-stat="speed">${speedUnit}</span></span>
    </div>
    <div class="stat-row">
      <span class="stat-label">Flagged:</span>
      <span class="stat-value ${flaggedClass}">${flaggedPercent.toFixed(1)}%</span>
    </div>
    <div class="stat-row">
      <span class="stat-label">Samples:</span>
      <span class="stat-value">${sampleCount.toLocaleString()}</span>
    </div>
  `;
}

export function bindUnitToggles(stats, container) {
  const RAW_VALUES = {
    distance: stats.totalDistance,
    depth: stats.maxDepth,
    speed: stats.avgSpeed,
  };

  const unitElements = container.querySelectorAll('.stat-unit');

  unitElements.forEach(element => {
    element.addEventListener('click', () => {
      const statName = element.getAttribute('data-stat');
      if (!statName || !RAW_VALUES.hasOwnProperty(statName)) {
        return;
      }

      cycleUnit(statName);

      const rawValue = RAW_VALUES[statName];
      const convertedValue = convertValue(statName, rawValue);
      const precision = getPrecision(statName);
      const unit = getUnit(statName);

      const numberElement = container.querySelector(`.stat-number[data-stat="${statName}"]`);
      const unitElement = container.querySelector(`.stat-unit[data-stat="${statName}"]`);

      if (numberElement) {
        numberElement.textContent = convertedValue.toFixed(precision);
      }
      if (unitElement) {
        unitElement.textContent = unit;
      }
    });
  });
}
