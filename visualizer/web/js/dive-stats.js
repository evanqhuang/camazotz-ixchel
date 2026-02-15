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
    const vizY = -pz;
    const vizZ = py;

    minPos[0] = Math.min(minPos[0], vizX);
    minPos[1] = Math.min(minPos[1], vizY);
    minPos[2] = Math.min(minPos[2], vizZ);

    maxPos[0] = Math.max(maxPos[0], vizX);
    maxPos[1] = Math.max(maxPos[1], vizY);
    maxPos[2] = Math.max(maxPos[2], vizZ);

    maxDepth = Math.max(maxDepth, Math.abs(pz));
  }

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
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins}:${secs.toString().padStart(2, '0')}`;
  };

  const flaggedClass = flaggedPercent > 50 ? 'critical' : flaggedPercent > 10 ? 'warning' : '';

  return `
    <div class="stat-row">
      <span class="stat-label">Duration:</span>
      <span class="stat-value">${formatDuration(duration)}</span>
    </div>
    <div class="stat-row">
      <span class="stat-label">Distance:</span>
      <span class="stat-value">${totalDistance.toFixed(2)} m</span>
    </div>
    <div class="stat-row">
      <span class="stat-label">Max Depth:</span>
      <span class="stat-value">${maxDepth.toFixed(2)} m</span>
    </div>
    <div class="stat-row">
      <span class="stat-label">Avg Speed:</span>
      <span class="stat-value">${avgSpeed.toFixed(3)} m/s</span>
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
