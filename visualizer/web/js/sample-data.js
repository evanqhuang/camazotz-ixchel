/**
 * Cave dive sample data generator
 * Produces unique randomized cave dives on every call
 */

/**
 * Generates a random float between min and max
 */
function randomFloat(min, max) {
  return min + Math.random() * (max - min);
}

/**
 * Generates a random integer between min and max (inclusive)
 */
function randomInt(min, max) {
  return Math.floor(randomFloat(min, max + 1));
}

/**
 * Cosine interpolation for smooth transitions
 */
function cosineInterp(a, b, t) {
  const mu = (1 - Math.cos(t * Math.PI)) / 2;
  return a * (1 - mu) + b * mu;
}

/**
 * Multi-octave noise for organic cave meandering
 * Returns value in range [-1, 1]
 */
function smoothNoise(t, phases) {
  let sum = 0;
  let amplitude = 1.0;
  let totalAmplitude = 0;

  for (let i = 0; i < phases.length; i++) {
    const freq = Math.pow(2, i) * 0.1;
    sum += Math.sin(t * freq + phases[i]) * amplitude;
    totalAmplitude += amplitude;
    amplitude *= 0.5;
  }

  return sum / totalAmplitude;
}

/**
 * Generate random cave segments with dramatic depth profiles
 */
function generateSegments(count, entryHeading) {
  // Zone types with characteristic depth ranges
  const ZONE_TYPES = {
    ENTRY_DESCENT: { depth: [-8, -3], duration: 1 },
    SHALLOW_PASSAGE: { depth: [-3, -1], duration: [2, 4] },
    GRADUAL_DESCENT: { depth: [-15, -8], duration: [1, 2] },
    DEEP_PIT: { depth: [-35, -25], duration: [1, 2] },
    ASCENT: { depth: [-5, -2], duration: [1, 2] }
  };

  // Generate depth zones for the cave profile
  const zones = [];
  let remainingSegments = count;

  // Always start with entry descent
  zones.push({
    type: 'ENTRY_DESCENT',
    targetDepth: -randomFloat(3, 8),
    segmentCount: 1
  });
  remainingSegments -= 1;

  // Guarantee at least one deep pit and one long shallow passage
  let hasDeepPit = false;
  let hasLongShallow = false;

  while (remainingSegments > 0) {
    let zoneType;
    let duration;

    // Force a deep pit if we haven't had one and we're past halfway
    if (!hasDeepPit && remainingSegments <= count / 2) {
      zoneType = ZONE_TYPES.DEEP_PIT;
      duration = randomInt(zoneType.duration[0], zoneType.duration[1]);
      hasDeepPit = true;
    }
    // Force a long shallow passage if we haven't had one
    else if (!hasLongShallow && remainingSegments >= 3) {
      zoneType = ZONE_TYPES.SHALLOW_PASSAGE;
      duration = Math.min(remainingSegments, randomInt(3, 4)); // Guarantee 3+ segments
      hasLongShallow = true;
    }
    // Otherwise pick randomly based on current depth and variety
    else {
      const lastDepth = zones[zones.length - 1].targetDepth;
      const weights = {
        SHALLOW_PASSAGE: lastDepth < -10 ? 0.3 : 0.2,
        GRADUAL_DESCENT: lastDepth > -10 ? 0.3 : 0.15,
        DEEP_PIT: !hasDeepPit && lastDepth > -20 ? 0.3 : 0.05,
        ASCENT: lastDepth < -15 ? 0.2 : 0.05
      };

      const rand = Math.random();
      let cumulative = 0;
      let selectedType = 'SHALLOW_PASSAGE';

      for (const [type, weight] of Object.entries(weights)) {
        cumulative += weight;
        if (rand < cumulative) {
          selectedType = type;
          break;
        }
      }

      zoneType = ZONE_TYPES[selectedType];
      const durRange = Array.isArray(zoneType.duration) ? zoneType.duration : [zoneType.duration, zoneType.duration];
      duration = Math.min(remainingSegments, randomInt(durRange[0], durRange[1]));
    }

    zones.push({
      type: Object.keys(ZONE_TYPES).find(k => ZONE_TYPES[k] === zoneType),
      targetDepth: -randomFloat(Math.abs(zoneType.depth[0]), Math.abs(zoneType.depth[1])),
      segmentCount: duration
    });

    remainingSegments -= duration;
  }

  // Generate segments following the zone profile
  const segments = [];
  let currentDepth = 0;
  let currentHeading = entryHeading;
  let biasLeft = Math.random() > 0.5;
  let zoneIdx = 0;
  let segmentInZone = 0;

  for (let i = 0; i < count; i++) {
    // Move to next zone if we've finished the current one
    if (segmentInZone >= zones[zoneIdx].segmentCount) {
      zoneIdx++;
      segmentInZone = 0;
    }

    const currentZone = zones[zoneIdx];
    const nextZone = zones[zoneIdx + 1];

    // Interpolate depth target within current zone
    let targetDepth = currentZone.targetDepth;
    if (nextZone && segmentInZone === currentZone.segmentCount - 1) {
      // Last segment in zone transitions toward next zone
      targetDepth = (currentZone.targetDepth + nextZone.targetDepth) / 2;
    }

    // Add some variation to target depth
    const depthVariation = randomFloat(-1, 1);
    const newDepth = Math.max(-40, Math.min(0, targetDepth + depthVariation));

    // Heading changes - alternate bias for natural meandering
    const headingDeltaDeg = randomFloat(15, 90) * (biasLeft ? -1 : 1);
    const headingDelta = headingDeltaDeg * Math.PI / 180;
    currentHeading += headingDelta;
    biasLeft = !biasLeft;

    segments.push({
      length: randomFloat(30, 80),
      startDepth: currentDepth,
      endDepth: newDepth,
      heading: currentHeading,
      swimSpeed: randomFloat(0.2, 0.6)
    });

    currentDepth = newDepth;
    segmentInZone++;
  }

  return segments;
}

/**
 * Generate random flag events
 */
function generateFlagEvents(totalSamples) {
  if (totalSamples < 700) return [];

  const events = [];
  const eventCount = randomInt(2, 4);
  const availableFlags = [0x01, 0x04, 0x10, 0x20];

  // Ensure at least one 0x01 and one 0x10/0x20
  const guaranteedFlags = [
    0x01,
    Math.random() > 0.5 ? 0x10 : 0x20
  ];

  for (let i = 0; i < eventCount; i++) {
    const startSample = randomInt(100, totalSamples - 600);
    const duration = randomInt(10, 60);
    const durationSamples = duration * 10; // 10Hz
    const flag = i < guaranteedFlags.length ?
      guaranteedFlags[i] :
      availableFlags[randomInt(0, availableFlags.length - 1)];

    events.push({
      start: startSample,
      end: startSample + durationSamples,
      flag: flag
    });
  }

  return events;
}

/**
 * Get flag value for a given sample index
 */
function getFlagForSample(sampleIdx, events) {
  for (const event of events) {
    if (sampleIdx >= event.start && sampleIdx < event.end) {
      return event.flag;
    }
  }
  return 0x00;
}

/**
 * Generate a complete random cave dive CSV
 */
export function generateSampleDiveCSV() {
  // Random dive parameters
  const durationMinutes = randomFloat(20, 40);
  const segmentCount = randomInt(8, 14);
  const entryHeading = randomFloat(0, Math.PI * 2);

  // Generate cave segments
  const segments = generateSegments(segmentCount, entryHeading);

  // Scale segment lengths so overall speed is realistic (6-12 m/min)
  const targetSpeedMPerMin = randomFloat(6, 12);
  const targetDistance = targetSpeedMPerMin * durationMinutes;
  const rawTotalDistance = segments.reduce((sum, seg) => sum + seg.length, 0);
  const scaleFactor = targetDistance / rawTotalDistance;
  segments.forEach(seg => {
    seg.length *= scaleFactor;
  });

  const totalDistance = targetDistance;
  const totalTime = durationMinutes * 60;
  const avgSpeed = totalDistance / totalTime;

  // Adjust per-segment speeds with slight variation
  segments.forEach(seg => {
    seg.swimSpeed = avgSpeed * randomFloat(0.8, 1.2);
  });

  // Generate samples at 10Hz
  const sampleRate = 10; // Hz
  const dt = 1.0 / sampleRate;
  const totalSamples = Math.floor(totalTime * sampleRate);

  // Generate flag events
  const flagEvents = generateFlagEvents(totalSamples);

  // Generate noise phases for smooth meandering
  const noisePhases = Array(4).fill(0).map(() => randomFloat(0, Math.PI * 2));

  // Build CSV
  const header = 'timestamp_ms,seq,angular_delta,qw,qx,qy,qz,px,py,pz,delta_dist,flags';
  const rows = [header];

  let currentSegmentIdx = 0;
  let distanceInSegment = 0;
  let px = 0;
  let py = 0;
  let pz = 0;
  let heading = entryHeading;
  let prevHeading = heading;

  // Emit starting position at origin
  rows.push([0, 0, '0.000000', Math.cos(entryHeading / 2).toFixed(6), '0.000000', '0.000000', Math.sin(entryHeading / 2).toFixed(6), '0.000000', '0.000000', '0.000000', '0.000000', '0x00'].join(','));

  for (let i = 0; i < totalSamples; i++) {
    const timestamp = (i + 1) * 100; // 100ms intervals

    // Find current segment
    while (currentSegmentIdx < segments.length - 1 &&
           distanceInSegment >= segments[currentSegmentIdx].length) {
      distanceInSegment -= segments[currentSegmentIdx].length;
      currentSegmentIdx++;
    }

    const segment = segments[currentSegmentIdx];
    const segmentProgress = distanceInSegment / segment.length;

    // Interpolate depth with cosine
    const depth = cosineInterp(segment.startDepth, segment.endDepth, segmentProgress);

    // Add noise to heading for organic meandering
    const noise = smoothNoise(i * dt, noisePhases);
    const headingNoise = noise * 0.15; // ±0.15 radians max deviation
    heading = segment.heading + headingNoise;

    // Calculate position
    const prevPx = px;
    const prevPy = py;
    const prevPz = pz;

    const stepDistance = segment.swimSpeed * dt;
    // Small lateral drift proportional to step size for realism
    const driftX = stepDistance * randomFloat(-0.05, 0.05);
    const driftY = stepDistance * randomFloat(-0.05, 0.05);
    px += Math.cos(heading) * stepDistance + driftX;
    py += Math.sin(heading) * stepDistance + driftY;
    pz = depth;

    // Calculate deltas
    const deltaX = px - prevPx;
    const deltaY = py - prevPy;
    const deltaZ = pz - prevPz;
    const deltaDist = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

    let angularDelta = heading - prevHeading;
    // Normalize to [-π, π]
    while (angularDelta > Math.PI) angularDelta -= 2 * Math.PI;
    while (angularDelta < -Math.PI) angularDelta += 2 * Math.PI;

    // Yaw-only quaternion
    const halfHeading = heading / 2;
    const qw = Math.cos(halfHeading);
    const qx = 0;
    const qy = 0;
    const qz = Math.sin(halfHeading);

    // Get flag
    const flag = getFlagForSample(i, flagEvents);
    const flagStr = `0x${flag.toString(16).padStart(2, '0')}`;

    // Build row
    const row = [
      timestamp,
      i + 1,
      angularDelta.toFixed(6),
      qw.toFixed(6),
      qx.toFixed(6),
      qy.toFixed(6),
      qz.toFixed(6),
      px.toFixed(6),
      py.toFixed(6),
      pz.toFixed(6),
      deltaDist.toFixed(6),
      flagStr
    ].join(',');

    rows.push(row);

    // Update for next iteration
    distanceInSegment += stepDistance;
    prevHeading = heading;
  }

  return rows.join('\n');
}
