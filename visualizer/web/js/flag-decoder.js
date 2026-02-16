export const FLAG_NAMES = new Map([
  [0x01, 'ENCODER_ESTIMATED'],
  [0x02, 'IMU_ESTIMATED'],
  [0x04, 'DEPTH_VIRTUAL'],
  [0x08, 'DEPTH_UNVERIFIED'],
  [0x10, 'NAV_CRITICAL'],
  [0x20, 'ENCODER_LOST'],
  [0x40, 'IMU_LOST'],
  [0x80, 'SENSOR_CONFLICT']
]);

export function flagSeverity(flag) {
  if (flag === 0x00) {
    return 'ok';
  }

  if ((flag & 0xF0) !== 0) {
    return 'critical';
  }

  return 'warning';
}

export function flagColor(flag) {
  const severity = flagSeverity(flag);

  switch (severity) {
    case 'ok':
      return 0x00994d;
    case 'warning':
      return 0xcc8800;
    case 'critical':
      return 0xcc0000;
    default:
      return 0xffffff;
  }
}

export function decodeFlags(flag) {
  if (flag === 0x00) {
    return [];
  }

  const activeFlags = [];
  for (const [bit, name] of FLAG_NAMES.entries()) {
    if ((flag & bit) !== 0) {
      activeFlags.push(name);
    }
  }
  return activeFlags;
}
