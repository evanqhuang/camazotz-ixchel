export function parseNavCSV(text) {
  const lines = text.split(/\r?\n/).filter(line => line.trim().length > 0);

  if (lines.length < 2) {
    throw new Error('CSV file is empty or missing header');
  }

  const header = lines[0];
  const expectedHeader = 'timestamp_ms,seq,angular_delta,qw,qx,qy,qz,px,py,pz,delta_dist,flags';
  if (header !== expectedHeader) {
    throw new Error(`Invalid CSV header. Expected: ${expectedHeader}`);
  }

  const dataLines = lines.slice(1);
  const count = dataLines.length;

  const timestamps = new Float64Array(count);
  const sequences = new Uint32Array(count);
  const angularDeltas = new Float32Array(count);
  const quaternions = new Float32Array(count * 4);
  const positions = new Float32Array(count * 3);
  const deltaDistances = new Float32Array(count);
  const flags = new Uint8Array(count);

  for (let i = 0; i < count; i++) {
    const line = dataLines[i];
    const parts = line.split(',');

    if (parts.length !== 12) {
      throw new Error(`Invalid line ${i + 2}: expected 12 columns, got ${parts.length}`);
    }

    timestamps[i] = parseFloat(parts[0]);
    sequences[i] = parseInt(parts[1], 10);
    angularDeltas[i] = parseFloat(parts[2]);

    quaternions[i * 4 + 0] = parseFloat(parts[3]);
    quaternions[i * 4 + 1] = parseFloat(parts[4]);
    quaternions[i * 4 + 2] = parseFloat(parts[5]);
    quaternions[i * 4 + 3] = parseFloat(parts[6]);

    positions[i * 3 + 0] = parseFloat(parts[7]);
    positions[i * 3 + 1] = parseFloat(parts[8]);
    positions[i * 3 + 2] = parseFloat(parts[9]);

    deltaDistances[i] = parseFloat(parts[10]);

    const flagStr = parts[11].trim();
    if (!flagStr.startsWith('0x')) {
      throw new Error(`Invalid flag format at line ${i + 2}: ${flagStr}`);
    }
    flags[i] = parseInt(flagStr, 16);
  }

  return {
    timestamps,
    sequences,
    angularDeltas,
    quaternions,
    positions,
    deltaDistances,
    flags,
    count
  };
}
