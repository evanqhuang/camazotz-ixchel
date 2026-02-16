import { convertValue, getUnit, getPrecision } from './units.js';
import { decodeFlags, flagSeverity } from './flag-decoder.js';

export class Timeline {
  constructor(parsedData, marker, sceneBuilder) {
    this.parsedData = parsedData;
    this.marker = marker;
    this.sceneBuilder = sceneBuilder;

    this.playBtn = document.getElementById('play-btn');
    this.slider = document.getElementById('timeline-slider');
    this.timeDisplay = document.getElementById('time-display');
    this.speedSelect = document.getElementById('speed-select');

    this.playing = false;
    this.currentIndex = 0;
    this.playbackSpeed = 1.0;
    this.lastUpdateTime = 0;

    this.slider.max = parsedData.count - 1;
    this.slider.value = this.currentIndex;

    // Precompute cumulative distances for O(1) lookup
    this.cumulativeDistances = new Float32Array(parsedData.count);
    let runningTotal = 0;
    for (let i = 0; i < parsedData.count; i++) {
      runningTotal += parsedData.deltaDistances[i];
      this.cumulativeDistances[i] = runningTotal;
    }

    // Cache popup DOM element references
    this.popup = document.getElementById('timeline-popup');
    this.popupTime = document.getElementById('popup-time');
    this.popupDepth = document.getElementById('popup-depth');
    this.popupDepthUnit = document.getElementById('popup-depth-unit');
    this.popupDistance = document.getElementById('popup-distance');
    this.popupDistanceUnit = document.getElementById('popup-distance-unit');
    this.popupSpeed = document.getElementById('popup-speed');
    this.popupSpeedUnit = document.getElementById('popup-speed-unit');
    this.popupFlagsRow = document.getElementById('popup-flags-row');
    this.popupFlags = document.getElementById('popup-flags');

    this.playBtn.addEventListener('click', () => this.toggle());
    this.slider.addEventListener('input', (e) => {
      this.seekTo(parseInt(e.target.value, 10));
    });
    this.speedSelect.addEventListener('change', (e) => {
      this.playbackSpeed = parseFloat(e.target.value);
    });

    this.updateDisplay();
  }

  play() {
    if (this.currentIndex >= this.parsedData.count - 1) {
      this.currentIndex = 0;
    }
    this.playing = true;
    this.playBtn.textContent = '❚❚';
    this.lastUpdateTime = performance.now();
  }

  pause() {
    this.playing = false;
    this.playBtn.textContent = '▶';
  }

  toggle() {
    if (this.playing) {
      this.pause();
    } else {
      this.play();
    }
  }

  update(currentTime) {
    if (!this.playing) {
      return;
    }

    if (this.lastUpdateTime === 0) {
      this.lastUpdateTime = currentTime;
      return;
    }

    const deltaTime = (currentTime - this.lastUpdateTime) / 1000.0;
    this.lastUpdateTime = currentTime;

    if (this.currentIndex >= this.parsedData.count - 1) {
      this.pause();
      return;
    }

    const idx = Math.floor(this.currentIndex);
    const nextIdx = Math.min(idx + 1, this.parsedData.count - 1);
    const timeDelta = this.parsedData.timestamps[nextIdx] -
                      this.parsedData.timestamps[idx];

    const indexDelta = timeDelta > 0 ? (deltaTime * 1000.0 * this.playbackSpeed) / timeDelta : 1;

    this.currentIndex = Math.min(
      this.currentIndex + indexDelta,
      this.parsedData.count - 1
    );

    this.updateDisplay();
  }

  seekTo(index) {
    this.currentIndex = Math.max(0, Math.min(index, this.parsedData.count - 1));
    this.updateDisplay();
  }

  updateDisplay() {
    const idx = Math.floor(this.currentIndex);
    const position = this.sceneBuilder.getPointAtIndex(idx);
    this.marker.position.copy(position);

    this.slider.value = idx;

    const baseTimeMs = this.parsedData.timestamps[0];
    const currentTimeMs = this.parsedData.timestamps[idx] - baseTimeMs;
    const totalTimeMs = this.parsedData.timestamps[this.parsedData.count - 1] - baseTimeMs;

    this.timeDisplay.textContent = `${this.formatTime(currentTimeMs)} / ${this.formatTime(totalTimeMs)}`;

    // Update popup with current stats
    this.updatePopup(idx);
  }

  computeSpeed(idx) {
    // Use a window of points to smooth speed calculation
    const windowSize = 5;
    const startIdx = Math.max(0, idx - windowSize);
    const endIdx = idx;

    if (startIdx === endIdx) {
      return 0;
    }

    let distance = 0;
    for (let i = startIdx + 1; i <= endIdx; i++) {
      distance += this.parsedData.deltaDistances[i];
    }

    const timeMs = this.parsedData.timestamps[endIdx] - this.parsedData.timestamps[startIdx];
    const timeSec = timeMs / 1000;

    return timeSec > 0 ? distance / timeSec : 0;
  }

  updatePopup(idx) {
    const baseTimeMs = this.parsedData.timestamps[0];
    const currentTimeMs = this.parsedData.timestamps[idx] - baseTimeMs;

    // Time elapsed
    this.popupTime.textContent = this.formatTime(currentTimeMs);

    // Current depth (pz is depth in the coordinate system)
    const depth = Math.abs(this.parsedData.positions[idx * 3 + 2]);
    const depthValue = convertValue('depth', depth);
    const depthPrecision = getPrecision('depth');
    this.popupDepth.textContent = depthValue.toFixed(depthPrecision);
    this.popupDepthUnit.textContent = getUnit('depth');

    // Cumulative distance
    const cumDistance = this.cumulativeDistances[idx];
    const distValue = convertValue('distance', cumDistance);
    const distPrecision = getPrecision('distance');
    this.popupDistance.textContent = distValue.toFixed(distPrecision);
    this.popupDistanceUnit.textContent = getUnit('distance');

    // Current speed
    const speed = this.computeSpeed(idx);
    const speedValue = convertValue('speed', speed);
    const speedPrecision = getPrecision('speed');
    this.popupSpeed.textContent = speedValue.toFixed(speedPrecision);
    this.popupSpeedUnit.textContent = getUnit('speed');

    // Flags
    const flag = this.parsedData.flags[idx];
    if (flag !== 0x00) {
      const flagNames = decodeFlags(flag);
      const severity = flagSeverity(flag);
      this.popupFlagsRow.classList.remove('hidden');
      this.popupFlagsRow.classList.toggle('critical', severity === 'critical');
      this.popupFlags.textContent = flagNames.join(', ');
    } else {
      this.popupFlagsRow.classList.add('hidden');
    }

    // Show popup
    this.popup.classList.remove('hidden');
  }

  formatTime(ms) {
    const totalSeconds = Math.floor(ms / 1000);
    const mins = Math.floor(totalSeconds / 60);
    const secs = totalSeconds % 60;
    return `${mins}:${secs.toString().padStart(2, '0')}`;
  }
}
