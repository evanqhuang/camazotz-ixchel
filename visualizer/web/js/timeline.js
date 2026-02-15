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
    this.currentIndex = parsedData.count - 1;
    this.playbackSpeed = 1.0;
    this.lastUpdateTime = 0;

    this.slider.max = parsedData.count - 1;
    this.slider.value = this.currentIndex;

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
  }

  formatTime(ms) {
    const seconds = Math.floor(ms / 1000);
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins}:${secs.toString().padStart(2, '0')}`;
  }
}
