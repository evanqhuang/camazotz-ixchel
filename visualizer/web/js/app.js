import { parseNavCSV } from './csv-parser.js';
import { computeStats, formatStats, bindUnitToggles } from './dive-stats.js';
import { loadUnitPreferences } from './units.js';
import { SceneBuilder } from './scene-builder.js';
import { Timeline } from './timeline.js';
import { CaveEnvironment } from './cave-environment.js';
import { EnvironmentControls } from './environment-controls.js';

loadUnitPreferences();

const dropZone = document.getElementById('drop-zone');
const fileInput = document.getElementById('file-input');
const statsPanel = document.getElementById('stats-panel');
const statsContent = document.getElementById('stats-content');
const controlsPanel = document.getElementById('controls-panel');
const timelinePanel = document.getElementById('timeline-panel');
const colorModeSelect = document.getElementById('color-mode');
const canvas = document.getElementById('viewport');
const environmentPanel = document.getElementById('environment-panel');

let sceneBuilder = null;
let timeline = null;
let parsedData = null;
let caveEnv = null;
let envControls = null;
let timelineRafId = null;

function handleFile(file) {
  if (!file || !file.name.endsWith('.csv')) {
    alert('Please select a valid CSV file');
    return;
  }

  const reader = new FileReader();

  reader.onload = (e) => {
    try {
      const text = e.target.result;
      parsedData = parseNavCSV(text);

      dropZone.classList.add('hidden');
      statsPanel.classList.remove('hidden');
      controlsPanel.classList.remove('hidden');
      timelinePanel.classList.remove('hidden');

      if (!sceneBuilder) {
        sceneBuilder = new SceneBuilder(canvas);
        sceneBuilder.animate();
      } else {
        if (caveEnv) {
          caveEnv.dispose();
          caveEnv = null;
        }
        if (envControls) {
          envControls.dispose();
          envControls = null;
        }
        sceneBuilder.dispose();
        sceneBuilder = new SceneBuilder(canvas);
        sceneBuilder.animate();
      }

      sceneBuilder.buildPath(parsedData, colorModeSelect.value);

      const pathMetrics = sceneBuilder.getPathMetrics();
      if (sceneBuilder.curve && pathMetrics) {
        caveEnv = new CaveEnvironment(sceneBuilder.scene);
        caveEnv.build(sceneBuilder.curve, pathMetrics);
        envControls = new EnvironmentControls(caveEnv);
        environmentPanel.classList.remove('hidden');
      }

      const stats = computeStats(parsedData);
      statsContent.innerHTML = formatStats(stats);
      bindUnitToggles(stats, statsContent);

      const marker = sceneBuilder.createMarker();
      timeline = new Timeline(parsedData, marker, sceneBuilder);

      if (timelineRafId) {
        cancelAnimationFrame(timelineRafId);
      }

      timelineRafId = requestAnimationFrame(function animate(time) {
        if (timeline) {
          timeline.update(time);
        }
        timelineRafId = requestAnimationFrame(animate);
      });

    } catch (error) {
      console.error('Error parsing CSV:', error);
      alert(`Error parsing CSV file: ${error.message}`);
    }
  };

  reader.onerror = () => {
    alert('Error reading file');
  };

  reader.readAsText(file);
}

dropZone.addEventListener('dragover', (e) => {
  e.preventDefault();
  e.stopPropagation();
  dropZone.classList.add('drag-over');
});

dropZone.addEventListener('dragleave', (e) => {
  e.preventDefault();
  e.stopPropagation();
  dropZone.classList.remove('drag-over');
});

dropZone.addEventListener('drop', (e) => {
  e.preventDefault();
  e.stopPropagation();
  dropZone.classList.remove('drag-over');

  const files = e.dataTransfer.files;
  if (files.length > 0) {
    handleFile(files[0]);
  }
});

fileInput.addEventListener('change', (e) => {
  const files = e.target.files;
  if (files.length > 0) {
    handleFile(files[0]);
  }
});

colorModeSelect.addEventListener('change', (e) => {
  if (sceneBuilder && parsedData) {
    sceneBuilder.setColorMode(parsedData, e.target.value);
  }
});
