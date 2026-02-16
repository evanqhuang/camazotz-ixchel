import { parseNavCSV } from './csv-parser.js';
import { computeStats, formatStats, bindUnitToggles } from './dive-stats.js';
import { loadUnitPreferences, setUnitSystem, getUnitSystem } from './units.js';
import { SceneBuilder } from './scene-builder.js';
import { Timeline } from './timeline.js';
import { CaveEnvironment } from './cave-environment.js';
import { StickMap } from './stick-map.js';
import { generateSampleDiveCSV } from './sample-data.js';

loadUnitPreferences();

// Prevent browser from handling file drops (opening files)
document.addEventListener('dragover', (e) => e.preventDefault());
document.addEventListener('drop', (e) => e.preventDefault());

const dropZone = document.getElementById('drop-zone');
const fileInput = document.getElementById('file-input');
const statsPanel = document.getElementById('stats-panel');
const statsContent = document.getElementById('stats-content');
const controlsPanel = document.getElementById('controls-panel');
const timelinePanel = document.getElementById('timeline-panel');
const colorModeSelect = document.getElementById('color-mode');
const canvas = document.getElementById('viewport');
const panelToggle = document.getElementById('panel-toggle');
const depthScaleToggle = document.getElementById('depth-scale-toggle');
const unitToggle = document.getElementById('unit-toggle');
unitToggle.checked = getUnitSystem() === 'imperial';

// Panel toggle for mobile - show/hide stats and controls
if (panelToggle) {
  panelToggle.addEventListener('click', () => {
    document.body.classList.toggle('panels-visible');
  });
}

let sceneBuilder = null;
let timeline = null;
let parsedData = null;
let caveEnv = null;
let timelineRafId = null;
let cachedStats = null;

function getDepthScale() {
  return depthScaleToggle.checked ? 3 : 1;
}

function loadVisualization(csvText) {
  try {
    parsedData = parseNavCSV(csvText);

    dropZone.classList.add('hidden');
    statsPanel.classList.remove('hidden');
    controlsPanel.classList.remove('hidden');
    timelinePanel.classList.remove('hidden');

    if (!sceneBuilder) {
      sceneBuilder = new SceneBuilder(canvas);
      sceneBuilder.animate();
    } else {
      if (timelineRafId) {
        cancelAnimationFrame(timelineRafId);
        timelineRafId = null;
      }
      timeline = null;
      if (caveEnv) {
        caveEnv.dispose();
        caveEnv = null;
      }
      sceneBuilder.dispose();
      sceneBuilder = new SceneBuilder(canvas);
      sceneBuilder.animate();
    }

    cachedStats = computeStats(parsedData);
    statsContent.innerHTML = formatStats(cachedStats);
    bindUnitToggles(cachedStats, statsContent);

    const depthScale = getDepthScale();
    sceneBuilder.buildPath(parsedData, colorModeSelect.value, cachedStats, depthScale);

    const pathMetrics = sceneBuilder.getPathMetrics();
    if (pathMetrics) {
      caveEnv = new CaveEnvironment(sceneBuilder.scene);
      caveEnv.build(pathMetrics, depthScale);
    }

    sceneBuilder.createEndpointMarkers(parsedData);

    const marker = sceneBuilder.createMarker();
    timeline = new Timeline(parsedData, marker, sceneBuilder);

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
}

function handleFile(file) {
  if (!file || !file.name.endsWith('.csv')) {
    alert('Please select a valid CSV file');
    return;
  }

  const reader = new FileReader();

  reader.onload = (e) => {
    loadVisualization(e.target.result);
  };

  reader.onerror = () => {
    alert('Error reading file');
  };

  reader.readAsText(file);
}

dropZone.addEventListener('dragenter', (e) => {
  e.preventDefault();
  e.stopPropagation();
  dropZone.classList.add('drag-over');
});

dropZone.addEventListener('dragover', (e) => {
  e.preventDefault();
  e.stopPropagation();
  e.dataTransfer.dropEffect = 'copy';
  dropZone.classList.add('drag-over');
});

dropZone.addEventListener('dragleave', (e) => {
  e.preventDefault();
  e.stopPropagation();
  if (!dropZone.contains(e.relatedTarget)) {
    dropZone.classList.remove('drag-over');
  }
});

dropZone.addEventListener('drop', (e) => {
  e.preventDefault();
  e.stopPropagation();
  dropZone.classList.remove('drag-over');

  const files = e.dataTransfer.files;
  if (files.length > 0) {
    handleFile(files[0]);
  } else if (e.dataTransfer.types.includes('codefiles') ||
             e.dataTransfer.types.includes('application/vnd.code.uri-list')) {
    alert('Please drag the file from Finder or File Explorer, not from your code editor.');
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

depthScaleToggle.addEventListener('change', () => {
  const scale = getDepthScale();

  if (!sceneBuilder || !parsedData) return;

  sceneBuilder.buildPath(parsedData, colorModeSelect.value, cachedStats, scale);

  if (caveEnv) {
    caveEnv.dispose();
    caveEnv = null;
  }
  const pathMetrics = sceneBuilder.getPathMetrics();
  if (pathMetrics) {
    caveEnv = new CaveEnvironment(sceneBuilder.scene);
    caveEnv.build(pathMetrics, scale);
  }

  sceneBuilder.createEndpointMarkers(parsedData);
});

unitToggle.addEventListener('change', () => {
  setUnitSystem(unitToggle.checked ? 'imperial' : 'metric');
  if (cachedStats) {
    statsContent.innerHTML = formatStats(cachedStats);
    bindUnitToggles(cachedStats, statsContent);
  }
});

document.getElementById('export-map-btn').addEventListener('click', () => {
  if (!parsedData) return;
  const btn = document.getElementById('export-map-btn');
  btn.disabled = true;
  btn.textContent = 'Exporting...';

  const stats = cachedStats;
  const stickMap = new StickMap(parsedData, stats, colorModeSelect.value);
  stickMap.export('cave-stick-map.png', () => {
    btn.disabled = false;
    btn.textContent = 'Export Stick Map';
  });
});

document.getElementById('sample-link').addEventListener('click', (e) => {
  e.preventDefault();
  loadVisualization(generateSampleDiveCSV());
});
