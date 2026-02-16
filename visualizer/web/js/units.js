const M_TO_FT = 3.28084;
const S_TO_MIN = 60;

const UNIT_CONFIGS = {
  depth: {
    units: ['m', 'ft'],
    converters: {
      m: (v) => v,
      ft: (v) => v * M_TO_FT
    }
  },
  distance: {
    units: ['m', 'ft'],
    converters: {
      m: (v) => v,
      ft: (v) => v * M_TO_FT
    }
  },
  speed: {
    units: ['m/min', 'ft/min'],
    converters: {
      'm/min': (v) => v * S_TO_MIN,
      'ft/min': (v) => v * S_TO_MIN * M_TO_FT
    }
  }
};

const unitState = {
  depth: 'm',
  distance: 'm',
  speed: 'm/min'
};

export function loadUnitPreferences() {
  try {
    const stored = localStorage.getItem('camazotz-unit-prefs');
    if (stored) {
      const prefs = JSON.parse(stored);
      for (const [key, value] of Object.entries(prefs)) {
        const config = UNIT_CONFIGS[key];
        if (config && config.units.includes(value)) {
          unitState[key] = value;
        }
      }
    }
  } catch (error) {
    console.warn('Failed to load unit preferences:', error);
  }
}

export function saveUnitPreferences() {
  try {
    localStorage.setItem('camazotz-unit-prefs', JSON.stringify(unitState));
  } catch (error) {
    console.warn('Failed to save unit preferences:', error);
  }
}

export function getUnit(stat) {
  return unitState[stat];
}

export function cycleUnit(stat) {
  const config = UNIT_CONFIGS[stat];
  if (!config) {
    return unitState[stat];
  }

  const currentIndex = config.units.indexOf(unitState[stat]);
  const nextIndex = (currentIndex + 1) % config.units.length;
  unitState[stat] = config.units[nextIndex];

  saveUnitPreferences();
  return unitState[stat];
}

export function convertValue(stat, rawValue) {
  const config = UNIT_CONFIGS[stat];
  if (!config) {
    return rawValue;
  }

  const currentUnit = unitState[stat];
  const converter = config.converters[currentUnit];

  return converter ? converter(rawValue) : rawValue;
}

export function setUnitSystem(system) {
  if (system === 'imperial') {
    unitState.depth = 'ft';
    unitState.distance = 'ft';
    unitState.speed = 'ft/min';
  } else {
    unitState.depth = 'm';
    unitState.distance = 'm';
    unitState.speed = 'm/min';
  }
  saveUnitPreferences();
}

export function getUnitSystem() {
  return unitState.depth === 'ft' ? 'imperial' : 'metric';
}

export function getPrecision(stat) {
  const unit = unitState[stat];

  if (stat === 'depth' || stat === 'distance') {
    return unit === 'm' ? 2 : 1;
  }

  if (stat === 'speed') {
    return 1;
  }

  return 2;
}
