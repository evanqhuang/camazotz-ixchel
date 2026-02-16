export class EnvironmentControls {
  constructor(caveEnv) {
    this.caveEnv = caveEnv;
    this._listeners = [];

    const toggles = [
      { id: 'toggle-cave', element: 'cave' },
      { id: 'toggle-grid', element: 'grid' },
    ];

    for (const { id, element } of toggles) {
      const checkbox = document.getElementById(id);
      if (checkbox) {
        const handler = () => {
          this.caveEnv.setVisible(element, checkbox.checked);
        };
        checkbox.addEventListener('change', handler);
        this._listeners.push({ checkbox, handler });
      }
    }
  }

  dispose() {
    for (const { checkbox, handler } of this._listeners) {
      checkbox.removeEventListener('change', handler);
    }
    this._listeners = [];
  }
}
