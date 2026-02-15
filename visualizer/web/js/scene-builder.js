import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { flagColor } from './flag-decoder.js';

export class SceneBuilder {
  constructor(canvas) {
    this.canvas = canvas;

    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      alpha: false
    });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0x0a0e14);

    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.Fog(0x0a0e14, 100, 300);

    this.camera = new THREE.PerspectiveCamera(
      60,
      window.innerWidth / window.innerHeight,
      0.01,
      1000
    );
    this.camera.position.set(5, 5, 5);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;
    this.controls.enablePan = true;
    this.controls.enableZoom = true;

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 10, 10);
    this.scene.add(directionalLight);

    const axesHelper = new THREE.AxesHelper(0.5);
    this.scene.add(axesHelper);

    this.tubeMesh = null;
    this.lineMesh = null;
    this.marker = null;
    this.parsedData = null;
    this.tubeParams = null;

    this.handleResize();
    window.addEventListener('resize', () => this.handleResize());
  }

  handleResize() {
    const width = window.innerWidth;
    const height = window.innerHeight;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  buildPath(parsedData, colorMode = 'depth') {
    this.parsedData = parsedData;

    if (this.tubeMesh) {
      this.scene.remove(this.tubeMesh);
      this.tubeMesh.geometry.dispose();
      this.tubeMesh.material.dispose();
    }

    if (this.lineMesh) {
      this.scene.remove(this.lineMesh);
      this.lineMesh.geometry.dispose();
      this.lineMesh.material.dispose();
    }

    const { positions, count } = parsedData;

    const vizPositions = new Float32Array(count * 3);
    for (let i = 0; i < count; i++) {
      const px = positions[i * 3 + 0];
      const py = positions[i * 3 + 1];
      const pz = positions[i * 3 + 2];

      vizPositions[i * 3 + 0] = px;
      vizPositions[i * 3 + 1] = -pz;
      vizPositions[i * 3 + 2] = py;
    }

    const downsampleRate = 10;
    const downsampledPoints = [];

    for (let i = 0; i < count; i += downsampleRate) {
      downsampledPoints.push(new THREE.Vector3(
        vizPositions[i * 3 + 0],
        vizPositions[i * 3 + 1],
        vizPositions[i * 3 + 2]
      ));
    }

    const radialSegments = 8;
    const tubularSegments = downsampledPoints.length * 2;

    this.tubeParams = { tubularSegments, radialSegments };

    if (downsampledPoints.length > 1) {
      const curve = new THREE.CatmullRomCurve3(downsampledPoints);
      const tubeGeometry = new THREE.TubeGeometry(curve, tubularSegments, 0.15, radialSegments, false);

      this.applyVertexColors(tubeGeometry, parsedData, colorMode);

      const tubeMaterial = new THREE.MeshStandardMaterial({
        vertexColors: true,
        roughness: 0.4,
        metalness: 0.2
      });

      this.tubeMesh = new THREE.Mesh(tubeGeometry, tubeMaterial);
      this.scene.add(this.tubeMesh);
    }

    const lineGeometry = new THREE.BufferGeometry();
    lineGeometry.setAttribute('position', new THREE.BufferAttribute(vizPositions, 3));

    const lineMaterial = new THREE.LineBasicMaterial({
      color: 0x00d4ff,
      transparent: true,
      opacity: 0.3,
      linewidth: 1
    });

    this.lineMesh = new THREE.Line(lineGeometry, lineMaterial);
    this.scene.add(this.lineMesh);

    this.fitCameraToPath(vizPositions, count);
  }

  applyVertexColors(tubeGeometry, parsedData, colorMode) {
    const { positions, deltaDistances, flags, count } = parsedData;
    const posAttr = tubeGeometry.getAttribute('position');
    const vertexCount = posAttr.count;

    // Precompute depth/speed ranges from source data
    let maxDepth = 0;
    for (let i = 0; i < count; i++) {
      maxDepth = Math.max(maxDepth, Math.abs(positions[i * 3 + 2]));
    }

    let maxSpeed = 0;
    const speeds = new Float32Array(count);
    for (let i = 0; i < count; i++) {
      const dt = i > 0 ? (parsedData.timestamps[i] - parsedData.timestamps[i - 1]) / 1000.0 : 1.0;
      speeds[i] = dt > 0 ? deltaDistances[i] / dt : 0;
      maxSpeed = Math.max(maxSpeed, speeds[i]);
    }

    // TubeGeometry layout: (tubularSegments + 1) rings, each with (radialSegments + 1) vertices
    const tubSegs = this.tubeParams.tubularSegments;
    const radSegs = this.tubeParams.radialSegments;
    const ringSize = radSegs + 1;

    const colors = new Float32Array(vertexCount * 3);

    for (let v = 0; v < vertexCount; v++) {
      // Determine which ring this vertex belongs to
      const ring = Math.floor(v / ringSize);
      const u = ring / tubSegs; // 0..1 along tube

      // Map u to data index
      const dataIndex = Math.min(Math.floor(u * (count - 1)), count - 1);

      let r, g, b;

      if (colorMode === 'depth') {
        // Use the actual vertex Y position for smooth depth coloring
        const depth = Math.abs(positions[dataIndex * 3 + 2]);
        const t = maxDepth > 0 ? depth / maxDepth : 0;

        r = 0.0 + t * (0.24 - 0.0);     // cyan -> indigo
        g = 0.83 + t * (0.0 - 0.83);
        b = 1.0 + t * (0.6 - 1.0);
      } else if (colorMode === 'speed') {
        const speed = speeds[dataIndex];
        const t = maxSpeed > 0 ? Math.min(speed / maxSpeed, 1.0) : 0;

        r = 0.0 + t * (1.0 - 0.0);       // green -> red
        g = 1.0 + t * (0.2 - 1.0);
        b = 0.53 + t * (0.2 - 0.53);
      } else {
        // flags mode
        const flag = flags[dataIndex];
        const color = new THREE.Color(flagColor(flag));
        r = color.r;
        g = color.g;
        b = color.b;
      }

      colors[v * 3 + 0] = r;
      colors[v * 3 + 1] = g;
      colors[v * 3 + 2] = b;
    }

    tubeGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  }

  setColorMode(parsedData, mode) {
    if (!this.tubeMesh || !parsedData) {
      return;
    }

    this.applyVertexColors(this.tubeMesh.geometry, parsedData, mode);
    this.tubeMesh.geometry.attributes.color.needsUpdate = true;
  }

  fitCameraToPath(vizPositions, count) {
    const box = new THREE.Box3();

    for (let i = 0; i < count; i++) {
      box.expandByPoint(new THREE.Vector3(
        vizPositions[i * 3 + 0],
        vizPositions[i * 3 + 1],
        vizPositions[i * 3 + 2]
      ));
    }

    const center = new THREE.Vector3();
    box.getCenter(center);

    const size = new THREE.Vector3();
    box.getSize(size);

    const maxDim = Math.max(size.x, size.y, size.z);
    const fov = this.camera.fov * (Math.PI / 180);
    const cameraDistance = maxDim / (2 * Math.tan(fov / 2)) * 1.5;

    this.camera.position.set(
      center.x + cameraDistance * 0.5,
      center.y + cameraDistance * 0.5,
      center.z + cameraDistance * 0.5
    );

    this.controls.target.copy(center);
    this.controls.update();
  }

  createMarker() {
    const geometry = new THREE.SphereGeometry(0.3, 16, 16);

    // Add dummy color attribute to prevent WebGL VAO state mismatch
    // when renderer switches between tube (vertexColors) and sphere (no vertexColors)
    const posAttr = geometry.getAttribute('position');
    const dummyColors = new Float32Array(posAttr.count * 3);
    dummyColors.fill(0);
    geometry.setAttribute('color', new THREE.BufferAttribute(dummyColors, 3));

    const material = new THREE.MeshStandardMaterial({
      color: 0x00d4ff,
      emissive: 0x00d4ff,
      emissiveIntensity: 0.5
    });

    this.marker = new THREE.Mesh(geometry, material);
    this.scene.add(this.marker);

    return this.marker;
  }

  getPointAtIndex(index) {
    if (!this.parsedData) {
      return new THREE.Vector3();
    }

    const { positions, count } = this.parsedData;
    const i = Math.min(Math.max(0, index), count - 1);

    const px = positions[i * 3 + 0];
    const py = positions[i * 3 + 1];
    const pz = positions[i * 3 + 2];

    return new THREE.Vector3(px, -pz, py);
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    if (this.tubeMesh) {
      this.tubeMesh.geometry.dispose();
      this.tubeMesh.material.dispose();
      this.scene.remove(this.tubeMesh);
    }

    if (this.lineMesh) {
      this.lineMesh.geometry.dispose();
      this.lineMesh.material.dispose();
      this.scene.remove(this.lineMesh);
    }

    if (this.marker) {
      this.marker.geometry.dispose();
      this.marker.material.dispose();
      this.scene.remove(this.marker);
    }

    this.controls.dispose();
    this.renderer.dispose();
  }
}
