import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { flagColor } from './flag-decoder.js';
import { getDepthColor } from './depth-colormap.js';

export class SceneBuilder {
  constructor(canvas) {
    this.canvas = canvas;

    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      alpha: false
    });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0xe8ecf1);

    this.scene = new THREE.Scene();

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

    this._onDocPointerMove = (e) => {
      if (e.target !== canvas) {
        canvas.dispatchEvent(new PointerEvent('pointermove', {
          pointerId: e.pointerId, pointerType: e.pointerType,
          clientX: e.clientX, clientY: e.clientY,
          button: e.button, buttons: e.buttons
        }));
      }
    };
    this._onDocPointerUp = (e) => {
      if (e.target !== canvas) {
        canvas.dispatchEvent(new PointerEvent('pointerup', {
          pointerId: e.pointerId, pointerType: e.pointerType,
          clientX: e.clientX, clientY: e.clientY,
          button: e.button, buttons: e.buttons
        }));
      }
    };
    document.addEventListener('pointermove', this._onDocPointerMove);
    document.addEventListener('pointerup', this._onDocPointerUp);

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 10, 10);
    this.scene.add(directionalLight);

    const axesHelper = new THREE.AxesHelper(0.5);
    this.scene.add(axesHelper);

    this.tubeMesh = null;
    this.marker = null;
    this.parsedData = null;
    this.tubeParams = null;
    this.curve = null;
    this.pathBoundingBox = null;
    this._animFrameId = null;

    this.handleResize();
    this._onResize = () => this.handleResize();
    window.addEventListener('resize', this._onResize);
  }

  handleResize() {
    const width = window.innerWidth;
    const height = window.innerHeight;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  buildPath(parsedData, colorMode = 'depth', stats = null) {
    this.parsedData = parsedData;

    if (this.tubeMesh) {
      this.scene.remove(this.tubeMesh);
      this.tubeMesh.geometry.dispose();
      this.tubeMesh.material.dispose();
    }

    const { positions, count } = parsedData;

    const vizPositions = new Float32Array(count * 3);
    for (let i = 0; i < count; i++) {
      const px = positions[i * 3 + 0];
      const py = positions[i * 3 + 1];
      const pz = positions[i * 3 + 2];

      vizPositions[i * 3 + 0] = px;
      vizPositions[i * 3 + 1] = pz;
      vizPositions[i * 3 + 2] = py;
    }

    const TARGET_CURVE_POINTS = 1500;
    const downsampleRate = Math.max(1, Math.floor(count / TARGET_CURVE_POINTS));
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
      this.curve = curve;
      const tubeGeometry = new THREE.TubeGeometry(curve, tubularSegments, 1.5, radialSegments, false);

      this.applyVertexColors(tubeGeometry, parsedData, colorMode, stats);

      const tubeMaterial = new THREE.MeshBasicMaterial({
        vertexColors: true
      });

      this.tubeMesh = new THREE.Mesh(tubeGeometry, tubeMaterial);
      this.scene.add(this.tubeMesh);
    }


    this.fitCameraToPath(vizPositions, count);
  }

  applyVertexColors(tubeGeometry, parsedData, colorMode, stats = null) {
    const { positions, deltaDistances, flags, count } = parsedData;
    const posAttr = tubeGeometry.getAttribute('position');
    const vertexCount = posAttr.count;

    // Compute depth percentiles for color normalization
    let depthP5, depthP95;
    if (stats) {
      depthP5 = stats.depthP5;
      depthP95 = stats.depthP95;
    } else {
      const depths = new Float32Array(count);
      for (let i = 0; i < count; i++) {
        depths[i] = Math.abs(positions[i * 3 + 2]);
      }
      depths.sort();
      depthP5 = depths[Math.floor(count * 0.05)] ?? 0;
      depthP95 = depths[Math.floor(count * 0.95)] ?? 0;
    }
    const depthRange = depthP95 - depthP5;

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
        const depth = Math.abs(positions[dataIndex * 3 + 2]);
        const t = depthRange > 0 ? Math.max(0, Math.min(1, (depth - depthP5) / depthRange)) : 0;

        const color = getDepthColor(t);
        r = color.r;
        g = color.g;
        b = color.b;
      } else if (colorMode === 'speed') {
        const speed = speeds[dataIndex];
        const t = maxSpeed > 0 ? Math.min(speed / maxSpeed, 1.0) : 0;

        r = 0.13 + t * (0.8 - 0.13);     // forest green -> dark red
        g = 0.55 + t * (0.13 - 0.55);
        b = 0.13 + t * (0.13 - 0.13);
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
    const tempVec = new THREE.Vector3();

    for (let i = 0; i < count; i++) {
      tempVec.set(
        vizPositions[i * 3 + 0],
        vizPositions[i * 3 + 1],
        vizPositions[i * 3 + 2]
      );
      box.expandByPoint(tempVec);
    }

    this.pathBoundingBox = box.clone();

    const center = new THREE.Vector3();
    box.getCenter(center);

    const size = new THREE.Vector3();
    box.getSize(size);

    const maxDim = Math.max(size.x, size.y, size.z);
    const fov = this.camera.fov * (Math.PI / 180);
    const cameraDistance = maxDim / (2 * Math.tan(fov / 2)) * 1.1;

    // Get the start point of the dive (index 0)
    const startPoint = new THREE.Vector3(
      vizPositions[0],
      vizPositions[1],
      vizPositions[2]
    );

    // Direction from start toward center (XZ plane only for horizontal offset)
    const dirToCenter = new THREE.Vector3().subVectors(center, startPoint);
    dirToCenter.y = 0;
    dirToCenter.normalize();

    // Position camera behind the start point with high isometric viewpoint
    // - Offset horizontally behind the start (opposite direction from center)
    // - Elevated above for isometric view
    this.camera.position.set(
      startPoint.x - dirToCenter.x * cameraDistance * 0.6,
      center.y + cameraDistance * 0.8,
      startPoint.z - dirToCenter.z * cameraDistance * 0.6
    );

    // Target below center to reduce blank space at top of viewport
    this.controls.target.set(center.x, center.y - cameraDistance * 0.25, center.z);
    this.controls.update();
  }

  getPathMetrics() {
    if (!this.pathBoundingBox) return null;
    const size = new THREE.Vector3();
    this.pathBoundingBox.getSize(size);
    const center = new THREE.Vector3();
    this.pathBoundingBox.getCenter(center);
    const maxDim = Math.max(size.x, size.y, size.z);
    return { size, center, maxDim, box: this.pathBoundingBox.clone() };
  }

  createMarker() {
    // Large bright red sphere marker
    const geometry = new THREE.SphereGeometry(1.5, 32, 32);

    // Add dummy color attribute for WebGL VAO compatibility
    const posAttr = geometry.getAttribute('position');
    const dummyColors = new Float32Array(posAttr.count * 3);
    dummyColors.fill(0);
    geometry.setAttribute('color', new THREE.BufferAttribute(dummyColors, 3));

    const material = new THREE.MeshStandardMaterial({
      color: 0xff2222,        // Bright red
      emissive: 0xff0000,     // Strong red glow
      emissiveIntensity: 0.8,
      roughness: 0.2,
      metalness: 0.3
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

    return new THREE.Vector3(px, pz, py);
  }

  animate() {
    this._animFrameId = requestAnimationFrame(() => this.animate());
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    cancelAnimationFrame(this._animFrameId);
    window.removeEventListener('resize', this._onResize);
    document.removeEventListener('pointermove', this._onDocPointerMove);
    document.removeEventListener('pointerup', this._onDocPointerUp);

    if (this.tubeMesh) {
      this.tubeMesh.geometry.dispose();
      this.tubeMesh.material.dispose();
      this.scene.remove(this.tubeMesh);
    }

    if (this.marker) {
      this.marker.geometry.dispose();
      this.marker.material.dispose();
      this.scene.remove(this.marker);
    }

    this.curve = null;
    this.pathBoundingBox = null;

    this.controls.dispose();
    this.renderer.dispose();
  }
}
