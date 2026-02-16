import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { flagColor } from './flag-decoder.js';
import { getDepthColor } from './depth-colormap.js';
import { getSpeedColor } from './speed-colormap.js';

export class SceneBuilder {
  constructor(canvas) {
    this.canvas = canvas;

    // Mobile detection for performance optimization
    const isMobile = navigator.maxTouchPoints > 0 && window.matchMedia('(max-width: 768px)').matches;
    const maxDPR = isMobile ? 1.5 : 2;

    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: !isMobile,  // Disable MSAA on mobile for performance
      alpha: false,
      powerPreference: isMobile ? 'low-power' : 'high-performance'
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, maxDPR));
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
    this.endpointMarkers = [];

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

  buildPath(parsedData, colorMode = 'depth', stats = null, depthScale = 1) {
    this.parsedData = parsedData;
    this.depthScale = depthScale;

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
      vizPositions[i * 3 + 1] = pz * depthScale;
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
      const tubeGeometry = new THREE.TubeGeometry(curve, tubularSegments, 1, radialSegments, false);

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
      depthP5 = stats.depthP5 * (this.depthScale || 1);
      depthP95 = stats.depthP95 * (this.depthScale || 1);
    } else {
      const depths = new Float32Array(count);
      for (let i = 0; i < count; i++) {
        depths[i] = Math.abs(positions[i * 3 + 2]);
      }
      depths.sort();
      depthP5 = (depths[Math.floor(count * 0.05)] ?? 0) * (this.depthScale || 1);
      depthP95 = (depths[Math.floor(count * 0.95)] ?? 0) * (this.depthScale || 1);
    }
    const depthRange = depthP95 - depthP5;

    const speeds = new Float32Array(count);
    for (let i = 0; i < count; i++) {
      const dt = i > 0 ? (parsedData.timestamps[i] - parsedData.timestamps[i - 1]) / 1000.0 : 1.0;
      speeds[i] = dt > 0 ? deltaDistances[i] / dt : 0;
    }

    // Compute speed percentiles for color normalization (matches depth approach)
    const sortedSpeeds = Float32Array.from(speeds).sort();
    const speedP5 = sortedSpeeds[Math.floor(count * 0.05)] ?? 0;
    const speedP95 = sortedSpeeds[Math.floor(count * 0.95)] ?? 0;
    const speedRange = speedP95 - speedP5;

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
        const depth = Math.abs(posAttr.getY(v));
        const t = depthRange > 0 ? Math.max(0, Math.min(1, (depth - depthP5) / depthRange)) : 0;

        const color = getDepthColor(t);
        r = color.r;
        g = color.g;
        b = color.b;
      } else if (colorMode === 'speed') {
        const fIdx = u * (count - 1);
        const i0 = Math.floor(fIdx);
        const i1 = Math.min(i0 + 1, count - 1);
        const frac = fIdx - i0;
        const speed = speeds[i0] + frac * (speeds[i1] - speeds[i0]);
        const t = speedRange > 0 ? Math.max(0, Math.min(1, (speed - speedP5) / speedRange)) : 0;

        const color = getSpeedColor(t);
        r = color.r;
        g = color.g;
        b = color.b;
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

    this._addDummyColorAttr(geometry);

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

    return new THREE.Vector3(px, pz * (this.depthScale || 1), py);
  }

  _addDummyColorAttr(geometry) {
    const count = geometry.getAttribute('position').count;
    geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(count * 3), 3));
  }

  createTextSprite(text) {
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d');
    const fontSize = 96;

    canvas.width = 512;
    canvas.height = 256;

    context.fillStyle = 'rgba(255, 255, 255, 0.9)';
    context.fillRect(0, 0, canvas.width, canvas.height);

    context.font = `bold ${fontSize}px Arial`;
    context.fillStyle = 'rgba(0, 0, 0, 0.9)';
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillText(text, canvas.width / 2, canvas.height / 2);

    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(6, 3, 1);

    return sprite;
  }

  createEndpointMarkers(parsedData) {
    // Clean up previous markers
    for (const obj of this.endpointMarkers) {
      if (obj.geometry) obj.geometry.dispose();
      if (obj.material) {
        if (obj.material.map) obj.material.map.dispose();
        obj.material.dispose();
      }
      this.scene.remove(obj);
    }
    this.endpointMarkers = [];

    if (!parsedData || !parsedData.positions || parsedData.count === 0) {
      return;
    }

    const { positions, count } = parsedData;

    // START marker at origin (0, 0, 0)
    const startGeometry = new THREE.SphereGeometry(1.0, 32, 32);

    this._addDummyColorAttr(startGeometry);

    const startMaterial = new THREE.MeshStandardMaterial({
      color: 0x22ff22,
      emissive: 0x00ff00,
      emissiveIntensity: 0.6,
      roughness: 0.2,
      metalness: 0.3
    });

    const startMarker = new THREE.Mesh(startGeometry, startMaterial);
    startMarker.position.set(0, 0, 0);
    this.scene.add(startMarker);
    this.endpointMarkers.push(startMarker);

    // START label
    const startLabel = this.createTextSprite('START');
    startLabel.position.set(0, 3, 0);
    this.scene.add(startLabel);
    this.endpointMarkers.push(startLabel);

    // END marker at last data point
    const lastIndex = count - 1;
    const px = positions[lastIndex * 3 + 0];
    const py = positions[lastIndex * 3 + 1];
    const pz = positions[lastIndex * 3 + 2];

    // Apply coordinate transform: data(px, py, pz) -> viz(px, pz, py)
    const endPos = new THREE.Vector3(px, pz * (this.depthScale || 1), py);

    const endGeometry = new THREE.SphereGeometry(1.0, 32, 32);

    this._addDummyColorAttr(endGeometry);

    const endMaterial = new THREE.MeshStandardMaterial({
      color: 0xff8822,
      emissive: 0xff6600,
      emissiveIntensity: 0.6,
      roughness: 0.2,
      metalness: 0.3
    });

    const endMarker = new THREE.Mesh(endGeometry, endMaterial);
    endMarker.position.copy(endPos);
    this.scene.add(endMarker);
    this.endpointMarkers.push(endMarker);

    // END label
    const endLabel = this.createTextSprite('END');
    endLabel.position.set(endPos.x, endPos.y + 3, endPos.z);
    this.scene.add(endLabel);
    this.endpointMarkers.push(endLabel);
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

    // Clean up endpoint markers
    for (const obj of this.endpointMarkers) {
      if (obj.geometry) obj.geometry.dispose();
      if (obj.material) {
        if (obj.material.map) obj.material.map.dispose();
        obj.material.dispose();
      }
      this.scene.remove(obj);
    }
    this.endpointMarkers = [];

    this.curve = null;
    this.pathBoundingBox = null;

    this.controls.dispose();
    this.renderer.dispose();
  }
}
