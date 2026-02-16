import * as THREE from 'three';
import { SimplexNoise3D } from './simplex-noise.js';

export class CaveEnvironment {
  constructor(scene) {
    this.scene = scene;
    this.caveMesh = null;
    this.gridGroup = null;
    this._labelTextureCache = new Map();
  }

  build(curve, pathMetrics) {
    const { size, center, maxDim, box } = pathMetrics;

    this.buildCaveTunnel(curve, maxDim);
    this.buildGrid(center, size, box);
  }

  buildCaveTunnel(curve, maxDim) {
    const baseRadius = Math.min(Math.max(maxDim * 0.05, 1.5), 8.0);
    const tubularSegments = Math.min(Math.round(curve.points.length * 4), 600);
    const radialSegments = 24;

    const geometry = new THREE.TubeGeometry(
      curve,
      tubularSegments,
      baseRadius,
      radialSegments,
      false
    );

    geometry.computeVertexNormals();

    const noise = new SimplexNoise3D(42);
    const position = geometry.attributes.position;
    const normal = geometry.attributes.normal;
    const totalRings = tubularSegments + 1;

    for (let i = 0; i < position.count; i++) {
      const x = position.getX(i);
      const y = position.getY(i);
      const z = position.getZ(i);

      const nx = normal.getX(i);
      const ny = normal.getY(i);
      const nz = normal.getZ(i);

      const ringIndex = Math.floor(i / (radialSegments + 1));
      const t = ringIndex / totalRings;

      const radiusScale = 0.75 + 0.5 * (noise.noise3D(t * 2.0, 0, 0) * 0.5 + 0.5);

      const octave1 = noise.noise3D(x * 0.3, y * 0.3, z * 0.3) * baseRadius * 0.35;
      const octave2 = noise.noise3D(x * 1.2, y * 1.2, z * 1.2) * baseRadius * 0.10;

      const displacement = (octave1 + octave2) * radiusScale;

      position.setXYZ(i, x + nx * displacement, y + ny * displacement, z + nz * displacement);
    }

    position.needsUpdate = true;
    geometry.computeVertexNormals();

    const material = new THREE.MeshStandardMaterial({
      color: 0xc4b5a0,
      roughness: 0.75,
      metalness: 0.1,
      side: THREE.BackSide,
      flatShading: true,
    });

    this.caveMesh = new THREE.Mesh(geometry, material);
    this.scene.add(this.caveMesh);
  }

  buildGrid(_center, size, box) {
    this.gridGroup = new THREE.Group();

    const spacing = Math.max(Math.max(size.x, size.z), Math.abs(box.min.y)) > 50 ? 10 : 5;

    const xMin = Math.floor(box.min.x / spacing) * spacing;
    const xMax = Math.ceil(box.max.x / spacing) * spacing;
    const zMin = Math.floor(box.min.z / spacing) * spacing;
    const zMax = Math.ceil(box.max.z / spacing) * spacing;
    const yMin = Math.min(Math.floor(box.min.y / spacing) * spacing, -2 * spacing);

    const gridMaterial = new THREE.LineBasicMaterial({
      color: 0x336699,
      opacity: 0.45,
      transparent: true,
      depthWrite: false,
    });

    // Horizontal floor grid (rectangular, fits bounding box)
    const floorPositions = [];
    for (let x = xMin; x <= xMax; x += spacing) {
      floorPositions.push(x, 0, zMin);
      floorPositions.push(x, 0, zMax);
    }
    for (let z = zMin; z <= zMax; z += spacing) {
      floorPositions.push(xMin, 0, z);
      floorPositions.push(xMax, 0, z);
    }
    const floorGeometry = new THREE.BufferGeometry();
    floorGeometry.setAttribute('position', new THREE.Float32BufferAttribute(floorPositions, 3));
    this.gridGroup.add(new THREE.LineSegments(floorGeometry, gridMaterial));

    // Back wall grid (at min Z edge)
    const wallZ = zMin - spacing;
    const wallMaterial = new THREE.LineBasicMaterial({
      color: 0x0066cc,
      opacity: 0.4,
      transparent: true,
      depthWrite: false,
    });

    const wallPositions = [];
    for (let y = 0; y >= yMin; y -= spacing) {
      wallPositions.push(xMin, y, wallZ);
      wallPositions.push(xMax, y, wallZ);
    }
    for (let x = xMin; x <= xMax; x += spacing) {
      wallPositions.push(x, 0, wallZ);
      wallPositions.push(x, yMin, wallZ);
    }
    const wallGeometry = new THREE.BufferGeometry();
    wallGeometry.setAttribute('position', new THREE.Float32BufferAttribute(wallPositions, 3));
    this.gridGroup.add(new THREE.LineSegments(wallGeometry, wallMaterial));

    // Side wall grid (at max X edge)
    const wallX = xMax + spacing;
    const sideWallPositions = [];
    for (let y = 0; y >= yMin; y -= spacing) {
      sideWallPositions.push(wallX, y, zMin);
      sideWallPositions.push(wallX, y, zMax);
    }
    for (let z = zMin; z <= zMax; z += spacing) {
      sideWallPositions.push(wallX, 0, z);
      sideWallPositions.push(wallX, yMin, z);
    }
    const sideWallGeometry = new THREE.BufferGeometry();
    sideWallGeometry.setAttribute('position', new THREE.Float32BufferAttribute(sideWallPositions, 3));
    this.gridGroup.add(new THREE.LineSegments(sideWallGeometry, wallMaterial.clone()));

    // X-axis labels along far Z edge of floor
    for (let x = xMin; x <= xMax; x += spacing) {
      const label = Math.abs(x - xMin).toFixed(0);
      const sprite = this._createLabel(`${label}m`, new THREE.Vector3(x, 0.3, zMax + 1));
      this.gridGroup.add(sprite);
    }

    // X-axis labels along near Z edge of floor (mirrored)
    for (let x = xMin; x <= xMax; x += spacing) {
      const label = Math.abs(x - xMin).toFixed(0);
      const sprite = this._createLabel(`${label}m`, new THREE.Vector3(x, 0.3, zMin - 1));
      this.gridGroup.add(sprite);
    }

    // Z-axis labels along left X edge of floor
    for (let z = zMin; z <= zMax; z += spacing) {
      const label = Math.abs(z - zMin).toFixed(0);
      const sprite = this._createLabel(`${label}m`, new THREE.Vector3(xMin - 1, 0.3, z));
      this.gridGroup.add(sprite);
    }

    // Z-axis labels along right X edge of floor (mirrored)
    for (let z = zMin; z <= zMax; z += spacing) {
      const label = Math.abs(z - zMin).toFixed(0);
      const sprite = this._createLabel(`${label}m`, new THREE.Vector3(xMax + 1, 0.3, z));
      this.gridGroup.add(sprite);
    }

    // Depth labels on back wall - right edge
    for (let y = -spacing; y >= yMin; y -= spacing) {
      const sprite = this._createLabel(`${y}m`, new THREE.Vector3(xMax + 1, y, wallZ));
      this.gridGroup.add(sprite);
    }

    // Depth labels on back wall - left edge (mirrored)
    for (let y = -spacing; y >= yMin; y -= spacing) {
      const sprite = this._createLabel(`${y}m`, new THREE.Vector3(xMin - 1, y, wallZ));
      this.gridGroup.add(sprite);
    }

    // Depth labels on side wall - far edge
    for (let y = -spacing; y >= yMin; y -= spacing) {
      const sprite = this._createLabel(`${y}m`, new THREE.Vector3(wallX, y, zMax + 1));
      this.gridGroup.add(sprite);
    }

    // Depth labels on side wall - near edge (mirrored)
    for (let y = -spacing; y >= yMin; y -= spacing) {
      const sprite = this._createLabel(`${y}m`, new THREE.Vector3(wallX, y, zMin - 1));
      this.gridGroup.add(sprite);
    }

    // Flat axis arrows at positive ends
    const arrowSize = spacing * 1.2;
    const arrowMaterials = {
      x: new THREE.MeshBasicMaterial({ color: 0xff4444, side: THREE.DoubleSide }),
      y: new THREE.MeshBasicMaterial({ color: 0x44ff44, side: THREE.DoubleSide }),
      z: new THREE.MeshBasicMaterial({ color: 0x4444ff, side: THREE.DoubleSide })
    };

    // Create elongated arrow shape (clearly points in one direction)
    const createArrowShape = () => {
      const shape = new THREE.Shape();
      shape.moveTo(0, arrowSize * 0.7);       // Tip (longer)
      shape.lineTo(-arrowSize * 0.25, 0);     // Left wing
      shape.lineTo(-arrowSize * 0.08, 0);     // Inner left
      shape.lineTo(-arrowSize * 0.08, -arrowSize * 0.4);  // Tail left
      shape.lineTo(arrowSize * 0.08, -arrowSize * 0.4);   // Tail right
      shape.lineTo(arrowSize * 0.08, 0);      // Inner right
      shape.lineTo(arrowSize * 0.25, 0);      // Right wing
      shape.closePath();
      return new THREE.ShapeGeometry(shape);
    };

    // X-axis arrow (red) pointing +X - flat on floor
    const xArrow = new THREE.Mesh(createArrowShape(), arrowMaterials.x);
    xArrow.position.set(xMax + spacing * 0.8, 0.1, (zMin + zMax) / 2);
    xArrow.rotation.x = -Math.PI / 2;
    xArrow.rotation.z = -Math.PI / 2;
    this.gridGroup.add(xArrow);

    // Z-axis arrow (blue) pointing +Z - flat on floor
    const zArrow = new THREE.Mesh(createArrowShape(), arrowMaterials.z);
    zArrow.position.set((xMin + xMax) / 2, 0.1, zMax + spacing * 0.8);
    zArrow.rotation.x = -Math.PI / 2;
    zArrow.rotation.z = Math.PI;
    this.gridGroup.add(zArrow);

    // Y-axis arrow (green) pointing down - flat on back wall
    const yArrow = new THREE.Mesh(createArrowShape(), arrowMaterials.y);
    yArrow.position.set(xMax + spacing * 0.8, yMin - spacing * 0.3, wallZ);
    yArrow.rotation.y = Math.PI / 2;
    yArrow.rotation.z = Math.PI;
    this.gridGroup.add(yArrow);

    this.scene.add(this.gridGroup);
  }

  _createLabel(text, position) {
    let texture = this._labelTextureCache.get(text);

    if (!texture) {
      const canvas = document.createElement('canvas');
      canvas.width = 256;
      canvas.height = 128;
      const context = canvas.getContext('2d');
      context.font = 'bold 80px sans-serif';
      context.fillStyle = '#0066cc';
      context.textAlign = 'center';
      context.fillText(text, 128, 96);

      texture = new THREE.CanvasTexture(canvas);
      this._labelTextureCache.set(text, texture);
    }

    const spriteMaterial = new THREE.SpriteMaterial({
      map: texture,
      opacity: 0.7,
      depthWrite: false,
    });

    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.position.copy(position);
    sprite.scale.set(6, 3, 1);

    return sprite;
  }

  setVisible(element, visible) {
    const targets = {
      cave: this.caveMesh,
      grid: this.gridGroup,
    };

    const target = targets[element];
    if (target) {
      target.visible = visible;
    }
  }

  dispose() {
    if (this.caveMesh) {
      this.scene.remove(this.caveMesh);
      this.caveMesh.geometry.dispose();
      this.caveMesh.material.dispose();
      this.caveMesh = null;
    }

    if (this.gridGroup) {
      this.gridGroup.traverse((child) => {
        if (child.geometry) {
          child.geometry.dispose();
        }
        if (child.material) {
          if (child.material.map) {
            child.material.map.dispose();
          }
          child.material.dispose();
        }
      });
      this.scene.remove(this.gridGroup);
      this.gridGroup = null;
    }

    if (this._labelTextureCache) {
      for (const texture of this._labelTextureCache.values()) {
        texture.dispose();
      }
      this._labelTextureCache.clear();
    }
  }
}
