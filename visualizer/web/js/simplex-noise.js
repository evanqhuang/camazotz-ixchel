/**
 * 3D Simplex Noise Implementation
 * Based on Stefan Gustavson's public domain algorithm
 */

// Skew factors for 3D simplex noise
const F3 = 1.0 / 3.0;
const G3 = 1.0 / 6.0;

export class SimplexNoise3D {
    constructor(seed = 0) {
        // Gradient vectors for 3D (12 edges of a cube)
        this.grad3 = [
            [1, 1, 0], [-1, 1, 0], [1, -1, 0], [-1, -1, 0],
            [1, 0, 1], [-1, 0, 1], [1, 0, -1], [-1, 0, -1],
            [0, 1, 1], [0, -1, 1], [0, 1, -1], [0, -1, -1]
        ];

        // Initialize permutation table with seed
        this.perm = new Uint8Array(512);
        this.permMod12 = new Uint8Array(512);

        const p = new Uint8Array(256);
        for (let i = 0; i < 256; i++) {
            p[i] = i;
        }

        // Shuffle using seed
        this._shuffle(p, seed);

        // Duplicate permutation table
        for (let i = 0; i < 512; i++) {
            this.perm[i] = p[i & 255];
            this.permMod12[i] = this.perm[i] % 12;
        }
    }

    _shuffle(array, seed) {
        // Simple seeded shuffle
        let random = this._seededRandom(seed);
        for (let i = array.length - 1; i > 0; i--) {
            const j = Math.floor(random() * (i + 1));
            [array[i], array[j]] = [array[j], array[i]];
        }
    }

    _seededRandom(seed) {
        // Simple LCG for deterministic randomness
        let state = seed;
        return () => {
            state = (Math.imul(state, 1664525) + 1013904223) | 0;
            return (state >>> 0) / 4294967296;
        };
    }

    _dot(g, x, y, z) {
        return g[0] * x + g[1] * y + g[2] * z;
    }

    noise3D(xin, yin, zin) {
        // Skew input space to determine simplex cell
        const s = (xin + yin + zin) * F3;
        const i = Math.floor(xin + s);
        const j = Math.floor(yin + s);
        const k = Math.floor(zin + s);

        const t = (i + j + k) * G3;
        const X0 = i - t;
        const Y0 = j - t;
        const Z0 = k - t;

        // Distances from cell origin
        const x0 = xin - X0;
        const y0 = yin - Y0;
        const z0 = zin - Z0;

        // Determine which simplex we're in
        let i1, j1, k1, i2, j2, k2;

        if (x0 >= y0) {
            if (y0 >= z0) {
                i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 1; k2 = 0;
            } else if (x0 >= z0) {
                i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 0; k2 = 1;
            } else {
                i1 = 0; j1 = 0; k1 = 1; i2 = 1; j2 = 0; k2 = 1;
            }
        } else {
            if (y0 < z0) {
                i1 = 0; j1 = 0; k1 = 1; i2 = 0; j2 = 1; k2 = 1;
            } else if (x0 < z0) {
                i1 = 0; j1 = 1; k1 = 0; i2 = 0; j2 = 1; k2 = 1;
            } else {
                i1 = 0; j1 = 1; k1 = 0; i2 = 1; j2 = 1; k2 = 0;
            }
        }

        // Offsets for remaining corners
        const x1 = x0 - i1 + G3;
        const y1 = y0 - j1 + G3;
        const z1 = z0 - k1 + G3;
        const x2 = x0 - i2 + 2.0 * G3;
        const y2 = y0 - j2 + 2.0 * G3;
        const z2 = z0 - k2 + 2.0 * G3;
        const x3 = x0 - 1.0 + 3.0 * G3;
        const y3 = y0 - 1.0 + 3.0 * G3;
        const z3 = z0 - 1.0 + 3.0 * G3;

        // Hash coordinates for gradient indices
        const ii = i & 255;
        const jj = j & 255;
        const kk = k & 255;

        // Calculate contribution from four corners
        let n0 = 0, n1 = 0, n2 = 0, n3 = 0;

        let t0 = 0.6 - x0 * x0 - y0 * y0 - z0 * z0;
        if (t0 >= 0) {
            const gi0 = this.permMod12[ii + this.perm[jj + this.perm[kk]]];
            t0 *= t0;
            n0 = t0 * t0 * this._dot(this.grad3[gi0], x0, y0, z0);
        }

        let t1 = 0.6 - x1 * x1 - y1 * y1 - z1 * z1;
        if (t1 >= 0) {
            const gi1 = this.permMod12[ii + i1 + this.perm[jj + j1 + this.perm[kk + k1]]];
            t1 *= t1;
            n1 = t1 * t1 * this._dot(this.grad3[gi1], x1, y1, z1);
        }

        let t2 = 0.6 - x2 * x2 - y2 * y2 - z2 * z2;
        if (t2 >= 0) {
            const gi2 = this.permMod12[ii + i2 + this.perm[jj + j2 + this.perm[kk + k2]]];
            t2 *= t2;
            n2 = t2 * t2 * this._dot(this.grad3[gi2], x2, y2, z2);
        }

        let t3 = 0.6 - x3 * x3 - y3 * y3 - z3 * z3;
        if (t3 >= 0) {
            const gi3 = this.permMod12[ii + 1 + this.perm[jj + 1 + this.perm[kk + 1]]];
            t3 *= t3;
            n3 = t3 * t3 * this._dot(this.grad3[gi3], x3, y3, z3);
        }

        // Sum contributions and scale to [-1, 1]
        return 32.0 * (n0 + n1 + n2 + n3);
    }
}
