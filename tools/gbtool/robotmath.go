// robotmath.go — small float64 vector/matrix/quaternion kit for the robot pipeline (spec
// compilation, forward kinematics, inverse dynamics). Conventions match src/grb.h exactly:
// quaternions are x,y,z,w; URDF rpy is R = Rz(yaw) * Ry(pitch) * Rx(roll); matrices are
// row-major [3][3] here (Go side only -- never written to disk in this form).
package main

import "math"

type vec3 [3]float64
type mat3 [3][3]float64
type quat [4]float64 // x, y, z, w

func (a vec3) add(b vec3) vec3      { return vec3{a[0] + b[0], a[1] + b[1], a[2] + b[2]} }
func (a vec3) sub(b vec3) vec3      { return vec3{a[0] - b[0], a[1] - b[1], a[2] - b[2]} }
func (a vec3) scale(s float64) vec3 { return vec3{a[0] * s, a[1] * s, a[2] * s} }
func (a vec3) dot(b vec3) float64   { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }
func (a vec3) cross(b vec3) vec3 {
	return vec3{a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]}
}
func (a vec3) norm() float64 { return math.Sqrt(a.dot(a)) }
func (a vec3) unit() vec3 {
	n := a.norm()
	if n == 0 {
		return a
	}
	return a.scale(1 / n)
}

func identity3() mat3 { return mat3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} }

func (m mat3) mul(n mat3) mat3 {
	var o mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			for k := 0; k < 3; k++ {
				o[i][j] += m[i][k] * n[k][j]
			}
		}
	}
	return o
}

func (m mat3) apply(v vec3) vec3 {
	return vec3{
		m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
		m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
		m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2],
	}
}

func (m mat3) transpose() mat3 {
	var o mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			o[i][j] = m[j][i]
		}
	}
	return o
}

func (m mat3) det() float64 {
	return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]) -
		m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0]) +
		m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])
}

// axisAngleMat is Rodrigues' rotation formula.
func axisAngleMat(axis vec3, angle float64) mat3 {
	a := axis.unit()
	c, s := math.Cos(angle), math.Sin(angle)
	t := 1 - c
	x, y, z := a[0], a[1], a[2]
	return mat3{
		{t*x*x + c, t*x*y - s*z, t*x*z + s*y},
		{t*x*y + s*z, t*y*y + c, t*y*z - s*x},
		{t*x*z - s*y, t*y*z + s*x, t*z*z + c},
	}
}

func rpyMat(roll, pitch, yaw float64) mat3 {
	return axisAngleMat(vec3{0, 0, 1}, yaw).mul(axisAngleMat(vec3{0, 1, 0}, pitch)).mul(axisAngleMat(vec3{1, 0, 0}, roll))
}

// matToQuat converts a proper rotation matrix to a unit quaternion (Shepperd's method), with the
// sign fixed so w >= 0 -- a deterministic representative, so compiled assets hash stably.
func matToQuat(m mat3) quat {
	tr := m[0][0] + m[1][1] + m[2][2]
	var q quat
	switch {
	case tr > 0:
		s := math.Sqrt(tr+1) * 2
		q = quat{(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s}
	case m[0][0] > m[1][1] && m[0][0] > m[2][2]:
		s := math.Sqrt(1+m[0][0]-m[1][1]-m[2][2]) * 2
		q = quat{0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s}
	case m[1][1] > m[2][2]:
		s := math.Sqrt(1+m[1][1]-m[0][0]-m[2][2]) * 2
		q = quat{(m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s}
	default:
		s := math.Sqrt(1+m[2][2]-m[0][0]-m[1][1]) * 2
		q = quat{(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s}
	}
	n := math.Sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
	for i := range q {
		q[i] /= n
	}
	if q[3] < 0 {
		for i := range q {
			q[i] = -q[i]
		}
	}
	return q
}

func quatToMat(q quat) mat3 {
	x, y, z, w := q[0], q[1], q[2], q[3]
	return mat3{
		{1 - 2*(y*y+z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)},
		{2 * (x*y + z*w), 1 - 2*(x*x+z*z), 2 * (y*z - x*w)},
		{2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2*(x*x+y*y)},
	}
}

// alignZTo returns the rotation taking +Z onto unit vector a (grb hinges turn about their frame's
// +Z, while URDF allows any joint axis).
func alignZTo(a vec3) mat3 {
	a = a.unit()
	z := vec3{0, 0, 1}
	c := z.dot(a)
	if c > 1-1e-12 {
		return identity3()
	}
	if c < -1+1e-12 {
		return axisAngleMat(vec3{1, 0, 0}, math.Pi)
	}
	return axisAngleMat(z.cross(a), math.Acos(c))
}

// jacobiEigen diagonalizes a symmetric 3x3 matrix: returns eigenvalues and a proper rotation V
// (columns = eigenvectors, det +1) with A = V * diag(vals) * V^T. Cyclic Jacobi, converges to
// machine precision for 3x3 in a handful of sweeps.
func jacobiEigen(a mat3) (vec3, mat3) {
	v := identity3()
	for sweep := 0; sweep < 64; sweep++ {
		off := a[0][1]*a[0][1] + a[0][2]*a[0][2] + a[1][2]*a[1][2]
		if off < 1e-30 {
			break
		}
		for p := 0; p < 2; p++ {
			for q := p + 1; q < 3; q++ {
				if math.Abs(a[p][q]) < 1e-300 {
					continue
				}
				theta := (a[q][q] - a[p][p]) / (2 * a[p][q])
				t := 1 / (math.Abs(theta) + math.Sqrt(theta*theta+1))
				if theta < 0 {
					t = -t
				}
				c := 1 / math.Sqrt(t*t+1)
				s := t * c
				var r mat3 = identity3()
				r[p][p], r[q][q], r[p][q], r[q][p] = c, c, s, -s
				a = r.transpose().mul(a).mul(r)
				v = v.mul(r)
			}
		}
	}
	vals := vec3{a[0][0], a[1][1], a[2][2]}
	if v.det() < 0 {
		for i := 0; i < 3; i++ {
			v[i][2] = -v[i][2]
		}
	}
	return vals, v
}
