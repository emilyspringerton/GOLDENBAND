// rnea.go — exact rigid-body inverse dynamics (Recursive Newton-Euler) for a compiled robot: the
// joint torques a motion REQUIRES, computed analytically from q, qd, qdd -- no simulation, no
// controller in the loop. This is HQ-SPEC-SIM-100 §3's feasibility pass ("any frame demanding
// infeasible joint velocity/torque is flagged at authoring time, not discovered on a test
// stand"), and it doubles as an independent cross-check of the XPBD simulator in src/grb.c: two
// unrelated algorithms (analytic Newton-Euler here, constraint-projection dynamics there) must
// agree on e.g. the gravity-holding torque of a real UR5e pose (tests/test_grobot.c).
//
// World-frame formulation. Gravity enters as a fictitious upward base acceleration (the standard
// RNEA trick), so tau includes gravity load.
package main

type robotFK struct {
	R      []mat3 // child link world rotation
	P      []vec3 // joint origin world position (== child link origin)
	Z      []vec3 // joint axis, world
	C      []vec3 // child link COM, world
	TCP    vec3   // tool center point, world
	TCPRot mat3
}

// ForwardKinematics places every link for joint angles q (base at the world origin, identity).
func (c *CompiledRobot) ForwardKinematics(q []float64) robotFK {
	n := len(c.Joints)
	fk := robotFK{R: make([]mat3, n), P: make([]vec3, n), Z: make([]vec3, n), C: make([]vec3, n)}
	for i, j := range c.Joints {
		pR, pP := identity3(), vec3{}
		if j.Parent >= 0 {
			pR, pP = fk.R[j.Parent], fk.P[j.Parent]
		}
		frame := pR.mul(quatToMat(j.OriginQuat))
		fk.P[i] = pP.add(pR.apply(j.OriginXYZ))
		fk.Z[i] = frame.apply(j.Axis)
		fk.R[i] = frame.mul(axisAngleMat(j.Axis, q[i]))
		fk.C[i] = fk.P[i].add(fk.R[i].apply(j.COM))
	}
	if c.TCPJoint >= 0 {
		fk.TCP = fk.P[c.TCPJoint].add(fk.R[c.TCPJoint].apply(c.TCPXYZ))
		fk.TCPRot = fk.R[c.TCPJoint]
	}
	return fk
}

// linkInertiaWorld returns the link's inertia tensor about its COM, in world coordinates.
func (c *CompiledRobot) linkInertiaWorld(i int, R mat3) mat3 {
	j := c.Joints[i]
	P := R.mul(quatToMat(j.PrincipalQuat))
	D := mat3{{j.PrincipalMoments[0], 0, 0}, {0, j.PrincipalMoments[1], 0}, {0, 0, j.PrincipalMoments[2]}}
	return P.mul(D).mul(P.transpose())
}

// InverseDynamics returns tau (N*m per joint) for the motion state (q, qd, qdd) under gravity g
// (world vector, e.g. {0,0,-9.81} for a z-up robot).
func (c *CompiledRobot) InverseDynamics(q, qd, qdd []float64, g vec3) []float64 {
	n := len(c.Joints)
	fk := c.ForwardKinematics(q)
	w := make([]vec3, n)
	dw := make([]vec3, n)
	a := make([]vec3, n)  // linear acceleration of the joint origin
	ac := make([]vec3, n) // linear acceleration of the COM
	baseA := g.scale(-1)
	for i, j := range c.Joints {
		var wp, dwp, ap, pp vec3
		ap = baseA
		if j.Parent >= 0 {
			wp, dwp, ap, pp = w[j.Parent], dw[j.Parent], a[j.Parent], fk.P[j.Parent]
		}
		z := fk.Z[i]
		w[i] = wp.add(z.scale(qd[i]))
		dw[i] = dwp.add(z.scale(qdd[i])).add(wp.cross(z.scale(qd[i])))
		r := fk.P[i].sub(pp)
		a[i] = ap.add(dwp.cross(r)).add(wp.cross(wp.cross(r)))
		rc := fk.C[i].sub(fk.P[i])
		ac[i] = a[i].add(dw[i].cross(rc)).add(w[i].cross(w[i].cross(rc)))
	}
	f := make([]vec3, n)
	m := make([]vec3, n) // moment about the joint origin
	tau := make([]float64, n)
	for i := n - 1; i >= 0; i-- {
		j := c.Joints[i]
		I := c.linkInertiaWorld(i, fk.R[i])
		fi := ac[i].scale(j.Mass)
		rc := fk.C[i].sub(fk.P[i])
		mi := I.apply(dw[i]).add(w[i].cross(I.apply(w[i]))).add(rc.cross(fi))
		f[i] = f[i].add(fi)
		m[i] = m[i].add(mi)
		tau[i] = m[i].dot(fk.Z[i])
		if j.Parent >= 0 {
			p := j.Parent
			f[p] = f[p].add(f[i])
			m[p] = m[p].add(m[i]).add(fk.P[i].sub(fk.P[p]).cross(f[i]))
		}
	}
	return tau
}

// Energy returns (kinetic, potential) energy of the robot at state (q, qd) -- used by tests to
// verify InverseDynamics through the power balance sum(tau*qd) == dE/dt.
func (c *CompiledRobot) Energy(q, qd []float64, g vec3) (float64, float64) {
	n := len(c.Joints)
	fk := c.ForwardKinematics(q)
	w := make([]vec3, n)
	v := make([]vec3, n) // joint-origin linear velocity
	var ke, pe float64
	for i, j := range c.Joints {
		var wp, vp, pp vec3
		if j.Parent >= 0 {
			wp, vp, pp = w[j.Parent], v[j.Parent], fk.P[j.Parent]
		}
		w[i] = wp.add(fk.Z[i].scale(qd[i]))
		v[i] = vp.add(wp.cross(fk.P[i].sub(pp)))
		vc := v[i].add(w[i].cross(fk.C[i].sub(fk.P[i])))
		I := c.linkInertiaWorld(i, fk.R[i])
		ke += 0.5*j.Mass*vc.dot(vc) + 0.5*w[i].dot(I.apply(w[i]))
		pe -= j.Mass * g.dot(fk.C[i])
	}
	return ke, pe
}
