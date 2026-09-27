package main

import (
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"testing"
)

const urCommit = "89bbe795f38a7ab00fb66fe8831dfff79dc99edf"

func loadURSpec(t *testing.T, model string) (*RobotSpec, *CompiledRobot) {
	t.Helper()
	spec, c, err := loadCompiled(filepath.Join("..", "..", "robots", model+".grobot.json"))
	if err != nil {
		t.Fatalf("load %s: %v", model, err)
	}
	return spec, c
}

func TestYAMLSubset(t *testing.T) {
	m, err := parseYAMLSubset("# c\na:\n  b: 1.5 # trailing\n  c:\n    d: !degrees 180\n  e: true\nf: x\n")
	if err != nil {
		t.Fatal(err)
	}
	if v, _ := yamlFloat(m, "a.b"); v != 1.5 {
		t.Errorf("a.b = %v", v)
	}
	if v, _ := yamlFloat(m, "a.c.d"); math.Abs(v-math.Pi) > 1e-15 {
		t.Errorf("!degrees 180 = %v, want pi", v)
	}
	if m["a"].(yamlMap)["e"] != true || m["f"] != "x" {
		t.Errorf("scalars: %#v", m)
	}
	if _, err := parseYAMLSubset("a:\n  - 1\n"); err == nil {
		t.Error("sequences must be rejected, not misread")
	}
}

func TestJacobiEigen(t *testing.T) {
	A := mat3{{0.3, 0.01, -0.02}, {0.01, 0.2, 0.005}, {-0.02, 0.005, 0.1}}
	vals, V := jacobiEigen(A)
	if math.Abs(V.det()-1) > 1e-12 {
		t.Fatalf("V is not a proper rotation: det %v", V.det())
	}
	D := mat3{{vals[0], 0, 0}, {0, vals[1], 0}, {0, 0, vals[2]}}
	B := V.mul(D).mul(V.transpose())
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if math.Abs(A[i][j]-B[i][j]) > 1e-14 {
				t.Fatalf("reconstruction error at %d,%d: %v vs %v", i, j, A[i][j], B[i][j])
			}
		}
	}
	q := matToQuat(V)
	R := quatToMat(q)
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if math.Abs(R[i][j]-V[i][j]) > 1e-12 {
				t.Fatalf("quat round trip mismatch")
			}
		}
	}
}

// The committed specs must be exactly what the importer produces from the committed manufacturer
// YAML -- the spec files are generated artifacts with a reproducible provenance, not hand edits.
func TestImportURReproducesCommittedSpecs(t *testing.T) {
	dir := filepath.Join("..", "..", "robots", "sources", "universal_robots")
	for _, model := range []string{"ur3e", "ur5e", "ur10e"} {
		load := func(kind string) yamlMap {
			raw, err := os.ReadFile(filepath.Join(dir, model+"_"+kind+".yaml"))
			if err != nil {
				t.Fatal(err)
			}
			m, err := parseYAMLSubset(string(raw))
			if err != nil {
				t.Fatalf("%s %s: %v", model, kind, err)
			}
			return m
		}
		spec, err := ImportUR(model, urCommit, load("default_kinematics"), load("physical_parameters"), load("joint_limits"))
		if err != nil {
			t.Fatal(err)
		}
		if p := spec.Validate(); len(p) > 0 {
			t.Fatalf("%s invalid: %v", model, p)
		}
		tmp := filepath.Join(t.TempDir(), model+".json")
		if err := WriteRobotSpec(tmp, spec); err != nil {
			t.Fatal(err)
		}
		got, _ := os.ReadFile(tmp)
		want, _ := os.ReadFile(filepath.Join("..", "..", "robots", model+".grobot.json"))
		if string(got) != string(want) {
			t.Errorf("%s: robots/%s.grobot.json is out of date with its manufacturer sources -- regenerate with `gbtool robot import-ur`", model, model)
		}
	}
}

// UR publishes DH parameters for the UR5e (a2=-0.425, a3=-0.3922, d1=0.1625, d4=0.1333,
// d5=0.0997, d6=0.0996). At all-zero joint angles the DH flange sits at
// (a2+a3, -(d4+d6), d1-d5). Our URDF-frame FK over UR's own default_kinematics.yaml must land
// on the same point: two independent parameterizations of the same arm agreeing.
func TestUR5eZeroPoseMatchesDH(t *testing.T) {
	_, c := loadURSpec(t, "ur5e")
	fk := c.ForwardKinematics(make([]float64, 6))
	want := vec3{-0.425 - 0.3922, -(0.1333 + 0.0996), 0.1625 - 0.0997}
	if d := fk.TCP.sub(want).norm(); d > 1e-6 {
		t.Fatalf("zero-pose flange %v, DH says %v (off by %g m)", fk.TCP, want, d)
	}
}

// Power balance: for any state, sum_i tau_i*qd_i == dE/dt (E = kinetic + gravitational
// potential), where tau comes from inverse dynamics at (q, qd, qdd) and dE/dt is measured by
// central differences along the same trajectory. This checks every term of RNEA (inertia
// tensors, Coriolis/centrifugal, gravity) against an independent energy computation.
func TestRNEAPowerBalance(t *testing.T) {
	g := vec3{0, 0, -9.81}
	rng := rand.New(rand.NewSource(7))
	for _, model := range []string{"ur3e", "ur5e", "ur10e"} {
		_, c := loadURSpec(t, model)
		for trial := 0; trial < 20; trial++ {
			n := len(c.Joints)
			q, qd, qdd := make([]float64, n), make([]float64, n), make([]float64, n)
			for i := 0; i < n; i++ {
				q[i] = rng.Float64()*4 - 2
				qd[i] = rng.Float64()*2 - 1
				qdd[i] = rng.Float64()*4 - 2
			}
			tau := c.InverseDynamics(q, qd, qdd, g)
			power := 0.0
			for i := range tau {
				power += tau[i] * qd[i]
			}
			const eps = 1e-5
			state := func(s float64) ([]float64, []float64) {
				qs, qds := make([]float64, n), make([]float64, n)
				for i := 0; i < n; i++ {
					qs[i] = q[i] + qd[i]*s + 0.5*qdd[i]*s*s
					qds[i] = qd[i] + qdd[i]*s
				}
				return qs, qds
			}
			qp, qdp := state(eps)
			qm, qdm := state(-eps)
			kp, pp := c.Energy(qp, qdp, g)
			km, pm := c.Energy(qm, qdm, g)
			dE := ((kp + pp) - (km + pm)) / (2 * eps)
			if math.Abs(power-dE) > 1e-5*math.Max(1, math.Abs(dE)) {
				t.Fatalf("%s trial %d: sum(tau*qd)=%.9f but dE/dt=%.9f", model, trial, power, dE)
			}
		}
	}
}

func TestMinimumJerkAndFeasibility(t *testing.T) {
	if minimumJerk(0) != 0 || minimumJerk(1) != 1 || math.Abs(minimumJerk(0.5)-0.5) > 1e-15 {
		t.Fatal("minimum jerk endpoints/midpoint wrong")
	}
	_, c := loadURSpec(t, "ur5e")
	slow, err := parseKeyframes("0:0,-90,0,-90,0,0;2:60,-60,-60,-90,90,0", 6, true)
	if err != nil {
		t.Fatal(err)
	}
	rep := CheckFeasibility(c, bakeKeyframes(slow, 64), 64, vec3{0, 0, -9.81})
	if !rep.Feasible {
		t.Fatalf("a 2 s move should be feasible: %+v", rep.Joints)
	}
	// Same move in 0.25 s: minimum-jerk peak speed is 1.875 * 60deg / 0.25s = 450 deg/s > 180.
	fast, _ := parseKeyframes("0:0,-90,0,-90,0,0;0.25:60,-60,-60,-90,90,0", 6, true)
	rep = CheckFeasibility(c, bakeKeyframes(fast, 64), 64, vec3{0, 0, -9.81})
	if rep.Feasible || rep.Joints[0].VelViolations == 0 {
		t.Fatalf("a 0.25 s 60-degree move must violate the 180 deg/s datasheet speed")
	}
}

// Euler-Lagrange, joint by joint: tau_i = d/dt(dL/dqd_i) - dL/dq_i with L = KE - PE, every
// derivative taken numerically from Energy(). Unlike the power balance above, this sees the
// gyroscopic w x (I w) terms (which do no work, so power balance is blind to them -- found by
// mutation-testing RNEA).
func TestRNEAEulerLagrange(t *testing.T) {
	g := vec3{0, 0, -9.81}
	rng := rand.New(rand.NewSource(11))
	for _, model := range []string{"ur3e", "ur5e", "ur10e"} {
		_, c := loadURSpec(t, model)
		n := len(c.Joints)
		for trial := 0; trial < 10; trial++ {
			q, qd, qdd := make([]float64, n), make([]float64, n), make([]float64, n)
			for i := 0; i < n; i++ {
				q[i] = rng.Float64()*4 - 2
				qd[i] = rng.Float64()*4 - 2
				qdd[i] = rng.Float64()*4 - 2
			}
			tau := c.InverseDynamics(q, qd, qdd, g)
			L := func(qs, qds []float64) float64 { k, p := c.Energy(qs, qds, g); return k - p }
			const eps, del = 1e-4, 1e-4
			for i := 0; i < n; i++ {
				mom := func(s float64) float64 { // dL/dqd_i along the trajectory at time s
					qs, qa, qb := make([]float64, n), make([]float64, n), make([]float64, n)
					for k := 0; k < n; k++ {
						qs[k] = q[k] + qd[k]*s + 0.5*qdd[k]*s*s
						qa[k] = qd[k] + qdd[k]*s
						qb[k] = qa[k]
					}
					qa[i] += del
					qb[i] -= del
					return (L(qs, qa) - L(qs, qb)) / (2 * del)
				}
				qa, qb := append([]float64(nil), q...), append([]float64(nil), q...)
				qa[i] += del
				qb[i] -= del
				dLdq := (L(qa, qd) - L(qb, qd)) / (2 * del)
				want := (mom(eps)-mom(-eps))/(2*eps) - dLdq
				if math.Abs(tau[i]-want) > 1e-4*math.Max(1, math.Abs(want)) {
					t.Fatalf("%s trial %d joint %d: RNEA tau=%.8f, Euler-Lagrange %.8f", model, trial, i, tau[i], want)
				}
			}
		}
	}
}
