// robot.go — the GOLDEN BAND robot spec (`<name>.grobot.json`, human-readable, every number
// cited to a manufacturer source) and its compiled runtime form (`<name>.grobot`, fixed binary,
// read by src/grobot.c). See format/GROBOT_FORMAT.md.
//
// Founder real-time, 2026-09-27: "we need to get goldenband rigged up with real robot data from
// industrial data sheets." HQ-SPEC-SIM-100 §3 already said what that data is: "for
// hardware-bound characters, actuator metadata: torque limits, velocity limits" plus per-link
// mass/inertia -- "the real NUMBERS CAD/motor-datasheet work produces" (SHANKPIT
// RAGDOLL_ORIENTATION_NORTHSTAR's own robotics section). The spec is URDF-shaped on purpose
// (origin xyz/rpy per joint, inertial per link, SI units) because that is the form manufacturers
// actually publish machine-readable data in.
package main

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"sort"
)

type RobotSource struct {
	ID        string `json:"id"`
	Title     string `json:"title"`
	URL       string `json:"url"`
	Commit    string `json:"commit,omitempty"`
	Retrieved string `json:"retrieved,omitempty"`
	License   string `json:"license,omitempty"`
}

type InertiaSpec struct {
	Ixx float64 `json:"ixx"`
	Ixy float64 `json:"ixy"`
	Ixz float64 `json:"ixz"`
	Iyy float64 `json:"iyy"`
	Iyz float64 `json:"iyz"`
	Izz float64 `json:"izz"`
}

type LinkSpec struct {
	Name string `json:"name"`
	// Mass in kg; COM (m) in the link frame; the inertia tensor (kg*m^2, about the COM) is
	// expressed in the link frame rotated by inertia_rpy -- exactly URDF's <inertial><origin>
	// (xyz positions the COM, rpy orients the inertia frame).
	Mass       float64     `json:"mass"`
	COM        vec3        `json:"com"`
	InertiaRPY vec3        `json:"inertia_rpy"`
	Inertia    InertiaSpec `json:"inertia"`
	Source     string      `json:"source"`
	Note       string      `json:"note,omitempty"`
}

type JointSpec struct {
	Name      string  `json:"name"`
	Type      string  `json:"type"` // "revolute" | "continuous"
	Parent    string  `json:"parent"`
	Child     string  `json:"child"`
	OriginXYZ vec3    `json:"origin_xyz"` // m, in the parent link frame
	OriginRPY vec3    `json:"origin_rpy"` // rad
	Axis      vec3    `json:"axis"`       // unit, in the joint frame
	Lower     float64 `json:"lower"`      // rad
	Upper     float64 `json:"upper"`      // rad
	Velocity  float64 `json:"velocity"`   // rad/s, datasheet max joint speed
	Effort    float64 `json:"effort"`     // N*m, datasheet max joint torque
	Damping   float64 `json:"damping"`    // N*m*s/rad; 0 when the manufacturer publishes none
	Source    string  `json:"source"`
	Note      string  `json:"note,omitempty"`
}

type TCPSpec struct {
	Link string `json:"link"`
	XYZ  vec3   `json:"xyz"`
}

type RobotSpec struct {
	GRobotVersion int           `json:"grobot_version"`
	Name          string        `json:"name"`
	Manufacturer  string        `json:"manufacturer"`
	Model         string        `json:"model"`
	Payload       float64       `json:"payload_kg,omitempty"`
	Sources       []RobotSource `json:"sources"`
	Notes         []string      `json:"notes,omitempty"`
	BaseLink      string        `json:"base_link"`
	Links         []LinkSpec    `json:"links"`
	Joints        []JointSpec   `json:"joints"`
	TCP           TCPSpec       `json:"tcp"`
}

func ReadRobotSpec(path string) (*RobotSpec, []byte, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, nil, err
	}
	var s RobotSpec
	if err := json.Unmarshal(raw, &s); err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
	}
	return &s, raw, nil
}

func WriteRobotSpec(path string, s *RobotSpec) error {
	b, err := json.MarshalIndent(s, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, append(b, '\n'), 0644)
}

func (s *RobotSpec) link(name string) *LinkSpec {
	for i := range s.Links {
		if s.Links[i].Name == name {
			return &s.Links[i]
		}
	}
	return nil
}

// linkInertiaInLinkFrame rotates the published tensor into the link frame: I_link = R I R^T.
func (l *LinkSpec) inertiaInLinkFrame() mat3 {
	t := l.Inertia
	I := mat3{{t.Ixx, t.Ixy, t.Ixz}, {t.Ixy, t.Iyy, t.Iyz}, {t.Ixz, t.Iyz, t.Izz}}
	R := rpyMat(l.InertiaRPY[0], l.InertiaRPY[1], l.InertiaRPY[2])
	return R.mul(I).mul(R.transpose())
}

// Validate checks the physical and structural sanity of a spec. Every problem is returned, not
// just the first, so a datasheet transcription error is fixed in one pass.
func (s *RobotSpec) Validate() []string {
	var p []string
	if s.GRobotVersion != 1 {
		p = append(p, fmt.Sprintf("grobot_version=%d, only 1 is supported", s.GRobotVersion))
	}
	if s.Name == "" || len(s.Name) >= 32 {
		p = append(p, "name must be 1..31 bytes")
	}
	srcIDs := map[string]bool{}
	for _, src := range s.Sources {
		srcIDs[src.ID] = true
	}
	if s.link(s.BaseLink) == nil {
		p = append(p, fmt.Sprintf("base_link %q is not a declared link", s.BaseLink))
	}
	for _, l := range s.Links {
		if len(l.Name) >= 32 {
			p = append(p, fmt.Sprintf("link %q: name too long", l.Name))
		}
		if l.Name != s.BaseLink && !(l.Mass > 0) {
			p = append(p, fmt.Sprintf("link %q: mass must be > 0", l.Name))
		}
		if !srcIDs[l.Source] {
			p = append(p, fmt.Sprintf("link %q: source %q is not in sources[]", l.Name, l.Source))
		}
		vals, _ := jacobiEigen(l.inertiaInLinkFrame())
		if l.Name != s.BaseLink {
			for _, v := range vals {
				if !(v > 0) {
					p = append(p, fmt.Sprintf("link %q: inertia tensor is not positive definite (principal moment %g)", l.Name, v))
				}
			}
			// Physical realizability: every principal moment <= the sum of the other two.
			sorted := []float64{vals[0], vals[1], vals[2]}
			sort.Float64s(sorted)
			if sorted[2] > (sorted[0]+sorted[1])*(1+1e-6) {
				p = append(p, fmt.Sprintf("link %q: inertia violates the triangle inequality (%g > %g + %g)", l.Name, sorted[2], sorted[0], sorted[1]))
			}
		}
	}
	seenChild := map[string]bool{s.BaseLink: true}
	for i, j := range s.Joints {
		if len(j.Name) >= 32 {
			p = append(p, fmt.Sprintf("joint %q: name too long", j.Name))
		}
		if j.Type != "revolute" && j.Type != "continuous" {
			p = append(p, fmt.Sprintf("joint %q: type %q (only revolute/continuous in v0)", j.Name, j.Type))
		}
		if s.link(j.Child) == nil || s.link(j.Parent) == nil {
			p = append(p, fmt.Sprintf("joint %q: parent/child link missing", j.Name))
		}
		if !seenChild[j.Parent] {
			p = append(p, fmt.Sprintf("joint %d %q: parent link %q must be the base or a child of an EARLIER joint", i, j.Name, j.Parent))
		}
		if seenChild[j.Child] {
			p = append(p, fmt.Sprintf("joint %q: child link %q already attached (kinematic trees only)", j.Name, j.Child))
		}
		seenChild[j.Child] = true
		if math.Abs(j.Axis.norm()-1) > 1e-6 {
			p = append(p, fmt.Sprintf("joint %q: axis must be unit length", j.Name))
		}
		if j.Type == "revolute" && !(j.Lower < j.Upper) {
			p = append(p, fmt.Sprintf("joint %q: lower must be < upper", j.Name))
		}
		if !(j.Velocity > 0) || !(j.Effort > 0) {
			p = append(p, fmt.Sprintf("joint %q: velocity and effort limits must be > 0 (datasheet values)", j.Name))
		}
		if !srcIDs[j.Source] {
			p = append(p, fmt.Sprintf("joint %q: source %q is not in sources[]", j.Name, j.Source))
		}
	}
	if s.link(s.TCP.Link) == nil || s.TCP.Link == s.BaseLink {
		p = append(p, fmt.Sprintf("tcp.link %q must be a moving link", s.TCP.Link))
	}
	return p
}

// ---------------------------------------------------------------- compiled binary (.grobot)

const (
	grobotHeaderSize = 104
	grobotRecordSize = 280
	grobotMaxJoints  = 64 // matches src/grobot.h GROBOT_MAX_JOINTS
)

// CompiledJoint is one .grobot record: a joint plus the rigid body of its child link, with the
// inertia tensor diagonalized (principal frame) since grb bodies store a diagonal inertia.
type CompiledJoint struct {
	Name, Child      string
	Parent           int32 // index of the joint whose child is this joint's parent link; -1 = base
	Continuous       bool
	OriginXYZ        vec3
	OriginQuat       quat
	Axis             vec3
	Lower, Upper     float64
	Velocity, Effort float64
	Damping          float64
	Mass             float64
	COM              vec3 // in the child link frame
	PrincipalQuat    quat // child link frame -> principal frame
	PrincipalMoments vec3
}

type CompiledRobot struct {
	Name     string
	SpecHash [32]byte
	TCPJoint int32
	TCPXYZ   vec3
	Joints   []CompiledJoint
}

func CompileRobot(s *RobotSpec, specBytes []byte) (*CompiledRobot, error) {
	if probs := s.Validate(); len(probs) > 0 {
		return nil, fmt.Errorf("invalid robot spec:\n  %s", joinLines(probs))
	}
	if len(s.Joints) > grobotMaxJoints {
		return nil, fmt.Errorf("%d joints exceeds GROBOT_MAX_JOINTS (%d)", len(s.Joints), grobotMaxJoints)
	}
	c := &CompiledRobot{Name: s.Name, SpecHash: sha256.Sum256(specBytes), TCPJoint: -1, TCPXYZ: s.TCP.XYZ}
	childIndex := map[string]int32{}
	for i, j := range s.Joints {
		l := s.link(j.Child)
		parent := int32(-1)
		if j.Parent != s.BaseLink {
			parent = childIndex[j.Parent]
		}
		// Rotate the published tensor into the link frame, then diagonalize it.
		vals, V := jacobiEigen(l.inertiaInLinkFrame())
		c.Joints = append(c.Joints, CompiledJoint{
			Name: j.Name, Child: j.Child, Parent: parent, Continuous: j.Type == "continuous",
			OriginXYZ:  j.OriginXYZ,
			OriginQuat: matToQuat(rpyMat(j.OriginRPY[0], j.OriginRPY[1], j.OriginRPY[2])),
			Axis:       j.Axis.unit(),
			Lower:      j.Lower, Upper: j.Upper, Velocity: j.Velocity, Effort: j.Effort, Damping: j.Damping,
			Mass:             l.Mass,
			COM:              l.COM,
			PrincipalQuat:    matToQuat(V),
			PrincipalMoments: vals,
		})
		childIndex[j.Child] = int32(i)
		if j.Child == s.TCP.Link {
			c.TCPJoint = int32(i)
		}
	}
	return c, nil
}

func joinLines(xs []string) string {
	out := ""
	for i, x := range xs {
		if i > 0 {
			out += "\n  "
		}
		out += x
	}
	return out
}

func (c *CompiledRobot) WriteFile(path string) error {
	buf := make([]byte, grobotHeaderSize+len(c.Joints)*grobotRecordSize)
	le := binary.LittleEndian
	copy(buf[0:4], "GRBT")
	le.PutUint32(buf[4:8], 1)
	le.PutUint32(buf[8:12], uint32(len(c.Joints)))
	le.PutUint32(buf[12:16], uint32(c.TCPJoint))
	putF64s(buf[16:40], c.TCPXYZ[:])
	copy(buf[40:72], c.Name)
	copy(buf[72:104], c.SpecHash[:])
	for i, j := range c.Joints {
		r := buf[grobotHeaderSize+i*grobotRecordSize:]
		copy(r[0:32], j.Name)
		copy(r[32:64], j.Child)
		le.PutUint32(r[64:68], uint32(j.Parent))
		typ := uint32(1)
		if j.Continuous {
			typ = 2
		}
		le.PutUint32(r[68:72], typ)
		f := []float64{}
		f = append(f, j.OriginXYZ[:]...)
		f = append(f, j.OriginQuat[:]...)
		f = append(f, j.Axis[:]...)
		f = append(f, j.Lower, j.Upper, j.Velocity, j.Effort, j.Damping, j.Mass)
		f = append(f, j.COM[:]...)
		f = append(f, j.PrincipalQuat[:]...)
		f = append(f, j.PrincipalMoments[:]...)
		if len(f) != 26 {
			panic("grobot record layout drifted")
		}
		putF64s(r[72:grobotRecordSize], f)
	}
	return os.WriteFile(path, buf, 0644)
}

func putF64s(dst []byte, vals []float64) {
	for i, v := range vals {
		binary.LittleEndian.PutUint64(dst[i*8:], math.Float64bits(v))
	}
}

// ---------------------------------------------------------------- skeleton (.gskel) for viewers

// Skeleton emits the robot's rest (all joints at 0) kinematic tree as a .gskel so every existing
// GOLDEN BAND consumer -- gpose FK, SHANKPIT's skeleton renderer, NOCK's animation repository --
// can load and display a robot rig without knowing anything about robots. Joint 0 is the base
// link; joint i+1 is spec joint i (named after the JOINT, so `<joint>.angle` clip channels name a
// real skeleton joint). Rest translation/rotation = the joint origin; units are meters, z-up.
func (c *CompiledRobot) Skeleton(baseName string) *GSkelFile {
	sk := &GSkelFile{Version: 1}
	worldR := []mat3{identity3()}
	worldP := []vec3{{}}
	sk.Joints = append(sk.Joints, GSkelJoint{Name: baseName, ParentIndex: -1, RestRotation: [4]float32{0, 0, 0, 1}, InverseBind: identityMat4()})
	for _, j := range c.Joints {
		pi := int(j.Parent) + 1
		R := worldR[pi].mul(quatToMat(j.OriginQuat))
		P := worldP[pi].add(worldR[pi].apply(j.OriginXYZ))
		worldR = append(worldR, R)
		worldP = append(worldP, P)
		sj := GSkelJoint{Name: j.Name, ParentIndex: int32(pi)}
		for k := 0; k < 3; k++ {
			sj.RestTranslation[k] = float32(j.OriginXYZ[k])
		}
		for k := 0; k < 4; k++ {
			sj.RestRotation[k] = float32(j.OriginQuat[k])
		}
		// inverse bind = inverse(world bind) = [R^T | -R^T P], column-major.
		Rt := R.transpose()
		t := Rt.apply(P).scale(-1)
		var m [16]float32
		for col := 0; col < 3; col++ {
			for row := 0; row < 3; row++ {
				m[col*4+row] = float32(Rt[row][col])
			}
		}
		m[12], m[13], m[14], m[15] = float32(t[0]), float32(t[1]), float32(t[2]), 1
		sj.InverseBind = m
		sk.Joints = append(sk.Joints, sj)
	}
	return sk
}
