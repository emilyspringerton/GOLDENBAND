// robot_cmd.go — `gbtool robot ...`: import manufacturer data, compile it, author reference
// motions for it, and check motions against its datasheet envelope.
//
//	gbtool robot import-ur --model ur5e --dir <yaml dir> --commit <sha> --out robots/ur5e.grobot.json
//	gbtool robot compile   --robot robots/ur5e.grobot.json --out build/ur5e
//	gbtool robot bake-motion --robot robots/ur5e.grobot.json --out wave --keyframes "0:0,-90,90,-90,-90,0;1.5:45,-70,70,-90,-90,0"
//	gbtool robot check     --robot robots/ur5e.grobot.json --clip wave [--annotate] [--csv torques.csv]
package main

import (
	"crypto/sha256"
	"encoding/hex"
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

func runRobot(args []string) int {
	if len(args) < 1 {
		fmt.Fprintln(os.Stderr, "robot: subcommand required (import-ur | compile | bake-motion | check)")
		return 1
	}
	switch args[0] {
	case "import-ur":
		return runRobotImportUR(args[1:])
	case "compile":
		return runRobotCompile(args[1:])
	case "bake-motion":
		return runRobotBakeMotion(args[1:])
	case "check":
		return runRobotCheck(args[1:])
	}
	fmt.Fprintf(os.Stderr, "robot: unknown subcommand %q\n", args[0])
	return 1
}

// ---------------------------------------------------------------- import-ur

// urJoints maps UR's own joint names to the link each one drives and the key its origin lives
// under in default_kinematics.yaml -- the exact topology of UR's ur_macro.xacro.
var urJoints = []struct{ joint, parent, child, kin string }{
	{"shoulder_pan_joint", "base_link_inertia", "shoulder_link", "shoulder"},
	{"shoulder_lift_joint", "shoulder_link", "upper_arm_link", "upper_arm"},
	{"elbow_joint", "upper_arm_link", "forearm_link", "forearm"},
	{"wrist_1_joint", "forearm_link", "wrist_1_link", "wrist_1"},
	{"wrist_2_joint", "wrist_1_link", "wrist_2_link", "wrist_2"},
	{"wrist_3_joint", "wrist_2_link", "wrist_3_link", "wrist_3"},
}

// ImportUR converts Universal Robots' own published description data (the three config/<model>/
// YAML files in github.com/UniversalRobots/Universal_Robots_ROS2_Description) into a spec.
func ImportUR(model, commit string, kinematics, physical, limits yamlMap) (*RobotSpec, error) {
	s := &RobotSpec{
		GRobotVersion: 1,
		Name:          model,
		Manufacturer:  "Universal Robots",
		Model:         strings.ToUpper(model[:len(model)-1]) + model[len(model)-1:],
		BaseLink:      "base_link_inertia",
		TCP:           TCPSpec{Link: "wrist_3_link"},
		Sources: []RobotSource{
			{
				ID:        "ur-ros2-description",
				Title:     "Universal Robots, Universal_Robots_ROS2_Description, config/" + model + "/{default_kinematics,physical_parameters,joint_limits}.yaml",
				URL:       "https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/" + commit + "/config/" + model,
				Commit:    commit,
				Retrieved: "2026-09-27",
				License:   "BSD-3-Clause",
			},
			{
				ID:    "ur-user-manual",
				Title: "Universal Robots e-Series User Manual (joint position/velocity limits; cited by UR's joint_limits.yaml)",
				URL:   "https://s3-eu-west-1.amazonaws.com/ur-support-site/69091/99404_UR5e_User_Manual_en_Global.pdf",
			},
			{
				ID:    "ur-max-joint-torques",
				Title: "Universal Robots support article \"Max. joint torques\" (joint effort limits; cited by UR's joint_limits.yaml)",
				URL:   "https://www.universal-robots.com/articles/ur-articles/max-joint-torques",
			},
		},
		Notes: []string{
			"Masses, centers of mass, inertia tensors and joint frames are Universal Robots' own published values (their calibrated nominal kinematics, not a per-robot calibration).",
			"base_link_inertia is the fixed base; its mass is recorded for completeness but never simulated (UR's own file flags it: 'This mass might be incorrect').",
			"Joint damping/friction and acceleration limits are not publicly available from the manufacturer; damping is 0 and acceleration is unconstrained here.",
		},
	}
	baseMass, err := yamlFloat(physical, "inertia_parameters.base_mass")
	if err != nil {
		return nil, err
	}
	s.Links = append(s.Links, LinkSpec{
		Name: "base_link_inertia", Mass: baseMass, Source: "ur-ros2-description",
		Note: "fixed to the world, never simulated; UR's own file flags this mass as possibly incorrect; inertia is a unit placeholder (UR publishes none for the base)",
		Inertia: InertiaSpec{Ixx: 1, Iyy: 1, Izz: 1},
	})
	for _, uj := range urJoints {
		link := strings.TrimSuffix(uj.child, "_link")
		get := func(path string) float64 {
			if err != nil {
				return 0
			}
			var v float64
			v, err = yamlFloat(physical, "inertia_parameters."+path)
			return v
		}
		l := LinkSpec{Name: uj.child, Source: "ur-ros2-description"}
		l.Mass = get(link + "_mass")
		l.COM = vec3{get("center_of_mass." + link + "_cog.x"), get("center_of_mass." + link + "_cog.y"), get("center_of_mass." + link + "_cog.z")}
		l.InertiaRPY = vec3{get("rotation." + link + ".roll"), get("rotation." + link + ".pitch"), get("rotation." + link + ".yaw")}
		l.Inertia = InertiaSpec{
			Ixx: get("tensor." + link + ".ixx"), Ixy: get("tensor." + link + ".ixy"), Ixz: get("tensor." + link + ".ixz"),
			Iyy: get("tensor." + link + ".iyy"), Iyz: get("tensor." + link + ".iyz"), Izz: get("tensor." + link + ".izz"),
		}
		if err != nil {
			return nil, err
		}
		s.Links = append(s.Links, l)

		kin := func(k string) float64 {
			if err != nil {
				return 0
			}
			var v float64
			v, err = yamlFloat(kinematics, "kinematics."+uj.kin+"."+k)
			return v
		}
		lim := func(k string) float64 {
			if err != nil {
				return 0
			}
			var v float64
			v, err = yamlFloat(limits, "joint_limits."+uj.joint+"."+k)
			return v
		}
		j := JointSpec{
			Name: uj.joint, Type: "revolute", Parent: uj.parent, Child: uj.child,
			OriginXYZ: vec3{kin("x"), kin("y"), kin("z")},
			OriginRPY: vec3{kin("roll"), kin("pitch"), kin("yaw")},
			Axis:      vec3{0, 0, 1},
			Velocity:  lim("max_velocity"), Effort: lim("max_effort"),
			Source: "ur-ros2-description",
			Note:   "effort: ur-max-joint-torques; position/velocity: ur-user-manual (both as transcribed by the manufacturer into joint_limits.yaml)",
		}
		// has_position_limits: false is UR's own marker for an infinitely-rotating joint (e.g. the
		// UR3e's wrist 3) -- a real "continuous" joint, not a missing value.
		if hp, ok := limits["joint_limits"].(yamlMap)[uj.joint].(yamlMap)["has_position_limits"].(bool); ok && !hp {
			j.Type = "continuous"
		} else {
			j.Lower, j.Upper = lim("min_position"), lim("max_position")
		}
		if uj.joint == "elbow_joint" {
			j.Note += "; UR's file limits the elbow to +/-180 deg because the shoulder physically blocks further rotation"
		}
		if err != nil {
			return nil, err
		}
		s.Joints = append(s.Joints, j)
	}
	return s, nil
}

func runRobotImportUR(args []string) int {
	fs := flag.NewFlagSet("robot import-ur", flag.ContinueOnError)
	model := fs.String("model", "", "UR model directory name, e.g. ur5e")
	dir := fs.String("dir", "", "directory holding <model>_{default_kinematics,physical_parameters,joint_limits}.yaml")
	commit := fs.String("commit", "", "Universal_Robots_ROS2_Description commit the YAML was fetched at")
	out := fs.String("out", "", "output spec path (.grobot.json)")
	if fs.Parse(args) != nil || *model == "" || *dir == "" || *out == "" || *commit == "" {
		fmt.Fprintln(os.Stderr, "robot import-ur: --model, --dir, --commit and --out are required")
		return 1
	}
	load := func(kind string) (yamlMap, error) {
		raw, err := os.ReadFile(filepath.Join(*dir, *model+"_"+kind+".yaml"))
		if err != nil {
			return nil, err
		}
		return parseYAMLSubset(string(raw))
	}
	kin, err1 := load("default_kinematics")
	phys, err2 := load("physical_parameters")
	lim, err3 := load("joint_limits")
	for _, err := range []error{err1, err2, err3} {
		if err != nil {
			fmt.Fprintf(os.Stderr, "robot import-ur: %v\n", err)
			return 1
		}
	}
	spec, err := ImportUR(*model, *commit, kin, phys, lim)
	if err != nil {
		fmt.Fprintf(os.Stderr, "robot import-ur: %v\n", err)
		return 1
	}
	if probs := spec.Validate(); len(probs) > 0 {
		fmt.Fprintf(os.Stderr, "robot import-ur: imported spec is invalid:\n  %s\n", joinLines(probs))
		return 1
	}
	if err := WriteRobotSpec(*out, spec); err != nil {
		fmt.Fprintf(os.Stderr, "robot import-ur: %v\n", err)
		return 1
	}
	fmt.Printf("imported %s (%d joints) -> %s\n", spec.Model, len(spec.Joints), *out)
	return 0
}

// ---------------------------------------------------------------- compile

func loadCompiled(path string) (*RobotSpec, *CompiledRobot, error) {
	spec, raw, err := ReadRobotSpec(path)
	if err != nil {
		return nil, nil, err
	}
	c, err := CompileRobot(spec, raw)
	return spec, c, err
}

func robotSkeletonHash(spec *RobotSpec, c *CompiledRobot) ([32]byte, error) {
	tmp, err := os.CreateTemp("", "grobot-*.gskel")
	if err != nil {
		return [32]byte{}, err
	}
	tmp.Close()
	defer os.Remove(tmp.Name())
	if err := c.Skeleton(spec.BaseLink).WriteFile(tmp.Name()); err != nil {
		return [32]byte{}, err
	}
	b, err := os.ReadFile(tmp.Name())
	if err != nil {
		return [32]byte{}, err
	}
	return sha256.Sum256(b), nil
}

func runRobotCompile(args []string) int {
	fs := flag.NewFlagSet("robot compile", flag.ContinueOnError)
	robot := fs.String("robot", "", "robot spec (.grobot.json)")
	out := fs.String("out", "", "output name (writes <name>.grobot + <name>.gskel)")
	if fs.Parse(args) != nil || *robot == "" || *out == "" {
		fmt.Fprintln(os.Stderr, "robot compile: --robot and --out are required")
		return 1
	}
	spec, c, err := loadCompiled(*robot)
	if err != nil {
		fmt.Fprintf(os.Stderr, "robot compile: %v\n", err)
		return 1
	}
	if err := c.WriteFile(*out + ".grobot"); err != nil {
		fmt.Fprintf(os.Stderr, "robot compile: %v\n", err)
		return 1
	}
	if err := c.Skeleton(spec.BaseLink).WriteFile(*out + ".gskel"); err != nil {
		fmt.Fprintf(os.Stderr, "robot compile: %v\n", err)
		return 1
	}
	total := 0.0
	for _, j := range c.Joints {
		total += j.Mass
	}
	fmt.Printf("compiled %s %s -> %s.grobot + %s.gskel (%d joints, %.3f kg moving mass, spec sha256 %s)\n",
		spec.Manufacturer, spec.Model, *out, *out, len(c.Joints), total, hex.EncodeToString(c.SpecHash[:])[:16])
	return 0
}

// ---------------------------------------------------------------- bake-motion

// minimumJerk is the classic quintic 10s^3 - 15s^4 + 6s^5 (Flash & Hogan 1985): zero velocity
// AND zero acceleration at both ends -- the standard smooth point-to-point robot move.
func minimumJerk(s float64) float64 {
	if s <= 0 {
		return 0
	}
	if s >= 1 {
		return 1
	}
	return s * s * s * (10 + s*(-15+6*s))
}

type keyframe struct {
	t float64
	q []float64
}

func parseKeyframes(s string, n int, degrees bool) ([]keyframe, error) {
	var kfs []keyframe
	for _, part := range strings.Split(s, ";") {
		part = strings.TrimSpace(part)
		if part == "" {
			continue
		}
		c := strings.Index(part, ":")
		if c < 0 {
			return nil, fmt.Errorf("keyframe %q: expected time:q1,q2,...", part)
		}
		t, err := strconv.ParseFloat(strings.TrimSpace(part[:c]), 64)
		if err != nil {
			return nil, fmt.Errorf("keyframe %q: bad time", part)
		}
		vals := strings.Split(part[c+1:], ",")
		if len(vals) != n {
			return nil, fmt.Errorf("keyframe at t=%g has %d values, robot has %d joints", t, len(vals), n)
		}
		kf := keyframe{t: t}
		for _, v := range vals {
			f, err := strconv.ParseFloat(strings.TrimSpace(v), 64)
			if err != nil {
				return nil, fmt.Errorf("keyframe at t=%g: bad value %q", t, v)
			}
			if degrees {
				f *= math.Pi / 180
			}
			kf.q = append(kf.q, f)
		}
		if len(kfs) > 0 && t <= kfs[len(kfs)-1].t {
			return nil, fmt.Errorf("keyframe times must strictly increase (t=%g)", t)
		}
		kfs = append(kfs, kf)
	}
	if len(kfs) < 2 || kfs[0].t != 0 {
		return nil, fmt.Errorf("need at least two keyframes, the first at t=0")
	}
	return kfs, nil
}

// bakeKeyframes samples a minimum-jerk path through the keyframes at tickRate, inclusive of the
// final keyframe's own tick.
func bakeKeyframes(kfs []keyframe, tickRate uint32) [][]float64 {
	last := kfs[len(kfs)-1].t
	ticks := int(math.Round(last*float64(tickRate))) + 1
	out := make([][]float64, ticks)
	seg := 0
	for k := 0; k < ticks; k++ {
		t := float64(k) / float64(tickRate)
		for seg < len(kfs)-2 && t > kfs[seg+1].t {
			seg++
		}
		a, b := kfs[seg], kfs[seg+1]
		s := minimumJerk((t - a.t) / (b.t - a.t))
		row := make([]float64, len(a.q))
		for i := range row {
			row[i] = a.q[i] + (b.q[i]-a.q[i])*s
		}
		out[k] = row
	}
	return out
}

func runRobotBakeMotion(args []string) int {
	fs := flag.NewFlagSet("robot bake-motion", flag.ContinueOnError)
	robot := fs.String("robot", "", "robot spec (.grobot.json)")
	out := fs.String("out", "", "output clip name (writes <name>.gband + <name>.gband.json)")
	keys := fs.String("keyframes", "", `"t:q1,...,qN;t:..." joint angles in degrees (or radians with --radians)`)
	radians := fs.Bool("radians", false, "keyframe angles are radians")
	tickRate := fs.Uint("tick-rate", 64, "ticks/second")
	who := fs.String("who", "", "authorship.who (default: a description of this bake)")
	tags := fs.String("tags", "showpiece", "comma-separated intent tags")
	if fs.Parse(args) != nil || *robot == "" || *out == "" || *keys == "" {
		fmt.Fprintln(os.Stderr, "robot bake-motion: --robot, --out and --keyframes are required")
		return 1
	}
	spec, c, err := loadCompiled(*robot)
	if err != nil {
		fmt.Fprintf(os.Stderr, "robot bake-motion: %v\n", err)
		return 1
	}
	kfs, err := parseKeyframes(*keys, len(c.Joints), !*radians)
	if err != nil {
		fmt.Fprintf(os.Stderr, "robot bake-motion: %v\n", err)
		return 1
	}
	rows := bakeKeyframes(kfs, uint32(*tickRate))
	skelHash, err := robotSkeletonHash(spec, c)
	if err != nil {
		fmt.Fprintf(os.Stderr, "robot bake-motion: %v\n", err)
		return 1
	}
	g := &GBandFile{Version: 1, TickRate: uint32(*tickRate), DurationTicks: uint32(len(rows)), NumChannels: uint32(len(c.Joints)), SkeletonHash: skelHash}
	for _, r := range rows {
		for _, v := range r {
			g.Data = append(g.Data, float32(v))
		}
	}
	if err := g.WriteFile(*out + ".gband"); err != nil {
		fmt.Fprintf(os.Stderr, "robot bake-motion: %v\n", err)
		return 1
	}
	var channels []string
	for _, j := range c.Joints {
		channels = append(channels, j.Name+".angle")
	}
	if *who == "" {
		*who = fmt.Sprintf("gbtool robot bake-motion: minimum-jerk path through %d authored keyframes for %s", len(kfs), spec.Model)
	}
	var tagList []string
	for _, t := range strings.Split(*tags, ",") {
		if t = strings.TrimSpace(t); t != "" {
			tagList = append(tagList, t)
		}
	}
	m := &Manifest{
		GBandVersion: 1, SkeletonHash: hex.EncodeToString(skelHash[:]), ContentHash: hex.EncodeToString(g.ContentHash[:]),
		TickRate: g.TickRate, DurationTicks: g.DurationTicks, Channels: channels,
		Authorship: Authorship{Kind: "human", Who: *who}, IntentTags: tagList,
		LoopPoints: LoopPoints{StartTick: 0, EndTick: g.DurationTicks},
	}
	if err := WriteManifest(*out+".gband.json", m); err != nil {
		fmt.Fprintf(os.Stderr, "robot bake-motion: %v\n", err)
		return 1
	}
	fmt.Printf("baked %s: %d ticks @ %d/s, %d joint channels -> %s.gband\n", spec.Model, g.DurationTicks, g.TickRate, len(channels), *out)
	return 0
}

// ---------------------------------------------------------------- check (feasibility)

type FeasibilityJoint struct {
	Name                          string
	MaxAbsQ, MaxAbsQd, MaxAbsTau  float64
	WorstQTick, WorstQdTick       int
	WorstTauTick                  int
	PosViolations, VelViolations  int
	TorqueViolations              int
	Lower, Upper, VelLimit, Effort float64
}

type FeasibilityReport struct {
	Joints   []FeasibilityJoint
	Ticks    int
	Feasible bool
	Torques  [][]float64 // per tick, per joint
}

// clipJointAngles maps a clip's `<joint>.angle` channels onto the robot's joint order. Joints
// the clip doesn't animate hold 0 (their rest angle).
func clipJointAngles(c *CompiledRobot, m *Manifest, g *GBandFile) ([][]float64, error) {
	idx := make([]int, len(c.Joints))
	found := 0
	for i, j := range c.Joints {
		idx[i] = -1
		for k, ch := range m.Channels {
			if ch == j.Name+".angle" {
				idx[i] = k
				found++
			}
		}
	}
	if found == 0 {
		return nil, fmt.Errorf("clip has no <joint>.angle channels for this robot")
	}
	rows := make([][]float64, g.DurationTicks)
	for t := range rows {
		row := make([]float64, len(c.Joints))
		for i := range row {
			if idx[i] >= 0 {
				row[i] = float64(g.Data[t*int(g.NumChannels)+idx[i]])
			}
		}
		rows[t] = row
	}
	return rows, nil
}

// CheckFeasibility differentiates the clip (central differences at the clip's own tick rate) and
// runs exact inverse dynamics on every tick. Violations are counted per joint per tick.
func CheckFeasibility(c *CompiledRobot, rows [][]float64, tickRate uint32, gravity vec3) FeasibilityReport {
	n, T := len(c.Joints), len(rows)
	dt := 1 / float64(tickRate)
	rep := FeasibilityReport{Ticks: T, Feasible: true}
	for _, j := range c.Joints {
		rep.Joints = append(rep.Joints, FeasibilityJoint{Name: j.Name, Lower: j.Lower, Upper: j.Upper, VelLimit: j.Velocity, Effort: j.Effort})
	}
	at := func(t int) []float64 {
		if t < 0 {
			t = 0
		}
		if t >= T {
			t = T - 1
		}
		return rows[t]
	}
	for t := 0; t < T; t++ {
		q, qm, qp := at(t), at(t-1), at(t+1)
		qd, qdd := make([]float64, n), make([]float64, n)
		for i := 0; i < n; i++ {
			span := 2 * dt
			if t == 0 || t == T-1 {
				span = dt
			}
			qd[i] = (qp[i] - qm[i]) / span
			qdd[i] = (qp[i] - 2*q[i] + qm[i]) / (dt * dt)
		}
		tau := c.InverseDynamics(q, qd, qdd, gravity)
		rep.Torques = append(rep.Torques, tau)
		for i := 0; i < n; i++ {
			fj := &rep.Joints[i]
			if math.Abs(q[i]) > fj.MaxAbsQ {
				fj.MaxAbsQ, fj.WorstQTick = math.Abs(q[i]), t
			}
			if math.Abs(qd[i]) > fj.MaxAbsQd {
				fj.MaxAbsQd, fj.WorstQdTick = math.Abs(qd[i]), t
			}
			if math.Abs(tau[i]) > fj.MaxAbsTau {
				fj.MaxAbsTau, fj.WorstTauTick = math.Abs(tau[i]), t
			}
			if !c.Joints[i].Continuous && (q[i] < fj.Lower || q[i] > fj.Upper) {
				fj.PosViolations++
			}
			if math.Abs(qd[i]) > fj.VelLimit {
				fj.VelViolations++
			}
			if math.Abs(tau[i]) > fj.Effort {
				fj.TorqueViolations++
			}
		}
	}
	for _, fj := range rep.Joints {
		if fj.PosViolations+fj.VelViolations+fj.TorqueViolations > 0 {
			rep.Feasible = false
		}
	}
	return rep
}

func parseVec3(s string) (vec3, error) {
	parts := strings.Split(s, ",")
	if len(parts) != 3 {
		return vec3{}, fmt.Errorf("expected x,y,z, got %q", s)
	}
	var v vec3
	for i, p := range parts {
		f, err := strconv.ParseFloat(strings.TrimSpace(p), 64)
		if err != nil {
			return vec3{}, err
		}
		v[i] = f
	}
	return v, nil
}

func runRobotCheck(args []string) int {
	fs := flag.NewFlagSet("robot check", flag.ContinueOnError)
	robot := fs.String("robot", "", "robot spec (.grobot.json)")
	clip := fs.String("clip", "", "clip name (<name>.gband + <name>.gband.json)")
	grav := fs.String("gravity", "0,0,-9.81", "world gravity (the robot base is z-up)")
	annotate := fs.Bool("annotate", false, "write the peak joint speed/torque into the manifest's safety block")
	csv := fs.String("csv", "", "write per-tick required torques to this CSV")
	if fs.Parse(args) != nil || *robot == "" || *clip == "" {
		fmt.Fprintln(os.Stderr, "robot check: --robot and --clip are required")
		return 1
	}
	spec, c, err := loadCompiled(*robot)
	if err != nil {
		fmt.Fprintf(os.Stderr, "robot check: %v\n", err)
		return 1
	}
	g, err := ReadGBandFile(*clip + ".gband")
	if err != nil {
		fmt.Fprintf(os.Stderr, "robot check: %v\n", err)
		return 1
	}
	m, err := ReadManifest(*clip + ".gband.json")
	if err != nil {
		fmt.Fprintf(os.Stderr, "robot check: %v\n", err)
		return 1
	}
	rows, err := clipJointAngles(c, m, g)
	if err != nil {
		fmt.Fprintf(os.Stderr, "robot check: %v\n", err)
		return 1
	}
	gv, err := parseVec3(*grav)
	if err != nil {
		fmt.Fprintf(os.Stderr, "robot check: --gravity: %v\n", err)
		return 1
	}
	rep := CheckFeasibility(c, rows, g.TickRate, gv)
	fmt.Printf("feasibility: %s %s vs clip %s (%d ticks @ %d/s), exact inverse dynamics (RNEA)\n", spec.Manufacturer, spec.Model, *clip, rep.Ticks, g.TickRate)
	fmt.Printf("  %-22s %9s %9s %8s %9s %9s %8s  %s\n", "joint", "|q|max", "range", "|qd|max", "limit", "|tau|max", "effort", "violations(pos/vel/tau)")
	peakV, peakT := 0.0, 0.0
	for _, fj := range rep.Joints {
		fmt.Printf("  %-22s %8.1f° %8.0f° %7.1f°/s %7.0f°/s %8.2fNm %6.0fNm  %d/%d/%d\n",
			fj.Name, fj.MaxAbsQ*180/math.Pi, fj.Upper*180/math.Pi, fj.MaxAbsQd*180/math.Pi, fj.VelLimit*180/math.Pi,
			fj.MaxAbsTau, fj.Effort, fj.PosViolations, fj.VelViolations, fj.TorqueViolations)
		peakV = math.Max(peakV, fj.MaxAbsQd)
		peakT = math.Max(peakT, fj.MaxAbsTau)
	}
	if *csv != "" {
		var sb strings.Builder
		sb.WriteString("tick")
		for _, j := range c.Joints {
			sb.WriteString("," + j.Name)
		}
		sb.WriteString("\n")
		for t, tau := range rep.Torques {
			sb.WriteString(strconv.Itoa(t))
			for _, v := range tau {
				sb.WriteString("," + strconv.FormatFloat(v, 'g', 12, 64))
			}
			sb.WriteString("\n")
		}
		if err := os.WriteFile(*csv, []byte(sb.String()), 0644); err != nil {
			fmt.Fprintf(os.Stderr, "robot check: %v\n", err)
			return 1
		}
	}
	if *annotate {
		m.Safety.MaxJointVelocity = &peakV
		m.Safety.MaxJointTorque = &peakT
		if err := WriteManifest(*clip+".gband.json", m); err != nil {
			fmt.Fprintf(os.Stderr, "robot check: %v\n", err)
			return 1
		}
		fmt.Printf("  annotated %s.gband.json safety: max_joint_velocity=%.4f rad/s max_joint_torque=%.3f N*m\n", *clip, peakV, peakT)
	}
	if !rep.Feasible {
		fmt.Println("INFEASIBLE: this motion exceeds the robot's datasheet envelope")
		return 2
	}
	fmt.Println("FEASIBLE: every tick is inside the datasheet position/velocity/torque envelope")
	return 0
}
