// gbtool — GOLDEN BAND pipeline tool (HQ-SPEC-SIM-100 §8 build step 1).
//
// Usage:
//
//	gbtool import --bvh <file.bvh> --out <name> [--kind mocap|human|generative] [--who <text>]
//	gbtool bake-ease --out <name> --channel <name> --ticks <n> [--tick-rate <n>]
//	gbtool hash <name>
//	gbtool validate <name>
//
// <name> resolves to <name>.gband (binary) and <name>.gband.json (manifest).
package main

import (
	"encoding/hex"
	"flag"
	"fmt"
	"math"
	"os"
)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(1)
	}
	var code int
	switch os.Args[1] {
	case "import":
		code = runImport(os.Args[2:])
	case "bake-ease":
		code = runBakeEase(os.Args[2:])
	case "hash":
		code = runHash(os.Args[2:])
	case "validate":
		code = runValidate(os.Args[2:])
	default:
		fmt.Fprintf(os.Stderr, "gbtool: unknown command %q\n\n", os.Args[1])
		usage()
		code = 1
	}
	os.Exit(code)
}

func usage() {
	fmt.Fprintln(os.Stderr, `usage:
  gbtool import --bvh <file.bvh> --out <name> [--kind mocap|human|generative] [--who <text>]
  gbtool bake-ease --out <name> --channel <name> --ticks <n> [--tick-rate <n>]
  gbtool hash <name>
  gbtool validate <name>`)
}

// runBakeEase synthesizes a single-channel, procedurally-generated 0.0->1.0
// smoothstep (ease-in/ease-out) curve and writes it out as a real .gband +
// manifest pair, same on-disk shape as a BVH import. This is NOT a
// skeletal animation -- there is no real skeleton being sampled here, just
// a single scalar "blend_t" channel a caller resamples over its own real
// quantity (e.g. a hero's facing angle, lerped from its starting value to
// its target value using this curve as the interpolation weight each
// tick). skeleton_hash is the zeroed sentinel for exactly this reason
// (same convention gbtool import already uses for "no skeleton asset
// resolution yet," HQ-SPEC-SIM-100 v0 scope) -- flagged honestly as a
// real, deliberate repurposing of the sampler for a non-skeletal scalar
// curve, not a skeleton retargeting pass. Per HQ-SPEC-SIM-100 §3 ("no
// curves-with-twelve-interpolation-modes cleverness; resample at import"),
// the easing shape is baked into concrete per-tick float values here, at
// authoring time -- the runtime C sampler (src/gband.c) still does zero
// curve interpolation of its own, exactly as spec'd.
func runBakeEase(args []string) int {
	fs := flag.NewFlagSet("bake-ease", flag.ContinueOnError)
	out := fs.String("out", "", "output name (writes <name>.gband + <name>.gband.json)")
	channel := fs.String("channel", "blend_t", "channel name recorded in the manifest")
	ticks := fs.Int("ticks", 0, "duration in ticks (required)")
	tickRate := fs.Uint("tick-rate", 64, "ticks/second this clip is authored at")
	who := fs.String("who", "gbtool bake-ease (procedural smoothstep)", "authorship.who")
	if err := fs.Parse(args); err != nil {
		return 1
	}
	if *out == "" || *ticks <= 0 {
		fmt.Fprintln(os.Stderr, "bake-ease: --out and --ticks (> 0) are required")
		return 1
	}

	data := smoothstepCurve(*ticks)

	gband := &GBandFile{
		Version:       1,
		TickRate:      uint32(*tickRate),
		DurationTicks: uint32(*ticks),
		NumChannels:   1,
		Data:          data,
	}
	binPath := *out + ".gband"
	if err := gband.WriteFile(binPath); err != nil {
		fmt.Fprintf(os.Stderr, "bake-ease: writing %s: %v\n", binPath, err)
		return 1
	}

	manifest := &Manifest{
		GBandVersion:  1,
		SkeletonHash:  hex.EncodeToString(make([]byte, 32)), // no skeleton -- scalar curve, not a pose
		ContentHash:   hex.EncodeToString(gband.ContentHash[:]),
		TickRate:      gband.TickRate,
		DurationTicks: gband.DurationTicks,
		Channels:      []string{*channel},
		Authorship:    Authorship{Kind: "generative", Who: *who},
		IntentTags:    []string{"ease-in-out", "scalar-curve", "ui-timing"},
		LoopPoints:    LoopPoints{StartTick: 0, EndTick: gband.DurationTicks},
	}
	jsonPath := *out + ".gband.json"
	if err := WriteManifest(jsonPath, manifest); err != nil {
		fmt.Fprintf(os.Stderr, "bake-ease: writing %s: %v\n", jsonPath, err)
		return 1
	}

	fmt.Printf("baked %s -> %s + %s\n", *channel, binPath, jsonPath)
	fmt.Printf("  tick_rate=%d duration_ticks=%d\n", gband.TickRate, gband.DurationTicks)
	fmt.Printf("  content_hash=%s\n", hex.EncodeToString(gband.ContentHash[:]))
	return 0
}

// smoothstepCurve returns `ticks` samples of the 3t^2-2t^3 smoothstep curve
// evenly spaced across [0, 1] (t=0 at the first sample, t=1 at the last),
// clamped to [0, 1] against any float rounding at the endpoints. Split out
// from runBakeEase so the actual curve math has a real unit test.
func smoothstepCurve(ticks int) []float32 {
	data := make([]float32, ticks)
	for i := 0; i < ticks; i++ {
		t := 1.0
		if ticks > 1 {
			t = float64(i) / float64(ticks-1)
		}
		eased := t * t * (3.0 - 2.0*t)
		data[i] = float32(math.Max(0.0, math.Min(1.0, eased)))
	}
	return data
}

func runImport(args []string) int {
	fs := flag.NewFlagSet("import", flag.ContinueOnError)
	bvhPath := fs.String("bvh", "", "input BVH file")
	out := fs.String("out", "", "output name (writes <name>.gband + <name>.gband.json)")
	kind := fs.String("kind", "mocap", "authorship.kind: mocap|human|generative")
	who := fs.String("who", "", "authorship.who — labeled honestly")
	if err := fs.Parse(args); err != nil {
		return 1
	}
	if *bvhPath == "" || *out == "" {
		fmt.Fprintln(os.Stderr, "import: --bvh and --out are required")
		return 1
	}

	f, err := os.Open(*bvhPath)
	if err != nil {
		fmt.Fprintf(os.Stderr, "import: %v\n", err)
		return 1
	}
	defer f.Close()

	bvh, err := ParseBVH(f)
	if err != nil {
		fmt.Fprintf(os.Stderr, "import: parsing %s: %v\n", *bvhPath, err)
		return 1
	}

	numChannels := len(bvh.ChannelIDs)
	durationTicks := len(bvh.Frames)
	data := make([]float32, 0, numChannels*durationTicks)
	for _, frame := range bvh.Frames {
		for _, v := range frame {
			data = append(data, float32(v))
		}
	}

	gband := &GBandFile{
		Version:       1,
		TickRate:      bvh.TickRate(),
		DurationTicks: uint32(durationTicks),
		NumChannels:   uint32(numChannels),
		Data:          data,
	}
	binPath := *out + ".gband"
	if err := gband.WriteFile(binPath); err != nil {
		fmt.Fprintf(os.Stderr, "import: writing %s: %v\n", binPath, err)
		return 1
	}

	manifest := &Manifest{
		GBandVersion:  1,
		SkeletonHash:  hex.EncodeToString(make([]byte, 32)), // no skeleton asset resolution yet (v0 scope)
		ContentHash:   hex.EncodeToString(gband.ContentHash[:]),
		TickRate:      gband.TickRate,
		DurationTicks: gband.DurationTicks,
		Channels:      bvh.ChannelIDs,
		Authorship:    Authorship{Kind: *kind, Who: *who},
		IntentTags:    []string{},
		LoopPoints:    LoopPoints{StartTick: 0, EndTick: gband.DurationTicks},
	}
	jsonPath := *out + ".gband.json"
	if err := WriteManifest(jsonPath, manifest); err != nil {
		fmt.Fprintf(os.Stderr, "import: writing %s: %v\n", jsonPath, err)
		return 1
	}

	fmt.Printf("imported %s -> %s + %s\n", *bvhPath, binPath, jsonPath)
	fmt.Printf("  tick_rate=%d duration_ticks=%d num_channels=%d\n", gband.TickRate, gband.DurationTicks, gband.NumChannels)
	fmt.Printf("  content_hash=%s\n", hex.EncodeToString(gband.ContentHash[:]))
	return 0
}

func runHash(args []string) int {
	if len(args) != 1 {
		fmt.Fprintln(os.Stderr, "usage: gbtool hash <name>")
		return 1
	}
	name := args[0]
	g, err := ReadGBandFile(name + ".gband")
	if err != nil {
		fmt.Fprintf(os.Stderr, "hash: %v\n", err)
		return 1
	}
	stored := hex.EncodeToString(g.ContentHash[:])
	computedHash := g.ComputeContentHash()
	computed := hex.EncodeToString(computedHash[:])
	fmt.Printf("stored:   %s\n", stored)
	fmt.Printf("computed: %s\n", computed)
	if stored == computed {
		fmt.Println("match: OK")
		return 0
	}
	fmt.Println("MISMATCH")
	return 1
}

func runValidate(args []string) int {
	if len(args) != 1 {
		fmt.Fprintln(os.Stderr, "usage: gbtool validate <name>")
		return 1
	}
	name := args[0]

	g, err := ReadGBandFile(name + ".gband")
	if err != nil {
		fmt.Fprintf(os.Stderr, "validate: %v\n", err)
		return 1
	}
	m, err := ReadManifest(name + ".gband.json")
	if err != nil {
		fmt.Fprintf(os.Stderr, "validate: %v\n", err)
		return 1
	}

	problems := m.ValidateAgainst(g)
	if len(problems) == 0 {
		fmt.Printf("%s: OK — manifest and binary agree, content hash verified\n", name)
		return 0
	}
	fmt.Printf("%s: %d problem(s):\n", name, len(problems))
	for _, p := range problems {
		fmt.Printf("  - %s\n", p)
	}
	return 1
}
