// bvh.go — a minimal BVH (Biovision Hierarchy) importer.
//
// BVH is a plain-text mocap format: a HIERARCHY block declares joints (each
// with an OFFSET and a CHANNELS list), followed by a MOTION block with a
// frame count, frame time, and one line of space-separated floats per
// frame (columns in the exact order channels were declared, depth-first).
//
// This reads the subset every mocap/animation tool actually emits — ROOT,
// JOINT, End Site, OFFSET, CHANNELS, MOTION, Frames:, Frame Time: — and
// resamples nothing (BVH's frame rate becomes the .gband tick_rate
// directly, since GOLDEN BAND already requires uniform sampling and BVH
// motion data already is uniform). glTF import is explicitly out of scope
// for this pass — see format/GBAND_FORMAT.md's "what v0 does not cover."
package main

import (
	"bufio"
	"fmt"
	"io"
	"strconv"
	"strings"
)

// BVHJoint is one node in the parsed hierarchy, in declaration order.
type BVHJoint struct {
	Name     string
	Channels []string // e.g. "Xposition", "Zrotation" — raw BVH channel names
}

// BVHClip is a fully parsed BVH file: the flattened channel list (in
// column order) and one []float64 row per frame.
type BVHClip struct {
	Joints     []BVHJoint
	ChannelIDs []string // "<joint>.<channel>", same order as each frame's columns
	FrameTime  float64  // seconds/frame
	Frames     [][]float64
}

// ParseBVH parses a BVH file from r.
func ParseBVH(r io.Reader) (*BVHClip, error) {
	sc := bufio.NewScanner(r)
	sc.Buffer(make([]byte, 0, 64*1024), 1024*1024)

	clip := &BVHClip{}
	inHierarchy := false
	inMotion := false
	var jointStack []string

	for sc.Scan() {
		line := strings.TrimSpace(sc.Text())
		if line == "" {
			continue
		}
		fields := strings.Fields(line)

		switch {
		case line == "HIERARCHY":
			inHierarchy = true
		case line == "MOTION":
			inHierarchy = false
			inMotion = true
		case inHierarchy && (fields[0] == "ROOT" || fields[0] == "JOINT"):
			name := fields[1]
			if len(jointStack) > 0 {
				name = jointStack[len(jointStack)-1] + "/" + name
			}
			jointStack = append(jointStack, name)
			clip.Joints = append(clip.Joints, BVHJoint{Name: name})
		case inHierarchy && fields[0] == "End" && len(fields) > 1 && fields[1] == "Site":
			jointStack = append(jointStack, "") // placeholder popped at the matching "}", no channels
		case inHierarchy && line == "}":
			if len(jointStack) > 0 {
				jointStack = jointStack[:len(jointStack)-1]
			}
		case inHierarchy && fields[0] == "CHANNELS":
			n, err := strconv.Atoi(fields[1])
			if err != nil {
				return nil, fmt.Errorf("bad CHANNELS count %q: %w", fields[1], err)
			}
			if len(clip.Joints) == 0 {
				return nil, fmt.Errorf("CHANNELS line with no active joint")
			}
			joint := &clip.Joints[len(clip.Joints)-1]
			joint.Channels = fields[2 : 2+n]
			for _, ch := range joint.Channels {
				clip.ChannelIDs = append(clip.ChannelIDs, joint.Name+"."+ch)
			}
		case inMotion && fields[0] == "Frames:":
			// Frame count is implied by how many data lines follow; not
			// stored separately, just validated at the end.
		case inMotion && fields[0] == "Frame" && len(fields) > 1 && fields[1] == "Time:":
			ft, err := strconv.ParseFloat(fields[2], 64)
			if err != nil {
				return nil, fmt.Errorf("bad Frame Time %q: %w", fields[2], err)
			}
			clip.FrameTime = ft
		case inMotion:
			row := make([]float64, len(fields))
			for i, f := range fields {
				v, err := strconv.ParseFloat(f, 64)
				if err != nil {
					return nil, fmt.Errorf("bad motion value %q on frame %d: %w", f, len(clip.Frames), err)
				}
				row[i] = v
			}
			if len(row) != len(clip.ChannelIDs) {
				return nil, fmt.Errorf("frame %d has %d values, expected %d (channel count)", len(clip.Frames), len(row), len(clip.ChannelIDs))
			}
			clip.Frames = append(clip.Frames, row)
		}
	}
	if err := sc.Err(); err != nil {
		return nil, err
	}
	if len(clip.Frames) == 0 {
		return nil, fmt.Errorf("no MOTION frames found")
	}
	if clip.FrameTime <= 0 {
		return nil, fmt.Errorf("missing or invalid Frame Time")
	}
	return clip, nil
}

// TickRate derives an integer tick rate from BVH's frame time (1/frame_time,
// rounded — BVH frame times are almost always exact reciprocals of a round
// rate like 30, 60, 120 already).
func (c *BVHClip) TickRate() uint32 {
	return uint32(1.0/c.FrameTime + 0.5)
}
