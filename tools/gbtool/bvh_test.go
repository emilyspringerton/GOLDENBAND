package main

import (
	"strings"
	"testing"
)

// A minimal but real 2-joint, 2-frame BVH fixture.
const testBVH = `HIERARCHY
ROOT Hips
{
	OFFSET 0.0 0.0 0.0
	CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
	JOINT Chest
	{
		OFFSET 0.0 5.0 0.0
		CHANNELS 3 Zrotation Xrotation Yrotation
		End Site
		{
			OFFSET 0.0 5.0 0.0
		}
	}
}
MOTION
Frames: 2
Frame Time: 0.016667
0.0 0.0 0.0 0.0 0.0 0.0 1.0 2.0 3.0
0.1 0.0 0.0 0.0 0.0 0.0 1.5 2.5 3.5
`

func TestParseBVH_Basic(t *testing.T) {
	clip, err := ParseBVH(strings.NewReader(testBVH))
	if err != nil {
		t.Fatalf("ParseBVH: %v", err)
	}
	if len(clip.Joints) != 2 {
		t.Fatalf("expected 2 joints (Hips, Chest — End Site has none), got %d: %+v", len(clip.Joints), clip.Joints)
	}
	if len(clip.ChannelIDs) != 9 {
		t.Fatalf("expected 9 channels (6 + 3), got %d: %v", len(clip.ChannelIDs), clip.ChannelIDs)
	}
	if len(clip.Frames) != 2 {
		t.Fatalf("expected 2 frames, got %d", len(clip.Frames))
	}
	if clip.Frames[0][6] != 1.0 || clip.Frames[0][7] != 2.0 || clip.Frames[0][8] != 3.0 {
		t.Errorf("frame 0 chest rotation values wrong: %v", clip.Frames[0][6:9])
	}
	if clip.Frames[1][0] != 0.1 {
		t.Errorf("frame 1 first value wrong: got %v want 0.1", clip.Frames[1][0])
	}
}

func TestParseBVH_TickRate(t *testing.T) {
	clip, err := ParseBVH(strings.NewReader(testBVH))
	if err != nil {
		t.Fatalf("ParseBVH: %v", err)
	}
	// 1/0.016667 ≈ 60.0 (a real BVH exported at 60fps)
	if tr := clip.TickRate(); tr != 60 {
		t.Errorf("TickRate() = %d, want 60", tr)
	}
}

func TestParseBVH_MismatchedFrameColumnsFails(t *testing.T) {
	bad := strings.Replace(testBVH, "0.0 0.0 0.0 0.0 0.0 0.0 1.0 2.0 3.0\n", "0.0 0.0 0.0\n", 1)
	_, err := ParseBVH(strings.NewReader(bad))
	if err == nil {
		t.Fatal("expected an error for a frame with the wrong number of columns")
	}
}

func TestParseBVH_NoMotionFails(t *testing.T) {
	noMotion := `HIERARCHY
ROOT Hips
{
	OFFSET 0.0 0.0 0.0
	CHANNELS 3 Xposition Yposition Zposition
}
`
	_, err := ParseBVH(strings.NewReader(noMotion))
	if err == nil {
		t.Fatal("expected an error for a file with no MOTION block")
	}
}
