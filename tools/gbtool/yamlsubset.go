// yamlsubset.go — a deliberately tiny YAML reader for exactly the shape of the robot
// manufacturer description files gbtool imports (Universal Robots' own
// Universal_Robots_ROS2_Description config/*.yaml): nested block mappings of scalars, `#`
// comments, and the `!degrees` tag those files use for joint limits. No sequences, flow style,
// anchors, or multi-line scalars -- anything outside that subset is a hard parse error, never a
// silent misread. Kept in-house so gbtool stays a stdlib-only module (same reason gltf.go is).
package main

import (
	"fmt"
	"math"
	"strconv"
	"strings"
)

type yamlMap map[string]any

func parseYAMLSubset(src string) (yamlMap, error) {
	root := yamlMap{}
	type frame struct {
		indent int
		m      yamlMap
	}
	stack := []frame{{-1, root}}
	var pendingKey string
	var pendingIndent = -1
	for lineno, raw := range strings.Split(src, "\n") {
		line := raw
		if i := strings.Index(line, "#"); i >= 0 {
			line = line[:i]
		}
		if strings.TrimSpace(line) == "" {
			continue
		}
		if strings.Contains(line, "\t") {
			return nil, fmt.Errorf("yaml line %d: tabs are not supported", lineno+1)
		}
		indent := len(line) - len(strings.TrimLeft(line, " "))
		body := strings.TrimSpace(line)
		if strings.HasPrefix(body, "- ") || body == "-" || strings.HasPrefix(body, "{") || strings.HasPrefix(body, "[") {
			return nil, fmt.Errorf("yaml line %d: sequences/flow style are outside the supported subset", lineno+1)
		}
		colon := strings.Index(body, ":")
		if colon <= 0 {
			return nil, fmt.Errorf("yaml line %d: expected `key: value`, got %q", lineno+1, body)
		}
		key := strings.TrimSpace(body[:colon])
		val := strings.TrimSpace(body[colon+1:])

		if pendingKey != "" {
			// The previous key had no inline value: this line must open its nested mapping.
			parent := stack[len(stack)-1].m
			if indent <= pendingIndent {
				return nil, fmt.Errorf("yaml line %d: key %q has no value", lineno+1, pendingKey)
			}
			child := yamlMap{}
			parent[pendingKey] = child
			stack = append(stack, frame{indent, child})
			pendingKey = ""
		}
		for len(stack) > 1 && indent < stack[len(stack)-1].indent {
			stack = stack[:len(stack)-1]
		}
		top := stack[len(stack)-1]
		if indent != top.indent && top.indent != -1 {
			return nil, fmt.Errorf("yaml line %d: inconsistent indentation", lineno+1)
		}
		if top.indent == -1 {
			stack[len(stack)-1].indent = indent
		}
		if val == "" {
			pendingKey, pendingIndent = key, indent
			continue
		}
		v, err := yamlScalar(val)
		if err != nil {
			return nil, fmt.Errorf("yaml line %d: %w", lineno+1, err)
		}
		stack[len(stack)-1].m[key] = v
	}
	if pendingKey != "" {
		stack[len(stack)-1].m[pendingKey] = nil
	}
	return root, nil
}

// yamlScalar converts a scalar: `!degrees N` -> radians (float64), numbers -> float64,
// true/false -> bool, anything else -> string.
func yamlScalar(s string) (any, error) {
	if strings.HasPrefix(s, "!degrees") {
		f, err := strconv.ParseFloat(strings.TrimSpace(strings.TrimPrefix(s, "!degrees")), 64)
		if err != nil {
			return nil, fmt.Errorf("bad !degrees value %q", s)
		}
		return f * math.Pi / 180, nil
	}
	if strings.HasPrefix(s, "!") {
		return nil, fmt.Errorf("unsupported tag in %q", s)
	}
	if f, err := strconv.ParseFloat(s, 64); err == nil {
		return f, nil
	}
	switch s {
	case "true":
		return true, nil
	case "false":
		return false, nil
	}
	return strings.Trim(s, `"'`), nil
}

// yamlFloat walks a dotted path ("inertia_parameters.tensor.shoulder.ixx") to a number.
func yamlFloat(m yamlMap, path string) (float64, error) {
	var cur any = m
	for _, part := range strings.Split(path, ".") {
		mm, ok := cur.(yamlMap)
		if !ok {
			return 0, fmt.Errorf("%s: %q is not a mapping", path, part)
		}
		cur, ok = mm[part]
		if !ok {
			return 0, fmt.Errorf("%s: missing key %q", path, part)
		}
	}
	f, ok := cur.(float64)
	if !ok {
		return 0, fmt.Errorf("%s: not a number (%v)", path, cur)
	}
	return f, nil
}
