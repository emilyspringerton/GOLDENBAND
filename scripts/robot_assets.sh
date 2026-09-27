#!/usr/bin/env bash
# Regenerates assets/robots/ from robots/*.grobot.json (manufacturer-sourced specs):
#   <model>.grobot / <model>.gskel      compiled rigs for UR3e, UR5e, UR10e
#   ur5e_wave.gband(.json)              an authored reference motion (minimum-jerk keyframes),
#                                       feasibility-annotated against the UR5e datasheet
#   ur5e_wave_policy.gpolicy(.json)     a policy trained to make the physical UR5e honor it
#   ur5e_wave_policy_rollout.gband(.json)  what that policy actually achieved in physics
# Deterministic: rerunning produces byte-identical files.
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=assets/robots
mkdir -p "$OUT"
(cd tools/gbtool && GOWORK=off go build -o /tmp/gb_gbtool .)
gcc -Wall -Wextra -O2 -o /tmp/gb_gbtrain tools/gbtrain/gbtrain.c src/grl.c src/grobot.c src/grb.c src/gband.c -lm
for m in ur3e ur5e ur10e; do
  /tmp/gb_gbtool robot compile --robot robots/$m.grobot.json --out "$OUT/$m"
done
/tmp/gb_gbtool robot bake-motion --robot robots/ur5e.grobot.json --out "$OUT/ur5e_wave" --tags gesture,showpiece \
  --who "authored keyframes (scripts/robot_assets.sh), minimum-jerk interpolation" \
  --keyframes "0:0,-90,90,-90,-90,0;1.0:60,-60,40,-70,-90,30;2.0:-30,-100,110,-100,-60,-40;3.0:0,-90,90,-90,-90,0"
/tmp/gb_gbtool robot check --robot robots/ur5e.grobot.json --clip "$OUT/ur5e_wave" --annotate
( cd "$OUT" && /tmp/gb_gbtrain --robot ur5e.grobot --clip ur5e_wave --out ur5e_wave_policy --iters 40 --pop 32 --elites 6 --seed 1 )
/tmp/gb_gbtool robot check --robot robots/ur5e.grobot.json --clip "$OUT/ur5e_wave_policy_rollout"
