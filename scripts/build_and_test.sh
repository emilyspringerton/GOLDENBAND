#!/usr/bin/env bash
# Builds and runs GOLDEN BAND's C tests, then the Go pipeline tool's tests.
set -euo pipefail
cd "$(dirname "$0")/.."

echo "== sha256 core =="
gcc -Wall -Wextra -O2 -o /tmp/gb_test_sha256 tests/test_sha256.c
/tmp/gb_test_sha256

echo
echo "== gband sampler =="
gcc -Wall -Wextra -O2 -o /tmp/gb_test_gband tests/test_gband.c src/gband.c
/tmp/gb_test_gband

echo
echo "== gskel skeleton loader (S144-07) =="
gcc -Wall -Wextra -O2 -o /tmp/gb_test_gskel tests/test_gskel.c src/gskel.c
/tmp/gb_test_gskel

echo
echo "== gmesh skinned-mesh loader (S144-07) =="
gcc -Wall -Wextra -O2 -o /tmp/gb_test_gmesh tests/test_gmesh.c src/gmesh.c
/tmp/gb_test_gmesh

echo
echo "== gseq animation stitching/crossfade (S144-XX) =="
gcc -Wall -Wextra -O2 -o /tmp/gb_test_gseq tests/test_gseq.c src/gseq.c src/gband.c -lm
/tmp/gb_test_gseq

echo
echo "== gpose forward kinematics + skinning (S144-XX) =="
gcc -Wall -Wextra -O2 -o /tmp/gb_test_gpose tests/test_gpose.c src/gpose.c -lm
/tmp/gb_test_gpose

echo
echo "== gsync multi-actor frame synchronization (S144-XX) =="
gcc -Wall -Wextra -O2 -o /tmp/gb_test_gsync tests/test_gsync.c src/gsync.c
/tmp/gb_test_gsync

echo
echo "== grb rigid body physics (XPBD) =="
gcc -Wall -Wextra -O2 -o /tmp/gb_test_grb tests/test_grb.c src/grb.c -lm
/tmp/gb_test_grb

echo
echo "== gbtool (Go) =="
# GOWORK=off: this repo is intentionally standalone, not part of the
# monorepo's go.work (mirrors SHANKPIT/PITVIPER/EmilyOS's own convention —
# GOLDEN BAND assets "know nothing about SHANKPIT", per HQ-SPEC-SIM-100 §2).
(cd tools/gbtool && GOWORK=off go build ./... && GOWORK=off go vet ./... && GOWORK=off go test ./...)

echo
echo "== robot pipeline: datasheet robot -> physics -> RNEA cross-check -> reward compiler -> training =="
(cd tools/gbtool && GOWORK=off go build -o /tmp/gb_gbtool .)
GBT=/tmp/gb_gbtool
R=/tmp/gb_robot
mkdir -p "$R"
$GBT robot compile --robot robots/ur5e.grobot.json --out "$R/ur5e"
$GBT robot bake-motion --robot robots/ur5e.grobot.json --out "$R/hold" --keyframes "0:0,-60,45,-60,-90,0;1:0,-60,45,-60,-90,0"
$GBT robot check --robot robots/ur5e.grobot.json --clip "$R/hold" --csv "$R/hold.csv" >/dev/null
$GBT robot bake-motion --robot robots/ur5e.grobot.json --out "$R/wave" \
  --keyframes "0:0,-90,90,-90,-90,0;1.0:60,-60,40,-70,-90,30;2.0:-30,-100,110,-100,-60,-40;3.0:0,-90,90,-90,-90,0"
$GBT robot check --robot robots/ur5e.grobot.json --clip "$R/wave"
# The same 60-degree pan squeezed into 0.25 s must be rejected by the datasheet speed limit.
$GBT robot bake-motion --robot robots/ur5e.grobot.json --out "$R/too_fast" --keyframes "0:0,-90,90,-90,-90,0;0.25:60,-90,90,-90,-90,0" >/dev/null
if $GBT robot check --robot robots/ur5e.grobot.json --clip "$R/too_fast" >/dev/null; then
  echo "FAIL: an over-speed clip passed the feasibility check"; exit 1
else
  echo "PASS: over-speed clip rejected by the feasibility check (exit $?)"
fi
gcc -Wall -Wextra -O2 -o /tmp/gb_test_grobot tests/test_grobot.c src/grobot.c src/grb.c -lm
/tmp/gb_test_grobot "$R/ur5e.grobot" "$R/hold.csv"
gcc -Wall -Wextra -O2 -o /tmp/gb_test_grl tests/test_grl.c src/grl.c src/grobot.c src/grb.c src/gband.c -lm
/tmp/gb_test_grl "$R/ur5e.grobot" "$R/wave"
gcc -Wall -Wextra -O2 -o /tmp/gb_gbtrain tools/gbtrain/gbtrain.c src/grl.c src/grobot.c src/grb.c src/gband.c -lm
# Short training run: must beat the servo-only baseline on the frozen eval (gbtrain exits 3
# otherwise) and be bit-for-bit reproducible from its seed.
/tmp/gb_gbtrain --robot "$R/ur5e.grobot" --clip "$R/wave" --out "$R/ci_a" --iters 6 --pop 12 --elites 4 | tail -14
/tmp/gb_gbtrain --robot "$R/ur5e.grobot" --clip "$R/wave" --out "$R/ci_b" --iters 6 --pop 12 --elites 4 >/dev/null
cmp "$R/ci_a.gpolicy" "$R/ci_b.gpolicy" && echo "PASS: training is deterministic for a fixed seed"
$GBT validate "$R/ci_a_rollout"
