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
echo "== gbtool (Go) =="
# GOWORK=off: this repo is intentionally standalone, not part of the
# monorepo's go.work (mirrors SHANKPIT/PITVIPER/EmilyOS's own convention —
# GOLDEN BAND assets "know nothing about SHANKPIT", per HQ-SPEC-SIM-100 §2).
(cd tools/gbtool && GOWORK=off go build ./... && GOWORK=off go vet ./... && GOWORK=off go test ./...)
