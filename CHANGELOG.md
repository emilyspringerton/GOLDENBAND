# GOLDENBAND Changelog

## 2026-08-05
- fix(tools): finish + verify export_gband_rig.py against real Blender 4.2.23 - completed export_animation (was referenced but unwritten), fixed 3 real bugs found via actual testing, full pipeline verified end-to-end through the real C loader (sess-20260723-2347-df115bd5)
- feat(ci): headless Blender Tools workflow (blender-tools.yml) - founder disk/script-constrained on Windows, so armature-building and asset export both run on GitHub Actions instead of locally; push-triggered gbtool half verified live via a real CI run (RESULT: success), workflow_dispatch armature-build half not yet triggered (sess-20260723-2347-df115bd5)

- feat: S144-07 .gskel/.gmesh formats + C loaders + Blender armature template/exporter (unrun, no Blender in this environment - see export script's own axis-conversion caveat) (sess-20260723-2347-df115bd5)


## 2026-07-24

- Initial build: HQ-SPEC-SIM-100 §8 build step 1. `.gband` binary format + manifest schema (`format/GBAND_FORMAT.md`), ~90-line C sampler (`src/gband.c`, `gb_init`/`gb_sample`/`gb_blend`/`gb_verify`), self-contained sha256 (trimmed from REDGARDEN's `hmac_sha256.h`, re-verified against NIST FIPS 180-4 vectors), and `gbtool` (Go: BVH import, hash, validate). 3 C tests + 10 Go tests, all passing; full end-to-end pipeline smoke-tested (synthetic BVH → `.gband` → validate → hash, all clean). EMILY/BACKLOG.md S144-01.
