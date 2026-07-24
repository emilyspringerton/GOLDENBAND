# GOLDENBAND Changelog

## 2026-07-24

- Initial build: HQ-SPEC-SIM-100 §8 build step 1. `.gband` binary format + manifest schema (`format/GBAND_FORMAT.md`), ~90-line C sampler (`src/gband.c`, `gb_init`/`gb_sample`/`gb_blend`/`gb_verify`), self-contained sha256 (trimmed from REDGARDEN's `hmac_sha256.h`, re-verified against NIST FIPS 180-4 vectors), and `gbtool` (Go: BVH import, hash, validate). 3 C tests + 10 Go tests, all passing; full end-to-end pipeline smoke-tested (synthetic BVH → `.gband` → validate → hash, all clean). EMILY/BACKLOG.md S144-01.
