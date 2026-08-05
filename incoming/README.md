# incoming/

Drop your exported `.blend` (mesh + Armature modifier, auto-weighted to TylerRig) or `.bvh`
(exported animation actions) files here via GitHub's website ("Add file" -> "Upload files") and
commit. Pushing here automatically triggers `.github/workflows/blender-tools.yml`'s
`export-incoming` job, which runs `export_gband_rig.py`/`gbtool import` for you and uploads the
results as a downloadable Actions artifact -- no local script-running needed.

This file exists only so the (otherwise empty) directory is tracked by git.
