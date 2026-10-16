# Stage20: availability-gated dual-stream replay

Serve the **parent demo directory**, not this subdirectory (shared pinned modules and Robot1 JPEGs use sibling relative paths):

```powershell
python -m http.server 8021 --bind 127.0.0.1 --directory "E:/university/uic/FYP/demo"
```

Open http://127.0.0.1:8021/stage20_dual_stream/ . On the server, serve `/home/cas/fyp_place_recognition/demo` on loopback and use an SSH tunnel. No public service/security-group change is required.

Both robots start at normalized zero, independently grow from R1 KF150 / R3 KF234, and preserve their original intervals. This is hypothetical synchronized replay of **separate recordings**, not live simultaneous robots. The first archived common-place constraint is ID45 R1 KF283 ↔ R3 KF252, released at 66.501611950 s when the arrived prefixes are R1 KF283 / R3 KF367. Do not confuse the historical match endpoint with the current Robot3 marker.

Three persistent panels keep local coordinates immutable. Before the bridge, the global panel is UNALIGNED. Afterward, it uses actual Stage20 iSAM2 states. Global maps/poses are held between saved snapshots (every loop event, periodic 60-second checkpoints and named milestones). The displayed global state time/prefix is explicit. No pose, map or loop interpolation. Local markers step through every arrived keyframe independently.

Play/Pause/Reset/Seek, 0.5x/1x/2x/4x, camera follow/top/perspective, panel/provenance/trajectory/loop toggles and twelve checkpoint jumps are provided. The base compression is 10 recorded seconds per demonstration second; the speed multiplier does not change stream timing ratios. R1 completes before the late R3 constraints become available. K thresholds may be crossed by multiple loops in one event: the displayed **actual K** is authoritative (e.g. K10 checkpoint can contain 11 loops).

R1 reuses Stage19 authentic forward images. R3 adds a bounded approximately 1 Hz stream, not a 10 Hz video. Images use each robot's own original bag/header timestamps; the last exported frame at/before that sensor time is shown. Missing images are explicit. JPEG decoding is on demand with two in flight, six queued and 24 cached images; global JSON state cache is bounded to three. Browser point clouds are deterministic subsets. RGB is not an optimizer input.

The 75 loop factors are precomputed frozen SC/GICP results, released only after both endpoints arrive. This does **not** implement causally online retrieval, retrospective search, distributed consensus or real network communication. Structural backend readiness does not imply numerical batch agreement or physical position accuracy. Historical numerical caveats remain visible. Gauge XYZ is not guaranteed geographic up; no vertical exaggeration or flattening is applied.

Reproduce backend using `scripts/run_stage20_dual_stream.sh` with a new output directory. `--display-extract` on the Python runner performs the identical update sequence with extra map snapshots and no new batch solves. Package with `src/demo/prepare_stage20_assets.py`; the exact-state comparison against the primary run must pass first. RGB exporter `src/demo/export_stage20_rgb.py` fsyncs count/byte/quality policy, measures the first ten recorded seconds, gates full extraction, and reads only a temporary SQLite expansion of the raw ZIP. Historical outputs are never overwritten.
