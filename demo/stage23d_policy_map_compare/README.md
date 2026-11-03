# Stage23D frozen P0/P5 final-map inspector

Serve the **parent** `demo/` directory, preserving the pinned Three.js r170
vendor URLs. From repository root: `bash scripts/run_stage23d_demo.sh --serve`.
Open `http://127.0.0.1:8024/stage23d_policy_map_compare/`.

Windows, after copying the new assets and the existing vendor directory:
`python -m http.server 8024 --bind 127.0.0.1 --directory E:/university/uic/FYP/demo`

Two synchronized cameras show real frozen final P0 and P5 maps. Switch between
their existing validated incremental/Batch trajectories; inspect shared-keyframe
trajectory overlays, exact loop identities, seven temporal groups and four
odometry-bin pairs. The white region markers are selected original poses.

Clouds are deterministic capped subsets (25,000/source) of the full 0.5m voxel
maps. Server-side metrics use full geometry. Maps retain original metric XYZ,
including tilted gauge Z; no GT transform, flattening, rescaling or cosmetic
cloud movement. NN and policy differences are diagnostics, not absolute errors.
Offline GT metrics are explicitly separate. The historical P5 quality failure
is not reversed by its lower ATE on this route.

PROGRESSIVE_STATE_EXPORT_REQUIRED: final states only. No fake replay/morph.
Separate recordings, hypothetical dual-stream replay, centralized backend,
protocol-simulated application bytes; no live robots or measured network.

On load failure the page reports the exact path/HTTP error. Map SHA256/lengths
are checked where browser cryptography is available. GPU buffers are cached
per frozen state, with disposal on page exit. Frozen Stage18–23C demos/results
are untouched. See docs/CUMULTI_STAGE23D.md for validation and limitations.
