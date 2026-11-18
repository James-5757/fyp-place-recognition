# Stage32R historical runtime restoration

**BLOCKED_ORIGINAL_RUNTIME_NOT_RECOVERED.** The expanded accessible search recovered no matching original System.cc/System.h, libORB_SLAM3.so or rgbd_full. Verified inputs and the original camera/build recipe remain available. Instrumented SDK copying, building and replay did not start.

## Search coverage

| Machine | Roots | Directories visited | Files visited | Archives inspected | Git repository/bundle entries |
|---|---|---:|---:|---:|---:|
| server | / | 50,989 | 414,734 | 108 / 111 | 4 |
| local | C:/, D:/, E:/ | 355,853 | 2,262,668 | 1622 / 1790 | 35 |

The server scan includes accessible home/workspace/backup/build/cache/install directories across the guest root. Virtual /proc, /sys, /dev, /run and OS snap application mounts are excluded from recursive file walking. Live process executables and ORB library mappings are inspected separately for recoverable deleted files. Git worktrees, named historical objects, reflog-reachable objects, all object-store blobs at the historical System/header/library sizes, and discovered Git bundles are checked. Compressed archive members are hashed in memory without unpacking unrelated files. Additional renamed ELF executables under one megabyte and files at known historical sizes are checked. The complete runtime search is broader than a filename-only SDK-path check.

Windows C/D/E roots were searched with junction pruning to avoid repeated traversal of aliases into already available drive trees. The scanner retains access errors and archive/Git limitations. This is an accessible search under the current accounts.

Docker socket/storage and /root are inaccessible to cas. Docker reports permission denied. No saved snap snapshots were reported. No mounted guest snapshot containing the runtime was located. Provider-side snapshots are not exposed through this account. Archive failures and unsupported formats are listed in search_summary.json and were not counted as inspected.

## Required identities

| Dependency | Status | Original identity verified |
|---|---|---|
| Tracking.cc | RECOVERED_AND_HASH_VERIFIED | yes |
| Frame.cc | RECOVERED_AND_HASH_VERIFIED | yes |
| ORBextractor.cc | RECOVERED_AND_HASH_VERIFIED | yes |
| Tracking.h | RECOVERED_AND_HASH_VERIFIED | yes |
| System.cc | RECOVERED_BUT_HASH_MISMATCH | no |
| System.h | RECOVERED_BUT_HASH_MISMATCH | no |
| libORB_SLAM3.so | NOT_FOUND | no |
| rgbd_full | NOT_FOUND | no |
| ORBvoc.txt | RECOVERED_AND_HASH_VERIFIED | yes |
| camera.yaml | RECOVERED_AND_HASH_VERIFIED | yes |
| build_rgbd_full.sh | RECOVERED_AND_HASH_VERIFIED | yes |
| sequence.txt | RECONSTRUCTED_NOT_ORIGINAL | no |

Source files matching the old receipts and the vocabulary include Stage32 recovered copies and/or their archive members. Public System variants are available but fail the historical hashes. They are candidates rather than restorations of the modified SDK. The exact historical hashes and every candidate location are recorded in stage32r_hash_verification.json and the inventory CSV.

The camera YAML is authenticated by the frozen project Git blob, and the native build script is compared to its historical SHA receipt. The reconstructed sequence has the same content hash as the prior Stage32 reconstruction. No original sequence-file receipt was found. Its classification remains RECONSTRUCTED_NOT_ORIGINAL. Input order can be reconstructed from the frozen sync rows, while original sequence identity remains unverified.

## Build environment and ABI

Current native OpenCV matches the recorded 4.5.4. Current compiler version, target triple, libstdc++ ABI macros, glibc/loader versions and shared-library inventory are recorded. The original build recipe uses C++17, O2, the fixed preprocessor flags and the ORB/DBoW2/Pangolin rpaths shown in the environment JSON. Original compiler exact version, actual ELF NEEDED/RUNPATH entries and original C++ ABI cannot be authenticated while the matching executable/library are missing. Present host compatibility alone does not establish the original runtime environment.

Camera settings remain 640×400, RGBD.DepthMapFactor 1000, 1000 ORB features, scale factor 1.2, eight levels and FAST 20/7. No package installation, build-parameter change or tracking invocation was performed.

## Unreproduced modified System behavior

The original TrackRGBD caller requires the System constructor, TrackRGBD gateway, tracking-state/MapPoint accessors and Shutdown. Without the original modified System implementation/header, their exact initialization, reset/map-control, entry/exit behavior and class ABI cannot be verified. The patch may concern export, tracking-relevant behavior, or both. The available hashes identify a difference but reveal no modification semantics. Inferring an export-only change from the public code would be unsupported.

## Minimum files needed to unblock

- Original System.cc, SHA256 11aa821b22ba90cadcc1e7290dda3c5eadcfa3cd83f36e30fb9f5c0c4378027c.
- Original System.h, SHA256 7d2eb3734a7a81110274683ea36dc0ba1ce14204d3d5f081bf950601ff13fab3.
- Original rgbd_full, SHA256 4833b5c64656d62549f26c3494ccd5e96bf724e6acfb173ea0450525b9c1728f.
- Original libORB_SLAM3.so, SHA256 ddb134dca4a630c702af5cf4d5acc0cd4d6159433e78a09884d27163605e8252.

An authenticated original SDK backup or its original patch is preferable because it retains the surrounding build/dependency context. An original sequence.txt or independently recorded original checksum is additionally needed before authenticating the exact sequence file. An administrator can inspect the inaccessible backup/container/provider snapshot locations if such backups exist. No further camera dataset download is required.

## Recovery gate and preservation

Exact reproduction and readiness for an instrumented build are false. The user stop condition is reached because the modified sources and original executable were not recovered. Stage24–Stage32 scientific artifacts were hash-checked and preserved. This checkpoint records restoration evidence and blockers, and does not claim completion of native tracking forensics.

## Reproduce

Run stage32r_search.py on the server root and Windows drive roots with separate output/work directories, then run stage32r_environment.py on the server. Place the server/local search JSONs and server_environment.json under the Stage32R output directory. stage32r_finalize.py --work <separate scratch directory> merges the evidence and enforces the critical-original identity gate. Original files and archive members are read-only; matching recovered copies and Git bundle mirrors go into scratch.
