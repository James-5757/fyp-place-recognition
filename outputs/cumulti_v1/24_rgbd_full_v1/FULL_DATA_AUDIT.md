# Full RGB-D data audit

This gate qualifies complete available inputs, not continuous tracking or successful fusion.

- robot1: 9997 RGB, 8765 depth clouds, 999.789161 recorded seconds; paired 8765/9997 within 25 ms.
- robot3: 20977 RGB, 19918 depth clouds, 2098.463330 recorded seconds; paired 19918/20977 within 25 ms.

Each camera start is independently normalized to zero. Side-by-side replay does not establish simultaneous physical recording.
A shorter recording ends explicitly; no frames or poses are fabricated to reach the Stage20 duration.
Every archive DB was read to EOF with ZIP CRC validation. All recorded CameraInfo/extrinsics messages were inspected.
Source header timestamps and bag timestamps are both preserved in the per-robot CSVs.
Calibration is unchanged; the established depth-to-color projection and 1000-feature baseline are retained.
