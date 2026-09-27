# Final Demo Architecture

## Current implementation: centralized backend

The FYP Demo V1 is a **CU-Multi Offline Replay**, not a live deployed robot system. `RobotAgent` instances retain local odometry, LiDAR/keyframe timelines, descriptor metadata, and display state. The `CentralCoordinator` owns the global descriptor database, cross-robot candidate events, GICP verification events, sanitized loop constraints, native GTSAM state, and global trajectories/maps. Global references are always `(robot_id, keyframe_id)`, so the current robot1/robot3 replay is not an R1/R3-specific software design.

The explicit local message interfaces are `RingKeyMessage`, `CandidateRequest`, `DescriptorResponse`, `PointCloudRequest`, `LoopConstraintMessage`, and `OptimizationUpdate`. They are presentation-safe local Python/JavaScript objects in Demo V1; they do not implement a network protocol or measured network traffic. The monitor emphasizes that raw LiDAR is not continuously sent.

Central server responsibilities are global descriptor search, cross-robot retrieval, GICP, loop management, GTSAM global PGO, and merged-map presentation. Robots contribute local odometry, local LiDAR/keyframes, and compact descriptor generation.

## Future extension: decentralized peer-to-peer exchange

Future work could move descriptor databases to robots, use peer-to-peer Ring Key exchange and candidate negotiation, request descriptors/clouds on demand, share inter-robot transforms, and investigate distributed PGO. This is an architectural boundary only: decentralized transport and optimization were not implemented or experimentally claimed.

The presentation is inspired by the architectural roles commonly visualized by COVINS/COVINS-G (central server backend), Swarm-SLAM (decentralized systems with optional base station), Kimera-Multi (local maps plus inter-robot closures), and DCL-SLAM (multi-robot LiDAR replay). This demo uses original visuals and no copied figure/UI assets.

**Current FYP claim:** the centralized backend path is implemented and validated through frozen offline CU-Multi artifacts. It is not real-time SLAM, a live robot deployment, ROS2 networking, or distributed PGO.
