# FYP project context

The FYP studies LiDAR place recognition and inter-robot data association. Historical KITTI Sequence 00 work uses Scan Context for high-recall geometric candidates, then tests BEV, RGB/OpenCLIP and temporal Cross-Max visual filtering/reranking. Cross-Max takes the strongest pairwise cosine similarity between query/candidate causal temporal windows rather than averaging embeddings. VLM was evaluated only as a semantic verifier and is excluded from the current CU-Multi main pipeline.

The documented formal split uses frame step 5, split 2500, gap 100, Robot A through frame 2400 and Robot B from frame 2600; positives have ground-truth x-z distance below 5 m. Ground truth is strictly offline for synchronization, overlap/positive definition, analysis and evaluation.

Existing layout: src/ contains retrieval, BEV/RGB/temporal, VLM, analysis and canonical-v2 code; outputs/ contains historical metrics and generated artefacts; data/, models/ and archive/ are private/large and ignored. README_zh.md is the detailed original report.

The next stage is CU-Multi Main Campus robot1/robot2. Its first milestone is a separate 100-keyframe-per-robot adapter/validation; it must not overwrite KITTI outputs/canonical_v2. Scan Context remains the candidate anchor. The historical frozen Top-20-to-Top-5 filtering fusion is 0.6*SC + 0.4*Cross-Max; 0.7 SC was a secondary ranking ablation.
