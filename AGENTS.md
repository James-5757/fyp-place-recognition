# FYP repository instructions

1. Read PROJECT_CONTEXT.md, RESULTS.md, EXPERIMENT_LOG.md, then NEXT_STEPS.md before changing the project.
2. Inspect existing code before changing architecture, metrics or data conventions.
3. Preserve reproducibility and never overwrite historical outputs; use a new named output directory.
4. Do not commit raw datasets, model checkpoints, credentials, API keys, caches or large generated files.
5. Ground truth is only for synchronization, overlap definition, analysis and evaluation. Never select or rerank candidates with it.
6. Do not change Scan Context, OpenCLIP, Cross-Max, VLM or GICP core algorithms without an explicit isolated experiment.
7. Update EXPERIMENT_LOG.md after meaningful experiments, RESULTS.md for verified metrics, and NEXT_STEPS.md for direction changes.
