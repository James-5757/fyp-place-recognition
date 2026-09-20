# CU-Multi Stage 8.1: Canonical Scan Context Sanity Check

## Scope and result

This is a read-only reproducibility and scientific-positioning check for the
accepted Stage-8 commit `0b0812aa09e1eeed2a00e29644ed24362329c524`. It does
not create a new retrieval experiment, alter accepted Stage-8 numerical
outputs, or use ground truth during candidate generation.

The check is implemented in
`src/cumulti/run_stage8_1_canonical_sc_sanity.py`. It reads only the frozen
`scan_context_descriptors.npy` files and writes the compact reports in
`outputs/cumulti_v1/08_1_canonical_sc_sanity/`.

## Official reference checked

The reference is the public `gisbi-kim/scancontext` repository, inspected at
commit `93672835bb85e9c03fedf0fbaf2d0491df12161e`.

- `cpp/module/Scancontext/Scancontext.cpp`,
  `SCManager::makeScancontext` creates the polar ring-by-sector descriptor.
- `SCManager::makeRingkeyFromScancontext` (lines 198--211) constructs an
  `rows x 1` invariant key by calling `curr_row.mean()` for every descriptor
  row. This is the canonical row-wise mean Ring Key.
- `SCManager::makeAndSaveScancontextAndKeys` (lines 230--244) saves the Scan
  Context and Ring Key.
- `SCManager::detectLoopClosureID` (lines 247--312) first searches the Ring
  Key tree and then calls `distanceBtnScanContext` on each shortlist member.
- `cpp/module/Scancontext/Scancontext.h` sets `PC_NUM_RING = 20`,
  `PC_NUM_SECTOR = 60`, and `NUM_CANDIDATES_FROM_TREE = 10` (lines 79--87).

The official default of 10 tree candidates is a design choice for its original
loop-closure setting. It is not assumed to transfer to CU-Multi inter-robot
retrieval.

## Ring-Key equivalence

For each frozen Robot1, Robot2, and Robot3 descriptor tensor with shape
`[N, 20, 60]`, Stage 8 computes `desc.mean(axis=2)`, hence `[N, 20]`. The
Stage-8.1 script selects 100 deterministic indices per robot and independently
computes every one of the 20 entries using an explicit per-row loop and
`row.mean()`. The JSON report records maximum/mean absolute error and the
number of descriptors exceeding `1e-7`.

This is mathematically the same operation as the canonical row-wise mean.

## KD-tree and exact SC sanity

Stage 8 uses `scipy.spatial.cKDTree` over 20-D Ring Keys and its Euclidean L2
query metric. Stage 8.1 compares the KD-tree sets with stable brute-force L2
rankings for 100 deterministic Robot1 queries against Robot3 at M = 10, 100,
and 1000. Exact boundary-distance ties are reported separately; their ordering
is not treated as an algorithmic discrepancy if all strictly nearer candidates
are retained and all selected candidates are tied-or-nearer than the boundary.

Candidate generation uses descriptors/Ring Keys only: it does not load or use
GT, pose, heading, or RGB. In the existing Stage-8
`run_stage8_efficient_sc_retrieval.py::fixed_run`, each shared shortlist
candidate is asserted to have identical accelerated versus exhaustive full-SC
score (`np.allclose`) and `best_shift` (`np.array_equal`). The accepted Stage-8
run completed this assertion; Stage 8.1 therefore summarizes the existing
equivalence evidence without rerunning the full benchmark.

## Scientific positioning

The following are standard Scan Context/prior-work components, not claimed as
our novelty:

- a 20-D Ring Key formed by row-wise sector means;
- a KD-tree coarse candidate search; and
- exact full Scan Context comparison over the shortlisted candidates.

Our CU-Multi contribution is the evaluation protocol around this standard
retrieval structure: an exhaustive multi-robot reference, a controlled
M=10...1000 shortlist sweep, measured accuracy--latency trade-offs, a strict
selection rule, M=1000 selected under the predeclared <=0.1 percentage-point
loss criterion, held-out Robot2-to-Robot3 validation, and 2-/4-robot
compute-capacity projections.

Under that strict CU-Multi criterion, smaller M values lost too much
candidate-recall/accuracy; M=1000 was the smallest tested shortlist satisfying
the predeclared requirement. This does not suggest that the canonical default
M=10 is incorrect in its original loop-closure setting.
