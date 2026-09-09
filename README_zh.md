# 基于 Scan Context、时序视觉匹配与选择性 VLM 验证的 LiDAR Place Recognition

## 1. 项目概述

本项目以 KITTI Odometry Sequence 00 为实验数据，研究 LiDAR place recognition 在多机器人 formal split 设置下的候选检索、视觉重排序、时序跨帧匹配、候选过滤与 VLM 语义验证问题。

核心目标不是以视觉模型替代 Scan Context，而是建立一个保守的分层系统：

```text
LiDAR Scan Context 高召回候选生成
    -> Temporal RGB Cross-Max 视觉过滤/重排序
    -> （可选）VLM 语义验证
```

实验表明，Scan Context 提供强而稳定的几何候选；直接使用 BEV/RGB 或简单时序均值池化容易引入 regression；显式 temporal Cross-Max 能更好保留局部、视角兼容的视觉证据。VLM 在本项目当前设置中产生部分正确语义判断，但尚不适合自动覆盖 Scan Context。

## 2. 数据与固定实验协议

| 项目 | 设置 |
|---|---|
| 数据集 | KITTI Odometry Sequence 00 |
| 帧采样 | `frame_step = 5` |
| Robot A | sampled frame `<= 2400` |
| Robot B | sampled frame `>= 2600` |
| split frame | `2500` |
| boundary gap | `100` |
| positive 定义 | ground-truth x-z distance `< 5.0 m` |
| valid query | Robot B 中至少有一个 positive |
| valid queries | `158` |
| Scan Context | 20 rings, 60 sectors, max radius 80 m |
| RGB 相机 | KITTI `image_2`, 1241 x 376 RGB |
| OpenCLIP | `ViT-B-32-quickgelu` |
| 时间窗口 | causal `[t-20, t-15, t-10, t-5, t]` |

说明：现有 formal baseline 源码的 positive 判定是严格 `< 5.0 m`，交付包所有复现分析保持这一既有语义，未改为 `<= 5.0 m`。

## 3. 主要实验结果

### 3.1 Scan Context baseline

```text
R@1 = 151/158 = 0.955696
MRR = 0.963608
R@5 candidate-pool recall = 154/158 = 0.974684
```

### 3.2 单模态视觉重排序

| 方法 | R@1 | MRR | Corrections | Regressions | 结论 |
|---|---:|---:|---:|---:|---|
| Scan Context | 0.955696 | 0.963608 | 0 | 0 | 强基线 |
| yaw-aligned BEV only | 0.917722 | 0.942511 | 0 | 6 | 不适合单独重排序 |
| single-frame RGB only | 0.892405 | 0.922363 | 1 | 11 | 视角敏感 |
| temporal RGB-3 mean | 0.867089 | 0.912131 | 0 | 14 | 均值池化恶化 |
| temporal RGB-5 mean | 0.854430 | 0.907700 | 1 | 17 | 均值池化恶化 |
| temporal Cross-Max | 0.930380 | 0.948312 | 1 | 5 | 最佳 standalone visual score |
| temporal Cross-Top3 | 0.898734 | 0.932489 | 1 | 10 | 次于 Cross-Max |
| temporal Cross-Symmetric | 0.860759 | 0.912447 | 2 | 17 | 不稳定 |

### 3.3 Temporal Cross-Max

Cross-Max 不平均 temporal embedding。对于 query/candidate 的 causal temporal window，建立全部 frame-to-frame cosine similarity matrix，并取矩阵最大值：

```text
temporal_cross_max = max_{i,j} cosine(e_query_i, e_candidate_j)
```

该设计比 temporal mean pooling 更好，因为它保留了窗口中最兼容的局部视觉观察，而不是将具有不同视角/内容的 frame 强制平均。

### 3.4 Top-20 candidate recall ceiling

完整 Scan Context ranking 分析：

| K | queries with positive | Recall@K |
|---:|---:|---:|
| 1 | 151 | 0.955696 |
| 5 | 154 | 0.974684 |
| 10 | 156 | 0.987342 |
| 20 | 158 | 1.000000 |
| 30 | 158 | 1.000000 |
| 50 | 158 | 1.000000 |

四个原始 Top-5 miss 的 first-positive rank 分别为：

```text
Query 0115: Rank 15
Query 0380: Rank 10
Query 0955: Rank 12
Query 1565: Rank 7
```

因此当前实验的 candidate-retrieval ceiling 结论为：

```text
SC_TOP20_SUFFICIENT_FOR_HIGH_RECALL
```

### 3.5 Top-20 -> Top-5 visual filtering

使用冻结的 fusion：

```text
fusion = 0.6 * Scan Context score + 0.4 * Temporal Cross-Max
```

在 Top-20 pool 上重排后保留 Top-5：

```text
SC Top-20 pool: 158/158 positives = 100%
SC-only Top-5: 154/158 = 97.47%
SC + Cross-Max alpha=0.6 Top-5: 157/158 = 99.37%
Candidate reduction: 75%
Rescues: 3
Losses: 0
Net retention gain: +3
```

恢复的三项为 Query 115、955、1565；Query 380 仍为 filtered-pool miss。

**冻结方法说明：**本项目将 `alpha=0.6` 冻结为候选过滤的部署配置，因为它在 Top-20 -> Top-5 的主要目标上达到 `157/158` positive retention、3 rescues、0 losses。下面的 `alpha=0.7` 仅是同一数据上的 secondary Top-1/MRR ablation 最佳点，不替代冻结的 filtering 配置。

### 3.6 Top-20 SC + Cross-Max ranking

对于完整 Top-20 candidate pool，`SC + Cross-Max alpha=0.7` 的 secondary ranking metric：

```text
R@1 = 153/158 = 0.968354
MRR = 0.977321
Corrections = 2
Regressions = 0
Net gain = +2
```

该结果需要在其他 KITTI sequence 或独立验证 split 上进一步确认，不能直接宣称普适泛化。

### 3.7 VLM semantic verification

VLM 仅作为小候选集的语义 verifier，不用于全数据库检索。

第一轮 VLM pilot 表明：

```text
Query 1550: raw consistent winner = positive candidate 4540,
            but min confidence = 78 < 80,
            therefore RAW_CORRECT_THRESHOLD_ABSTAINED.
```

Selectively evaluated VLM v2 的部署策略采用固定规则：只有 AB/BA 映射到同一物理 challenger 且 minimum confidence >= 80 时，才自动 override Scan Context；GT 仅用于离线 correction/regression 评估。

最终 selective VLM v2：

```text
High-confidence corrections = 2
High-confidence regressions = 3
Net system gain = -1
System R@1 = 150/158 = 0.949367
```

结论：VLM 提供了部分有用 raw semantic signal，但目前自动 override 会引入过多 regressions。

```text
VLM_VERIFICATION_INTRODUCES_TOO_MANY_REGRESSIONS
```

## 4. 最终研究结论

1. Scan Context 是强且稳定的 geometric retrieval baseline。
2. Forward-facing RGB 的 single-frame 表示高度受 viewpoint 影响。
3. Temporal embedding mean pooling 会稀释辨识性视觉证据，效果低于 single frame。
4. Temporal Cross-Max 显式跨帧匹配优于 temporal mean pooling。
5. Scan Context Top-20 为 158 个 valid queries 提供 100% positive candidate recall。
6. 冻结的 SC + Cross-Max alpha=0.6 可将 Top-20 减少 75% 到 Top-5，并保持 157/158 positive retention、无 loss。
7. Top-20 SC + Cross-Max alpha=0.7 达到 153/158 Top-1 correct、2 corrections、0 regressions。
8. 当前 VLM 适合作为受控分析工具，但尚不适合自动覆盖 geometric anchor。

## 5. 核心文件说明

### Scan Context 与候选分析

- `src/scan_context.py`: 既有 Scan Context descriptor 与 cyclic-shift similarity。
- `src/retrieval_baseline_formal.py`: formal baseline 的既有实验脚本。
- `src/analyze_sc_candidate_recall.py`: 全 Robot-B ranking 与 Recall@K 分析。
- `results/sc_candidate_recall_analysis/`: candidate recall curve、Top-5 misses、Top-1 failures。

### BEV/RGB/Cross-Max 实验

- `src/generate_bev.py`: LiDAR -> pseudo-RGB BEV。
- `src/full_bev_reranking.py`: yaw-aligned BEV reranking。
- `src/full_rgb_reranking.py`: full single-frame RGB reranking。
- `src/full_temporal_rgb_reranking.py`: causal temporal mean-pooling。
- `src/full_temporal_cross_frame_reranking.py`: Cross-Max / Top3 / Symmetric matching。
- `src/top20_crossmax_visual_filtering.py`: frozen alpha=0.6 Top-20 -> Top-K filtering。
- `results/top20_visual_filtering/`: retention、rescue/loss、rank shift、ranking metrics。

### VLM verification

- `src/prepare_vlm_verification_cases.py`: 初始 temporal-panel preparation。
- `src/run_vlm_verification_pilot.py`: OpenAI-compatible VLM runner。
- `src/evaluate_vlm_verification_pilot.py`: 原 VLM pilot evaluator。
- `src/prepare_selective_vlm_v2.py`: filtered Top-5 selective VLM case preparation。
- `src/run_selective_vlm_v2.py`: selective VLM runner。
- `src/evaluate_selective_vlm_v2.py`: GT-independent deployment policy 与离线评估。
- `results/selective_vlm_v2/`: 最新 selective VLM evaluator summaries/tables。

### 汇报图

- `report_figures/method_r1_lollipop.png`: 方法 R@1 对比。
- `report_figures/viewpoint_crossframe_motivation.png`: forward-facing RGB 的大 viewpoint difference 设计依据。
- `report_figures/candidate_recall_ceiling_funnel.png`: candidate recall ceiling。
- `report_figures/candidate_recall_at_k.png`: Recall@K curve。
- `report_figures/top5_miss_gt_rank_lollipop.png`: 四个 Top-5 miss 的 first-positive rank。
- `report_figures/sc_to_crossmax_transition_matrix.png`: SC -> Top-20 SC+Cross-Max alpha=0.7 error transition。
- `report_figures/top5_miss_trajectory_cases.png`: 四个 retrieval miss 的 trajectory context。

## 6. 快速复现实验

服务器环境：

```bash
source /home/cas/miniconda3/etc/profile.d/conda.sh
conda activate fyp_slam
cd /home/cas/fyp_place_recognition
```

候选召回分析：

```bash
python src/analyze_sc_candidate_recall.py
```

Top-20 visual filtering：

```bash
python src/top20_crossmax_visual_filtering.py
```

注意：上面两个脚本默认拒绝覆盖现有 output。若要重新运行，请先复制或指定新的 output 目录，不建议直接使用 overwrite。

## 7. 未包含内容与安全说明

本交付包有意排除：

```text
- 原始 KITTI 数据和 RGB/LiDAR 数据集
- OpenCLIP checkpoint 与模型权重
- embedding caches
- VLM raw responses
- VLM collages
- API key、env 文件、Authorization 信息
- 任何 Git/SSH 私有凭据
```

使用 VLM runner 前需要在服务器内部 source 私有 env 文件，例如：

```bash
source /home/cas/.config/fyp/vlm.env
```

该 env 文件不得提交、打包、下载或在聊天中展示。

## 8. 推荐的后续工作

最有价值的下一步是用预先固定的独立 KITTI sequence 或新 split 验证：

```text
SC Top-20 proposal + SC/Cross-Max alpha=0.6 Top-5 filtering
SC Top-20 proposal + SC/Cross-Max alpha=0.7 Top-1 ranking
```

当前不建议直接扩大 VLM automatic override，因为 selective VLM v2 的 net system gain 为负。
