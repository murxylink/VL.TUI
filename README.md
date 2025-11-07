# TouchTag

TouchTag is a toolkit for reliable tangible tag tracking on IR multi-touch tables, with optional FilterID integration.

It provides:

- Robust tag recognition from IR touch points (4–6 “legs” per tag)
- Clear separation of tags vs. user touches (finger interactions)
- FilterID-aware filtering (exclude tags parked on a stand/dock)
- A dedicated **Tag Builder** to create and export tag templates as JSON
- A **Runtime** core for vvvv gamma / VL

**License:** MIT

---

## Overview

On an IR touch surface, physical tags appear as sets of touch points, mixed with regular user touches.  
TouchTag solves:

> Detect which tag is on the table, where its center is, what its rotation is, and which touches are *not* tags — even with sensor noise, temporary point loss, and extra points during placement/movement.

Key properties:

- Uses normalized coordinates `(0..1, 0..1)` from the IR frame
- Supports tags with 4–6 legs, arbitrary (non-symmetric) layouts
- Tolerant to:
  - reasonable sensor noise
  - short-lived missing legs
  - transient extra points
- No ML: a clear geometric approach (similarity matching + small combinatorics)

---

## Repository Structure

The project is logically split into two parts.

### 1. `TouchTag.TagBuilder`

Tools (VL + C#) to define and export tag templates.

**Responsibilities:**

- Capture real leg positions from the sensor for each physical tag.
- Run **NormalizeTemplate**:
  - translate the centroid of legs to `(0,0)`;
  - fix a canonical orientation (pick a reference leg and rotate);
  - sort legs to get a stable and reproducible order.
- Export templates to `tags.json`, for example:

```JSON
[
  {
    "TagID": "ID0",
    "FilterID": "DEADBEEF",
    "Legs": [
      { "X": 0.1309, "Y": 0.0000 },
      { "X": 0.0639, "Y": -0.0280 },
      { "X": -0.0107, "Y": -0.1208 },
      { "X": -0.1279, "Y": 0.0200 },
      { "X": -0.0629, "Y": 0.0567 },
      { "X": 0.0067, "Y": 0.0721 }
    ],
    "Diameter": 0.6
  }
]
```

**Why normalization matters**

All templates live in a canonical local space.  
At runtime we only solve for one transform:

> “Rotate + translate (and near-unit scale) this canonical pattern to best fit the observed points.”

This:

- simplifies the math,
- makes templates comparable,
- reduces false positives between similar tag layouts.

---

### 2. `TouchTag.Runtime`

Runtime core (VL + C#) designed to plug into vvvv gamma patches.

**High-level pipeline:**

1. **Input**
   - Normalized IR touch points `(0..1, 0..1)`.

2. **Preprocessing**
   - Deduplicate / merge very close points.
   - Cluster using `VL.DBScan`.

3. **Cluster splitting**
   - `SplitLargeClustersFromPoints` splits merged clusters (e.g. two tags too close, or tag + fingers).

4. **Tag matching** (per cluster)

   For each cluster and each template:

   - **Count filter**
     - The cluster size must be compatible with the template leg count,
       considering:
       - `maxMissingLegs` — allowed missing legs
       - `maxExtraPoints` — allowed extra points

   - **Deterministic hypotheses**
     - Choose subsets with K ≥ 3 legs and K points.
     - Try all permutations (feasible for 4–6 legs).

   - **Similarity transform (Procrustes)**
     - Compute optimal rotation, translation and scale `s`.
     - Enforce `|s - 1| <= scaleTolerance` (all tags share physical size).

   - **Coverage check**
     - For the full template vs. all cluster points:
       - no more than `maxMissingLegs` legs without a nearby point (within `legTolerance`);
       - no more than `maxExtraPoints` points without a nearby leg;
       - mean error of matched legs ≤ `meanErrorTolerance`.

   For each `TagID`, keep only the best (lowest-error) hypothesis.

   **Ambiguity resolution:**

   - Sort candidates by error.
   - If the second-best *different* `TagID` is too close to the best
     (difference `< ambiguityMargin`), treat the cluster as ambiguous → no tag.

5. **FilterID integration (optional)**

   - `tagsDefinitelyOffscreen`:
     - list of TagIDs known (via FilterID) to be on a stand/dock → excluded from matching.
   - `tagsDefinitelyOnscreen`:
     - if available, restrict matching to this whitelist only.

6. **Multiple instances of same tag**

   - `allowMultipleSameTags = false`
     - At most one active match per TagID (keep the best).
   - `allowMultipleSameTags = true`
     - Multiple instances allowed, tracking resolves them by proximity.

7. **Outputs**

   - `RecognizedTagCandidateRecord[]`
     - `TagID`, `Center`, `RotationDeg`, `Error`
   - `UserTouches[]`
     - All points not assigned to any recognized tag (for UI/gestures).
   - `Tracker.TrackAndSmoothStepStateless`
     - Produces smoothed `RuntimeOutputTagRecord[]` with:
       - stable center & rotation,
       - `Stable` flag based on appear/disappear frame thresholds.

---

## Recommended Parameters

Good starting values for a typical normalized IR setup:

- `meanErrorTolerance = 0.01`
- `legTolerance = 0.02`
- `maxLegsPerTag = 6`
- `maxMissingLegs = 1`
- `maxExtraPoints = 1`
- `scaleTolerance = 0.10` (accept scale in `[0.90 .. 1.10]`)
- `ambiguityMargin = 0.003` (≈ `0.3 * meanErrorTolerance`)
- `belongEps = 0.015–0.02`
- `allowMultipleSameTags = false` (enable later if you truly need duplicates)

Tune these based on real-world sensor noise and mechanical tolerances.

---

## Requirements

- [vvvv gamma](https://visualprogramming.net/) (stable release)
- VL:
  - `VL.DBScan` (from `VL.StandardLibs` / nuget)
  - `VL.CoreLib`
- .NET 6+ / .NET Standard compatible for the C# components

No heavy external libraries are required.

---

## Usage

1. Use **Tag Builder** to capture legs and export `tags.json`.
2. Feed normalized IR touch points into your vvvv gamma patch.
3. Chain:
   - cleaning / deduplication,
   - `VL.DBScan`,
   - `SplitLargeClustersFromPoints`,
   - `ClusterMatcher.BatchMatchAndFilter`,
   - `Tracker.TrackAndSmoothStepStateless`.
4. Read:
   - tag poses from `RuntimeOutputTagRecord[]`,
   - free touches from `UserTouches[]` for interaction logic.

---

## Roadmap

- Optional RANSAC layer for even stronger outlier resistance.
- Visual debugging helpers (cluster overlays, match quality, coverage).
- Example projects for common interactive table scenarios.

---

## Contributing

Contributions are welcome:

- new matching strategies,
- improved cluster splitting heuristics,
- better visual tools/debuggers,
- additional demos and documentation.
