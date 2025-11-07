namespace TouchTag.Runtime
{
    // ==============================
    // Data types
    // ==============================

    public class TagTemplateRecord
    {
        // ВАЖНО:
        // Legs должны быть нормализованы в Tag Builder:
        // - центр масс ног в (0,0)
        // - фиксированная ориентация (NormalizeTemplate)
        public string TagID = string.Empty;
        public string FilterID = string.Empty;
        public Vector2[] Legs = Array.Empty<Vector2>();
        public float Diameter = 0f;
    }

    public class RecognizedTagCandidateRecord
    {
        public string TagID = string.Empty;
        public Vector2 Center;          // 0..1
        public float RotationDeg;       // 0..360
        public float Error;             // средняя ошибка по ногам
        public Vector2[] SourcePoints = Array.Empty<Vector2>(); // точки кластера
    }

    public class TrackedTagStateRecord
    {
        public string TagID = string.Empty;
        public Vector2 SmoothedCenter;
        public float SmoothedRotationDeg;
        public int FramesVisible;
        public int FramesMissing;
        public bool Stable;
    }

    public class RuntimeOutputTagRecord
    {
        public string TagID = string.Empty;
        public Vector2 Position;        // 0..1
        public float RotationDeg;       // 0..360
        public bool Stable;
    }

    // ==============================
    // Math helpers (корректный similarity)
    // ==============================

    public static class MatchingMath
    {
        static Vector2 Centroid(IList<Vector2> pts)
        {
            float sx = 0, sy = 0;
            int n = pts.Count;
            for (int i = 0; i < n; i++)
            {
                sx += pts[i].X;
                sy += pts[i].Y;
            }
            float inv = 1f / Math.Max(1, n);
            return new Vector2(sx * inv, sy * inv);
        }

        // Correct 2D similarity transform (Procrustes, без отражений):
        // по парам M[i] -> O[i] находим s, R, T,
        // минимизирующие сумму квадратов.
        public static bool SolveSimilarityTransform(
            IList<Vector2> M,
            IList<Vector2> O,
            out Vector2 T,
            out float cosTheta,
            out float sinTheta,
            out float s,
            out float meanError)
        {
            T = Vector2.Zero;
            cosTheta = 1f;
            sinTheta = 0f;
            s = 1f;
            meanError = float.MaxValue;

            int n = M.Count;
            if (n != O.Count || n < 2)
                return false;

            var Mc = Centroid(M);
            var Oc = Centroid(O);

            var X = new Vector2[n];
            var Y = new Vector2[n];
            for (int i = 0; i < n; i++)
            {
                X[i] = M[i] - Mc;
                Y[i] = O[i] - Oc;
            }

            double Sxx = 0.0;
            double Sxy = 0.0;

            for (int i = 0; i < n; i++)
            {
                // dot и псевдо-cross для оценки поворота
                Sxx += X[i].X * Y[i].X + X[i].Y * Y[i].Y;
                Sxy += X[i].X * Y[i].Y - X[i].Y * Y[i].X;
            }

            if (Math.Abs(Sxx) < 1e-12 && Math.Abs(Sxy) < 1e-12)
            {
                cosTheta = 1f;
                sinTheta = 0f;
            }
            else
            {
                double norm = Math.Sqrt(Sxx * Sxx + Sxy * Sxy);
                cosTheta = (float)(Sxx / norm);
                sinTheta = (float)(Sxy / norm);
            }

            double num = 0.0;
            double den = 0.0;

            for (int i = 0; i < n; i++)
            {
                float rx = cosTheta * X[i].X - sinTheta * X[i].Y;
                float ry = sinTheta * X[i].X + cosTheta * X[i].Y;
                num += Y[i].X * rx + Y[i].Y * ry;
                den += X[i].X * X[i].X + X[i].Y * X[i].Y;
            }

            if (den < 1e-12)
                return false;

            s = (float)(num / den);

            // translation
            float rmcx = cosTheta * Mc.X - sinTheta * Mc.Y;
            float rmcy = sinTheta * Mc.X + cosTheta * Mc.Y;

            T = new Vector2(
                Oc.X - s * rmcx,
                Oc.Y - s * rmcy
            );

            // mean error
            double err = 0.0;
            for (int i = 0; i < n; i++)
            {
                float rx = cosTheta * M[i].X - sinTheta * M[i].Y;
                float ry = sinTheta * M[i].X + cosTheta * M[i].Y;
                float px = T.X + s * rx;
                float py = T.Y + s * ry;
                float dx = px - O[i].X;
                float dy = py - O[i].Y;
                err += Math.Sqrt(dx * dx + dy * dy);
            }

            meanError = (float)(err / n);
            return true;
        }

        // Coverage: проверка, что весь шаблон лег на точки и наоборот.
        public static void ComputeCoverageAndError(
            IList<Vector2> templateLegs,
            IList<Vector2> observed,
            Vector2 T,
            float cosTheta,
            float sinTheta,
            float s,
            float legTolerance,
            out float meanError,
            out int missingLegs,
            out int extraPoints)
        {
            meanError = float.MaxValue;
            missingLegs = 0;
            extraPoints = 0;

            int m = templateLegs.Count;
            int o = observed.Count;
            if (m == 0 || o == 0)
            {
                missingLegs = m;
                extraPoints = o;
                return;
            }

            var preds = new Vector2[m];
            for (int k = 0; k < m; k++)
            {
                var leg = templateLegs[k];
                float rx = cosTheta * leg.X - sinTheta * leg.Y;
                float ry = sinTheta * leg.X + cosTheta * leg.Y;
                preds[k] = new Vector2(
                    T.X + s * rx,
                    T.Y + s * ry
                );
            }

            float tol2 = legTolerance * legTolerance;

            // ноги -> точки
            double errSum = 0.0;
            int hitCount = 0;

            for (int k = 0; k < m; k++)
            {
                float best2 = float.MaxValue;
                for (int i = 0; i < o; i++)
                {
                    float dx = preds[k].X - observed[i].X;
                    float dy = preds[k].Y - observed[i].Y;
                    float d2 = dx * dx + dy * dy;
                    if (d2 < best2) best2 = d2;
                }

                if (best2 > tol2)
                {
                    missingLegs++;
                }
                else
                {
                    errSum += Math.Sqrt(best2);
                    hitCount++;
                }
            }

            // точки -> ноги
            for (int i = 0; i < o; i++)
            {
                float best2 = float.MaxValue;
                for (int k = 0; k < m; k++)
                {
                    float dx = observed[i].X - preds[k].X;
                    float dy = observed[i].Y - preds[k].Y;
                    float d2 = dx * dx + dy * dy;
                    if (d2 < best2) best2 = d2;
                }

                if (best2 > tol2)
                    extraPoints++;
            }

            if (hitCount > 0)
                meanError = (float)(errSum / hitCount);
            else
                meanError = float.MaxValue;
        }
    }

    // ==============================
    // Combinatorics (m,n ≤ 6)
    // ==============================

    public static class CombPerm
    {
        public static List<int[]> Combinations(int n, int k)
        {
            var res = new List<int[]>();
            if (k < 0 || k > n) return res;

            var cur = new int[k];

            void Rec(int start, int depth)
            {
                if (depth == k)
                {
                    res.Add((int[])cur.Clone());
                    return;
                }

                for (int i = start; i <= n - (k - depth); i++)
                {
                    cur[depth] = i;
                    Rec(i + 1, depth + 1);
                }
            }

            Rec(0, 0);
            return res;
        }

        public static List<int[]> Permutations(int[] arr)
        {
            var a = (int[])arr.Clone();
            var list = new List<int[]>();

            void Emit() => list.Add((int[])a.Clone());

            void Heap(int k)
            {
                if (k == 1)
                {
                    Emit();
                    return;
                }

                Heap(k - 1);

                for (int i = 0; i < k - 1; i++)
                {
                    if (k % 2 == 0)
                    {
                        (a[i], a[k - 1]) = (a[k - 1], a[i]);
                    }
                    else
                    {
                        (a[0], a[k - 1]) = (a[k - 1], a[0]);
                    }
                    Heap(k - 1);
                }
            }

            Heap(a.Length);
            return list;
        }
    }

    // ==============================
    // Cluster matching
    // ==============================

    public static class ClusterMatcher
    {
        public static bool TryMatchClusterToTagFromPoints(
            IEnumerable<Vector2> clusterPoints,
            IEnumerable<TagTemplateRecord> tagPool,
            float meanErrorTolerance,
            float legTolerance,
            int maxLegsPerTag,
            int maxMissingLegs,
            int maxExtraPoints,
            float scaleTolerance,
            float ambiguityMargin,
            out RecognizedTagCandidateRecord bestCandidate)
        {
            bestCandidate = null;

            var O = (clusterPoints ?? Array.Empty<Vector2>()).ToArray();
            int oCount = O.Length;
            if (oCount < 3)
                return false;

            var pool = (tagPool ?? Array.Empty<TagTemplateRecord>()).ToArray();
            if (pool.Length == 0)
                return false;

            var bestByTag = new List<RecognizedTagCandidateRecord>();

            foreach (var tag in pool)
            {
                var M = tag.Legs ?? Array.Empty<Vector2>();
                int mCount = M.Length;
                if (mCount < 3) continue;
                if (mCount > maxLegsPerTag) continue;

                // быстрый фильтр по количеству
                if (oCount < mCount - maxMissingLegs) continue;
                if (oCount > mCount + maxExtraPoints) continue;

                RecognizedTagCandidateRecord bestForTag = null;
                float bestErrForTag = float.MaxValue;

                int Kmin = Math.Max(3, mCount - maxMissingLegs);
                int Kmax = Math.Min(mCount, oCount);

                for (int K = Kmax; K >= Kmin; K--)
                {
                    var combM = CombPerm.Combinations(mCount, K);
                    var combO = CombPerm.Combinations(oCount, K);

                    foreach (var idxM in combM)
                        foreach (var idxO in combO)
                        {
                            var perms = CombPerm.Permutations(idxO);
                            foreach (var perm in perms)
                            {
                                var Ms = new Vector2[K];
                                var Os = new Vector2[K];
                                for (int i = 0; i < K; i++)
                                {
                                    Ms[i] = M[idxM[i]];
                                    Os[i] = O[perm[i]];
                                }

                                if (!MatchingMath.SolveSimilarityTransform(
                                        Ms, Os,
                                        out var T,
                                        out var cosTh,
                                        out var sinTh,
                                        out var s,
                                        out var _))
                                    continue;

                                if (Math.Abs(s - 1f) > scaleTolerance)
                                    continue;

                                MatchingMath.ComputeCoverageAndError(
                                    M, O, T, cosTh, sinTh, s,
                                    legTolerance,
                                    out var fullErr,
                                    out var missingLegs,
                                    out var extraPoints);

                                if (missingLegs > maxMissingLegs) continue;
                                if (extraPoints > maxExtraPoints) continue;
                                if (fullErr > meanErrorTolerance) continue;

                                if (fullErr < bestErrForTag)
                                {
                                    bestErrForTag = fullErr;

                                    float angleRad = (float)Math.Atan2(sinTh, cosTh);
                                    float angleDeg = angleRad * 180f / (float)Math.PI;
                                    if (angleDeg < 0) angleDeg += 360f;

                                    bestForTag = new RecognizedTagCandidateRecord
                                    {
                                        TagID = tag.TagID,
                                        Center = T,
                                        RotationDeg = angleDeg,
                                        Error = fullErr,
                                        SourcePoints = O
                                    };
                                }
                            }
                        }

                    // ранний выход, если уже очень маленькая ошибка
                    if (bestForTag != null && bestErrForTag < meanErrorTolerance * 0.5f)
                        break;
                }

                if (bestForTag != null)
                    bestByTag.Add(bestForTag);
            }

            if (bestByTag.Count == 0)
                return false;

            bestByTag.Sort((a, b) => a.Error.CompareTo(b.Error));
            var best = bestByTag[0];

            if (bestByTag.Count > 1)
            {
                var second = bestByTag[1];

                // неоднозначность учитываем ТОЛЬКО если это разные теги
                if (second.TagID != best.TagID &&
                    second.Error - best.Error < ambiguityMargin)
                {
                    return false;
                }
            }

            bestCandidate = best;
            return true;
        }

        public static void BatchMatchAndFilter(
            IEnumerable<IEnumerable<Vector2>> clustersPoints,
            IEnumerable<Vector2> allCleanPoints,
            IEnumerable<TagTemplateRecord> tagPool,
            IEnumerable<string> tagsDefinitelyOffscreen,
            IEnumerable<string> tagsDefinitelyOnscreen,
            float meanErrorTolerance,
            float legTolerance,
            int maxLegsPerTag,
            int maxMissingLegs,
            int maxExtraPoints,
            float scaleTolerance,
            float ambiguityMargin,
            float belongEps,
            bool allowMultipleSameTags,
            out IEnumerable<RecognizedTagCandidateRecord> recognizedCandidates,
            out IEnumerable<Vector2> userTouches)
        {
            var cps = (clustersPoints ?? Array.Empty<IEnumerable<Vector2>>())
                .Select(c => (c ?? Array.Empty<Vector2>()).ToArray())
                .ToArray();

            var all = (allCleanPoints ?? Array.Empty<Vector2>()).ToArray();
            var poolAll = (tagPool ?? Array.Empty<TagTemplateRecord>()).ToArray();

            var off = new HashSet<string>(tagsDefinitelyOffscreen ?? Array.Empty<string>());
            var on = new HashSet<string>(tagsDefinitelyOnscreen ?? Array.Empty<string>());

            TagTemplateRecord[] pool;
            if (on.Count > 0)
                pool = poolAll.Where(t => on.Contains(t.TagID) && !off.Contains(t.TagID)).ToArray();
            else
                pool = poolAll.Where(t => !off.Contains(t.TagID)).ToArray();

            var valids = new List<RecognizedTagCandidateRecord>();
            var used = new bool[all.Length];
            float belongEps2 = belongEps * belongEps;

            foreach (var cp in cps)
            {
                if (TryMatchClusterToTagFromPoints(
                        cp,
                        pool,
                        meanErrorTolerance,
                        legTolerance,
                        maxLegsPerTag,
                        maxMissingLegs,
                        maxExtraPoints,
                        scaleTolerance,
                        ambiguityMargin,
                        out var cand))
                {
                    valids.Add(cand);

                    // вычитаем точки кластера из пользовательских касаний
                    for (int i = 0; i < all.Length; i++)
                    {
                        if (used[i]) continue;

                        for (int j = 0; j < cand.SourcePoints.Length; j++)
                        {
                            float dx = all[i].X - cand.SourcePoints[j].X;
                            float dy = all[i].Y - cand.SourcePoints[j].Y;
                            if (dx * dx + dy * dy <= belongEps2)
                            {
                                used[i] = true;
                                break;
                            }
                        }
                    }
                }
            }

            if (!allowMultipleSameTags)
            {
                var bestById = new Dictionary<string, RecognizedTagCandidateRecord>();
                foreach (var c in valids)
                {
                    if (!bestById.TryGetValue(c.TagID, out var old) || c.Error < old.Error)
                        bestById[c.TagID] = c;
                }
                valids = bestById.Values.ToList();
            }

            var touches = new List<Vector2>();
            for (int i = 0; i < all.Length; i++)
                if (!used[i]) touches.Add(all[i]);

            recognizedCandidates = valids;
            userTouches = touches;
        }
    }

    // ==============================
    // Tracking (тот же, опрятный)
    // ==============================

    public static class Tracker
    {
        static Vector2 Lerp(Vector2 a, Vector2 b, float t) => a + (b - a) * t;

        public static void TrackAndSmoothStepStateless(
            IEnumerable<RecognizedTagCandidateRecord> recognizedNow,
            IEnumerable<TrackedTagStateRecord> prevState,
            int appearConfirmFrames,
            int disappearGraceFrames,
            float alphaPos,
            float alphaRot,
            bool allowMultipleSameTags,
            float maxAssocDistance,
            out IEnumerable<TrackedTagStateRecord> nextState,
            out IEnumerable<RuntimeOutputTagRecord> outputTags)
        {
            var prev = (prevState ?? Array.Empty<TrackedTagStateRecord>())
                .Select(p => new TrackedTagStateRecord
                {
                    TagID = p.TagID,
                    SmoothedCenter = p.SmoothedCenter,
                    SmoothedRotationDeg = p.SmoothedRotationDeg,
                    FramesVisible = p.FramesVisible,
                    FramesMissing = p.FramesMissing,
                    Stable = p.Stable
                }).ToList();

            var cands = (recognizedNow ?? Array.Empty<RecognizedTagCandidateRecord>()).ToArray();
            var newStates = new List<TrackedTagStateRecord>();

            if (!allowMultipleSameTags)
            {
                var seen = new HashSet<string>();

                foreach (var cand in cands)
                {
                    seen.Add(cand.TagID);
                    var t = prev.Find(x => x.TagID == cand.TagID);
                    if (t == null)
                    {
                        t = new TrackedTagStateRecord
                        {
                            TagID = cand.TagID,
                            SmoothedCenter = cand.Center,
                            SmoothedRotationDeg = cand.RotationDeg,
                            FramesVisible = 1,
                            FramesMissing = 0,
                            Stable = (appearConfirmFrames <= 1)
                        };
                    }
                    else
                    {
                        t.SmoothedCenter = Lerp(t.SmoothedCenter, cand.Center, alphaPos);

                        float oldRad = t.SmoothedRotationDeg * (float)Math.PI / 180f;
                        float newRad = cand.RotationDeg * (float)Math.PI / 180f;
                        var vOld = new Vector2((float)Math.Cos(oldRad), (float)Math.Sin(oldRad));
                        var vNew = new Vector2((float)Math.Cos(newRad), (float)Math.Sin(newRad));
                        var vBlend = Lerp(vOld, vNew, alphaRot);
                        float len = vBlend.Length();
                        if (len > 1e-6f) vBlend /= len;
                        float deg = (float)(Math.Atan2(vBlend.Y, vBlend.X) * 180.0 / Math.PI);
                        if (deg < 0) deg += 360f;

                        t.SmoothedRotationDeg = deg;
                        t.FramesVisible += 1;
                        t.FramesMissing = 0;
                    }

                    t.Stable = (t.FramesVisible >= appearConfirmFrames);
                    newStates.Add(t);
                }

                foreach (var t in prev)
                {
                    if (!seen.Contains(t.TagID))
                    {
                        t.FramesMissing++;
                        if (t.FramesMissing <= disappearGraceFrames)
                            newStates.Add(t);
                    }
                }
            }
            else
            {
                float maxAssocDist2 = maxAssocDistance * maxAssocDistance;
                var used = new bool[cands.Length];

                foreach (var t in prev)
                {
                    int bestIdx = -1;
                    float bestD2 = float.MaxValue;

                    for (int i = 0; i < cands.Length; i++)
                    {
                        if (used[i]) continue;
                        if (cands[i].TagID != t.TagID) continue;

                        float d2 = Vector2.DistanceSquared(cands[i].Center, t.SmoothedCenter);
                        if (d2 < bestD2)
                        {
                            bestD2 = d2;
                            bestIdx = i;
                        }
                    }

                    if (bestIdx >= 0 && bestD2 <= maxAssocDist2)
                    {
                        var cand = cands[bestIdx];
                        used[bestIdx] = true;

                        t.SmoothedCenter = Lerp(t.SmoothedCenter, cand.Center, alphaPos);

                        float oldRad = t.SmoothedRotationDeg * (float)Math.PI / 180f;
                        float newRad = cand.RotationDeg * (float)Math.PI / 180f;
                        var vOld = new Vector2((float)Math.Cos(oldRad), (float)Math.Sin(oldRad));
                        var vNew = new Vector2((float)Math.Cos(newRad), (float)Math.Sin(newRad));
                        var vBlend = Lerp(vOld, vNew, alphaRot);
                        float len = vBlend.Length();
                        if (len > 1e-6f) vBlend /= len;
                        float deg = (float)(Math.Atan2(vBlend.Y, vBlend.X) * 180.0 / Math.PI);
                        if (deg < 0) deg += 360f;

                        t.SmoothedRotationDeg = deg;
                        t.FramesVisible += 1;
                        t.FramesMissing = 0;
                        t.Stable = (t.FramesVisible >= appearConfirmFrames);

                        newStates.Add(t);
                    }
                    else
                    {
                        t.FramesMissing++;
                        if (t.FramesMissing <= disappearGraceFrames)
                            newStates.Add(t);
                    }
                }

                for (int i = 0; i < cands.Length; i++)
                {
                    if (used[i]) continue;
                    var cand = cands[i];

                    var nt = new TrackedTagStateRecord
                    {
                        TagID = cand.TagID,
                        SmoothedCenter = cand.Center,
                        SmoothedRotationDeg = cand.RotationDeg,
                        FramesVisible = 1,
                        FramesMissing = 0,
                        Stable = (appearConfirmFrames <= 1)
                    };
                    newStates.Add(nt);
                }
            }

            var outs = new List<RuntimeOutputTagRecord>();
            foreach (var t in newStates)
            {
                outs.Add(new RuntimeOutputTagRecord
                {
                    TagID = t.TagID,
                    Position = t.SmoothedCenter,
                    RotationDeg = t.SmoothedRotationDeg,
                    Stable = t.Stable
                });
            }

            nextState = newStates.ToArray();
            outputTags = outs;
        }
    }
}
