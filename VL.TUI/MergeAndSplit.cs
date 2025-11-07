namespace TouchTag.Runtime
{
    public static class MergeAndSplit2D
    {
        // =====================
        //  ШАГ 2: антидубли
        // =====================

        /// <summary>
        /// Схлопывает почти совпадающие точки (радиус eps) одной порции/кадра.
        /// VL-пины: Spread<Vector2> -> Spread<Vector2>
        /// </summary>
        public static IEnumerable<Vector2> MergeDuplicates(IEnumerable<Vector2> points, float eps)
        {
            var arr = points?.ToArray() ?? Array.Empty<Vector2>();
            if (arr.Length == 0) yield break;

            var used = new bool[arr.Length];
            float eps2 = eps * eps;

            for (int i = 0; i < arr.Length; i++)
            {
                if (used[i]) continue;

                Vector2 acc = arr[i];
                int cnt = 1;
                used[i] = true;

                for (int j = i + 1; j < arr.Length; j++)
                {
                    if (used[j]) continue;
                    if (Vector2.DistanceSquared(arr[i], arr[j]) <= eps2)
                    {
                        acc += arr[j];
                        cnt++;
                        used[j] = true;
                    }
                }

                yield return acc / Math.Max(1, cnt);
            }
        }

        /// <summary>
        /// Удобный синоним для "кадр целиком" (тот же MergeDuplicates).
        /// </summary>
        public static IEnumerable<Vector2> MergeDuplicatesForFrame(IEnumerable<Vector2> framePoints, float eps)
            => MergeDuplicates(framePoints, eps);

        // ============================
        //  ШАГ 4: разлипание кластеров
        // ============================

        /// <summary>
        /// Разделяет "жирные" кластеры (k-means k=2 с инициализацией самой дальней парой, затем PCA-фоллбек).
        /// Вход/выход: Spread<Spread<Vector2>>
        /// </summary>
        public static IEnumerable<IEnumerable<Vector2>> SplitLargeClustersFromPoints(
            IEnumerable<IEnumerable<Vector2>> clustersPoints,
            int maxLegsPerTag,
            int extraAllowance,
            float maxClusterRadius,
            int minPointsPerSubcluster = 3,
            int kmeansMaxIters = 20)
        {
            var clusters = clustersPoints?.Select(cp => cp?.ToArray() ?? Array.Empty<Vector2>()).ToArray()
                           ?? Array.Empty<Vector2[]>();

            foreach (var pts in clusters)
            {
                if (pts.Length > maxLegsPerTag + extraAllowance)
                {
                    // Попытка k-means (k = 2)
                    if (TryKMeans2(pts, kmeansMaxIters, out var g1, out var g2))
                    {
                        bool ok = ValidateGroup(g1, minPointsPerSubcluster, maxClusterRadius)
                               && ValidateGroup(g2, minPointsPerSubcluster, maxClusterRadius);
                        if (ok)
                        {
                            yield return g1;
                            yield return g2;
                            continue;
                        }
                    }

                    // Fallback: PCA-сплит по главной оси (медиана проекций)
                    if (TryPCASplit(pts, out var h1, out var h2))
                    {
                        bool ok = ValidateGroup(h1, minPointsPerSubcluster, maxClusterRadius)
                               && ValidateGroup(h2, minPointsPerSubcluster, maxClusterRadius);
                        if (ok)
                        {
                            yield return h1;
                            yield return h2;
                            continue;
                        }
                    }
                }

                // как есть
                yield return pts;
            }
        }

        // ============================
        //  Вспомогательные методы
        // ============================

        static bool ValidateGroup(List<Vector2> pts, int minCount, float maxRadius)
        {
            if (pts == null || pts.Count < minCount) return false;
            var c = Centroid(pts);
            float rMax = 0f;
            foreach (var p in pts) rMax = MathF.Max(rMax, Vector2.Distance(p, c));
            return rMax <= maxRadius;
        }

        static Vector2 Centroid(IList<Vector2> pts)
        {
            float sx = 0, sy = 0;
            int n = pts.Count;
            for (int i = 0; i < n; i++) { sx += pts[i].X; sy += pts[i].Y; }
            float inv = 1f / Math.Max(1, n);
            return new Vector2(sx * inv, sy * inv);
        }

        // --- KMeans k=2 с инициализацией самой дальней парой
        static bool TryKMeans2(Vector2[] pts, int maxIters, out List<Vector2> g1, out List<Vector2> g2)
        {
            g1 = new List<Vector2>(); g2 = new List<Vector2>();
            int n = pts.Length;
            if (n < 2) return false;

            // стартовые центры — самая дальняя пара
            float bestD2 = -1;
            int iA = 0, iB = 1;
            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    float d2 = Vector2.DistanceSquared(pts[i], pts[j]);
                    if (d2 > bestD2) { bestD2 = d2; iA = i; iB = j; }
                }
            }
            var c1 = pts[iA];
            var c2 = pts[iB];

            var assign = new int[n];
            for (int it = 0; it < maxIters; it++)
            {
                bool anyChange = false;

                // assign
                for (int i = 0; i < n; i++)
                {
                    float d1 = Vector2.DistanceSquared(pts[i], c1);
                    float d2 = Vector2.DistanceSquared(pts[i], c2);
                    int a = (d1 <= d2) ? 0 : 1;
                    if (assign[i] != a) { assign[i] = a; anyChange = true; }
                }

                // recompute centers
                var sum1 = Vector2.Zero; int cnt1 = 0;
                var sum2 = Vector2.Zero; int cnt2 = 0;
                for (int i = 0; i < n; i++)
                {
                    if (assign[i] == 0) { sum1 += pts[i]; cnt1++; }
                    else { sum2 += pts[i]; cnt2++; }
                }
                if (cnt1 == 0 || cnt2 == 0) break; // неудачное разбиение (пустой кластер)
                c1 = sum1 / cnt1;
                c2 = sum2 / cnt2;

                if (!anyChange) break;
            }

            // собрать группы
            for (int i = 0; i < n; i++)
            {
                if (assign[i] == 0) g1.Add(pts[i]); else g2.Add(pts[i]);
            }

            return g1.Count > 0 && g2.Count > 0;
        }

        // --- PCA split: главная ось ковариации, порог = медиана проекций
        static bool TryPCASplit(Vector2[] pts, out List<Vector2> g1, out List<Vector2> g2)
        {
            g1 = new List<Vector2>(); g2 = new List<Vector2>();
            int n = pts.Length;
            if (n < 2) return false;

            var c = Centroid(pts);

            // ковариация 2x2
            float sxx = 0, sxy = 0, syy = 0;
            for (int i = 0; i < n; i++)
            {
                var v = pts[i] - c;
                sxx += v.X * v.X;
                sxy += v.X * v.Y;
                syy += v.Y * v.Y;
            }
            float invN = 1f / Math.Max(1, n);
            sxx *= invN; sxy *= invN; syy *= invN;

            // максимальное собственное значение/вектор для [[sxx,sxy],[sxy,syy]]
            float tr = sxx + syy;
            float det = sxx * syy - sxy * sxy;
            float disc = MathF.Max(0f, tr * tr / 4f - det);
            float sqrt = MathF.Sqrt(disc);
            float lambdaMax = tr / 2f + sqrt;

            Vector2 axis = (MathF.Abs(sxy) > 1e-6f)
                ? new Vector2(sxy, lambdaMax - sxx)
                : new Vector2(1, 0);
            float len = axis.Length();
            if (len < 1e-6f) return false;
            axis /= len;

            // проекции и медианный порог
            var projs = new float[n];
            for (int i = 0; i < n; i++) projs[i] = Vector2.Dot(pts[i] - c, axis);

            var sorted = projs.ToArray();
            Array.Sort(sorted);
            float median = (n % 2 == 1)
                ? sorted[n / 2]
                : 0.5f * (sorted[n / 2 - 1] + sorted[n / 2]);

            for (int i = 0; i < n; i++)
            {
                if (projs[i] <= median) g1.Add(pts[i]); else g2.Add(pts[i]);
            }

            return g1.Count > 0 && g2.Count > 0;
        }
    }
}
