using System.Text.Json;
using System.Text.Json.Serialization;

namespace TouchTag.Runtime
{
    public static class TagBuilderUtils
    {
        // 1) Устойчивое усреднение координат ног по нескольким кадрам
        //    frames: Spread<Spread<Vector2>> — набор кадров, где виден один и тот же тег
        //    Возвращает усреднённые точки (без нормализации).
        public static IEnumerable<Vector2> AverageLegsAcrossFrames(
            IEnumerable<IEnumerable<Vector2>> frames, float pairEps = 0.01f)
        {
            var groups = new List<List<Vector2>>(); // группы «одна и та же нога»

            foreach (var frame in frames ?? Enumerable.Empty<IEnumerable<Vector2>>())
            {
                var pts = frame?.ToArray() ?? Array.Empty<Vector2>();
                var used = new bool[pts.Length];

                // для каждой точки — найти ближайшую группу
                for (int i = 0; i < pts.Length; i++)
                {
                    int bestG = -1; float bestD2 = float.MaxValue;
                    for (int g = 0; g < groups.Count; g++)
                    {
                        var c = Average(groups[g]);
                        float d2 = Vector2.DistanceSquared(pts[i], c);
                        if (d2 < bestD2) { bestD2 = d2; bestG = g; }
                    }
                    if (bestG >= 0 && bestD2 <= pairEps * pairEps)
                        groups[bestG].Add(pts[i]);
                    else
                        groups.Add(new List<Vector2> { pts[i] });
                }
            }

            // усреднение по группам
            return groups.Select(g => Average(g)).ToArray();

            Vector2 Average(List<Vector2> vs)
            {
                if (vs == null || vs.Count == 0) return Vector2.Zero;
                float sx = 0, sy = 0; foreach (var v in vs) { sx += v.X; sy += v.Y; }
                float inv = 1f / vs.Count; return new Vector2(sx * inv, sy * inv);
            }
        }

        // 2) Нормализация: центр → (0,0), сортировка по углу (по часовой), задание нулевой ориентации
        public static IEnumerable<Vector2> NormalizeTemplate(
    IEnumerable<Vector2> legsWorld,
    float rotateZeroDeg = 0f,
    bool reorderAfterRotation = true,
    bool startListFromZeroLeg = true
)
        {
            var arr = legsWorld?.ToArray() ?? Array.Empty<Vector2>();
            if (arr.Length == 0) return arr;

            // 1) Центр масс -> (0,0)
            var cx = arr.Average(p => p.X);
            var cy = arr.Average(p => p.Y);
            for (int i = 0; i < arr.Length; i++)
                arr[i] = new Vector2(arr[i].X - cx, arr[i].Y - cy);

            // 2) Выбираем опорную ногу (самая «дальняя» от центра)
            int refIdx = 0;
            float bestR2 = -1f;
            for (int i = 0; i < arr.Length; i++)
            {
                float r2 = arr[i].X * arr[i].X + arr[i].Y * arr[i].Y;
                if (r2 > bestR2) { bestR2 = r2; refIdx = i; }
            }

            // 3) Поворачиваем так, чтобы опорная нога смотрела в rotateZeroDeg
            float targetRad = rotateZeroDeg * (float)Math.PI / 180f;
            float curRad = (float)Math.Atan2(arr[refIdx].Y, arr[refIdx].X);
            float d = targetRad - curRad;
            float cos = (float)Math.Cos(d);
            float sin = (float)Math.Sin(d);
            for (int i = 0; i < arr.Length; i++)
            {
                var v = arr[i];
                arr[i] = new Vector2(cos * v.X - sin * v.Y, sin * v.X + cos * v.Y);
            }

            if (reorderAfterRotation)
            {
                // 4) Переcортировка по углу ПО ЧАСОВОЙ после поворота
                Array.Sort(arr, (a, b) =>
                {
                    float aa = (float)Math.Atan2(a.Y, a.X);
                    float bb = (float)Math.Atan2(b.Y, b.X);
                    // по часовой — убывание угла
                    return (-aa).CompareTo(-bb);
                });

                if (startListFromZeroLeg)
                {
                    // 5) Сдвигаем список так, чтобы первой была нога, ближе всего к rotateZeroDeg
                    int zeroIdx = 0;
                    float bestAbs = float.MaxValue;
                    for (int i = 0; i < arr.Length; i++)
                    {
                        float ang = (float)Math.Atan2(arr[i].Y, arr[i].X); // уже после поворота
                                                                           // разность углов в диапазоне [-pi,pi]
                        float diff = ang - targetRad;
                        while (diff > Math.PI) diff -= (float)(2 * Math.PI);
                        while (diff < -Math.PI) diff += (float)(2 * Math.PI);
                        float abs = Math.Abs(diff);
                        if (abs < bestAbs) { bestAbs = abs; zeroIdx = i; }
                    }
                    if (zeroIdx != 0)
                    {
                        var rotated = new Vector2[arr.Length];
                        for (int k = 0; k < arr.Length; k++)
                            rotated[k] = arr[(zeroIdx + k) % arr.Length];
                        arr = rotated;
                    }
                }
            }

            return arr; // IEnumerable<Vector2>, красиво мапится в Spread
        }


        // 3) Масштабирование (опционально): подогнать под «референсный диаметр/радиус»
        public static IEnumerable<Vector2> ScaleTemplate(IEnumerable<Vector2> legsCentered, float scale)
        {
            return (legsCentered ?? Array.Empty<Vector2>()).Select(v => v * scale).ToArray();
        }

        // 4) Собрать запись тега
        public static TagTemplateRecord BuildTagRecord(
            string tagID, string FilterID, IEnumerable<Vector2> legsNorm, float diameter = 0f)
        {
            return new TagTemplateRecord
            {
                TagID = tagID ?? string.Empty,
                FilterID = FilterID ?? string.Empty,
                Legs = (legsNorm?.ToArray() ?? Array.Empty<Vector2>()),
                Diameter = diameter
            };
        }

        // 5) JSON: загрузка/сохранение пула
        public static IEnumerable<TagTemplateRecord> LoadPoolFromJson(string json)
        {
            if (string.IsNullOrWhiteSpace(json)) return Array.Empty<TagTemplateRecord>();
            var opts = new JsonSerializerOptions
            {
                PropertyNameCaseInsensitive = true,
                ReadCommentHandling = JsonCommentHandling.Skip,
                AllowTrailingCommas = true,
                IncludeFields = true   // <-- ВАЖНО: сериализуем ПОЛЯ, а не свойства
            };
            var arr = JsonSerializer.Deserialize<TagTemplateRecord[]>(json, opts);
            return arr ?? Array.Empty<TagTemplateRecord>();
        }

        public static string SavePoolToJson(IEnumerable<TagTemplateRecord> pool, bool indented = true)
        {
            var opts = new JsonSerializerOptions
            {
                WriteIndented = indented,
                DefaultIgnoreCondition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingNull,
                IncludeFields = true   // <-- ВАЖНО
            };
            return JsonSerializer.Serialize(
                pool?.ToArray() ?? Array.Empty<TagTemplateRecord>(), opts
            );
        }


        // 6) Валидация пула: дубликаты TagID/FilterID
        public static IEnumerable<string> ValidatePool(IEnumerable<TagTemplateRecord> pool)
        {
            var list = pool?.ToArray() ?? Array.Empty<TagTemplateRecord>();
            var errs = new List<string>();

            var dupTag = list.GroupBy(t => t.TagID)
                             .Where(g => !string.IsNullOrEmpty(g.Key) && g.Count() > 1)
                             .Select(g => $"Duplicate TagID: {g.Key}");
            errs.AddRange(dupTag);

            var dupFilterID = list.GroupBy(t => t.FilterID)
                              .Where(g => !string.IsNullOrEmpty(g.Key) && g.Count() > 1)
                              .Select(g => $"Duplicate FilterID: {g.Key}");
            errs.AddRange(dupFilterID);

            return errs;
        }
    }
}

