using Sdcb.NBody.Common;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;

namespace Sdcb.NBody.Direct2D;

public static class CircularBufferTimedVector2Extensions
{
    #region 代理类 (Proxy Class)
    /// <summary>
    /// 一个只读的、零分配的代理类，用于表示 CircularBuffer<TimedVector2> 中的一个数据切片。
    /// 它实现了 IReadOnlyList<Vector2> 接口，但本身不存储 Vector2 集合，
    /// 而是在访问时实时从源缓冲区中提取数据。
    /// </summary>
    private sealed class Vector2Slice(CircularBuffer<TimedVector2> source, int offset, int count) : IReadOnlyList<Vector2>
    {
        public int Count => count;

        public Vector2 this[int index]
        {
            get
            {
                if (index < 0 || index >= count)
                {
                    throw new ArgumentOutOfRangeException(nameof(index));
                }
                return source[offset + index].Vector;
            }
        }

        public IEnumerator<Vector2> GetEnumerator()
        {
            for (int i = 0; i < count; i++)
            {
                yield return this[i];
            }
        }

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }
    #endregion

    #region 代理类 (Proxy Class for Groups)
    /// <summary>
    /// 一个只读的、零分配的代理类，用于表示分组后的 Vector2 数据。
    /// 它实现了 IReadOnlyList<IReadOnlyList<Vector2>> 接口，本身不存储数据集合，
    /// 而是在访问时按需创建代表每个分组的 Vector2Slice 代理。
    /// </summary>
    private sealed class GroupedVector2SliceList(CircularBuffer<TimedVector2> source, List<(int offset, int count)> groupBoundaries) : IReadOnlyList<IReadOnlyList<Vector2>>
    {
        public int Count => groupBoundaries.Count;

        public IReadOnlyList<Vector2> this[int index]
        {
            get
            {
                if (index < 0 || index >= groupBoundaries.Count)
                {
                    throw new ArgumentOutOfRangeException(nameof(index));
                }
                (int offset, int count) = groupBoundaries[index];
                return new Vector2Slice(source, offset, count);
            }
        }

        public IEnumerator<IReadOnlyList<Vector2>> GetEnumerator()
        {
            for (int i = 0; i < Count; i++)
            {
                yield return this[i];
            }
        }

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }
    #endregion

    /// <summary>
    /// [从最新位置倒数指定时间，然后按时间顺序分组] 从最新的历史记录开始，向前倒数指定的时间长度，然后从该位置开始按时间顺序对数据进行分组。（高性能、零分配版本）
    /// </summary>
    /// <remarks>
    /// 此方法首先从最新位置（索引 Count-1）向前倒数 totalDurationSeconds 的时间，
    /// 找到起始位置后，再从该起始位置按时间先后顺序（向前）遍历并分组。
    /// </remarks>
    /// <param name="history">包含时间戳和向量数据的环形缓冲区。</param>
    /// <param name="totalDurationSeconds">要处理的总时间跨度（从最新位置向前倒数）。</param>
    /// <param name="groupDurationSeconds">每个分组应包含的时间长度。</param>
    /// <returns>
    /// 一个 IReadOnlyList&lt;IReadOnlyList&lt;Vector2&gt;&gt;，其中每个 IReadOnlyList&lt;Vector2&gt; 是一个按时间顺序排列的零分配代理视图。
    /// </returns>
    public static IReadOnlyList<IReadOnlyList<Vector2>> GroupedByTimeFromOldest(
        this CircularBuffer<TimedVector2> history,
        float totalDurationSeconds,
        float groupDurationSeconds)
    {
        // 1. 参数校验
        if (totalDurationSeconds <= 0) throw new ArgumentOutOfRangeException(nameof(totalDurationSeconds), "总时间长度必须为正数。");
        if (groupDurationSeconds <= 0) throw new ArgumentOutOfRangeException(nameof(groupDurationSeconds), "每组时间长度必须为正数。");
        if (history.Count == 0) return new GroupedVector2SliceList(history, []);

        // 2. 从最新位置向前倒数 totalDurationSeconds，找到起始索引
        float accumulatedTime = 0f;
        int startIndex = history.Count - 1; // 默认从最新位置开始

        // 从最新位置向前倒数时间
        for (int i = history.Count - 1; i >= 0; i--)
        {
            TimedVector2 timedVector = history[i];
            if (accumulatedTime + timedVector.Dt > totalDurationSeconds)
            {
                // 找到了起始位置
                startIndex = i;
                break;
            }
            accumulatedTime += timedVector.Dt;
            startIndex = i;
        }

        // 3. 从起始位置开始，按时间顺序分组
        List<(int offset, int count)> groupBoundaries = [];
        float accumulatedGroupTime = 0f;
        int currentGroupItemCount = 0;
        int groupStartIndex = startIndex;

        // 从起始位置向最新位置遍历
        for (int i = startIndex; i < history.Count; i++)
        {
            TimedVector2 timedVector = history[i];
            accumulatedGroupTime += timedVector.Dt;
            currentGroupItemCount++;

            // 检查当前组是否已满
            if (accumulatedGroupTime >= groupDurationSeconds)
            {
                // 返回当前组的代理视图
                groupBoundaries.Add((groupStartIndex, currentGroupItemCount));

                // 为下一组重置
                accumulatedGroupTime = 0;
                currentGroupItemCount = 0;
                // 下一个组的起始索引将是当前元素的下一个位置
                groupStartIndex = i + 1;
            }
        }

        // 4. 处理循环结束后剩余的最后一组（尾部）数据
        if (currentGroupItemCount > 0)
        {
            groupBoundaries.Add((groupStartIndex, currentGroupItemCount));
        }

        return new GroupedVector2SliceList(history, groupBoundaries);
    }

    /// <summary>
    /// [从最新位置倒数指定时间，然后按指定数量分组] 从最新的历史记录开始，向前倒数指定的时间长度，然后将该时间段平均分成指定数量的组。（高性能、零分配版本）
    /// </summary>
    /// <remarks>
    /// 此方法首先从最新位置（索引 Count-1）向前倒数，计算实际可用的总时间，
    /// 然后将实际可用时间除以 count 得到每组的时间长度，按此时间长度分组。
    /// 当历史数据的总时间不足 totalDurationSeconds 时，使用实际可用的总时间进行分组。
    /// </remarks>
    /// <param name="history">包含时间戳和向量数据的环形缓冲区。</param>
    /// <param name="totalDurationSeconds">期望处理的总时间跨度（从最新位置向前倒数）。</param>
    /// <param name="count">要分成的组数。</param>
    /// <returns>
    /// 一个 IReadOnlyList&lt;IReadOnlyList&lt;Vector2&gt;&gt;，其中每个 IReadOnlyList&lt;Vector2&gt; 是一个按时间顺序排列的零分配代理视图。
    /// 如果无法分组或参数无效，返回空列表。
    /// </returns>
    public static IReadOnlyList<IReadOnlyList<Vector2>> GroupedByTimeFromOldest(
        this CircularBuffer<TimedVector2> history,
        float totalDurationSeconds,
        int count)
    {
        // 1. 参数校验
        if (totalDurationSeconds <= 0) throw new ArgumentOutOfRangeException(nameof(totalDurationSeconds), "总时间长度必须为正数。");
        if (count <= 0) throw new ArgumentOutOfRangeException(nameof(count), "分组数量必须为正数。");
        if (history.Count == 0) return new GroupedVector2SliceList(history, []);

        // 2. 计算实际可用的总时间
        float actualTotalTime = 0f;
        for (int i = history.Count - 1; i >= 0; i--)
        {
            TimedVector2 timedVector = history[i];
            if (actualTotalTime + timedVector.Dt > totalDurationSeconds)
            {
                // 如果加上当前项的时间会超过期望时间，就停止累加
                break;
            }
            actualTotalTime += timedVector.Dt;
        }

        // 3. 如果没有足够的历史数据，返回空列表
        if (actualTotalTime <= 0)
        {
            return new GroupedVector2SliceList(history, []);
        }

        // 4. 用实际可用时间除以分组数量来计算每组的时间长度
        float groupDurationSeconds = actualTotalTime / count;

        // 5. 调用原有的实现，但使用实际可用时间作为总时间
        return GroupedByTimeFromOldest(history, actualTotalTime, groupDurationSeconds);
    }

    /// <summary>
    /// 获取分组数量，按 GroupedByTime 的分组方式。
    /// </summary>
    public static int GetGroupCount(
        this CircularBuffer<TimedVector2> history,
        float totalDurationSeconds,
        float groupDurationSeconds)
    {
        if (totalDurationSeconds <= 0) throw new ArgumentOutOfRangeException(nameof(totalDurationSeconds), "总时间长度必须为正数。");
        if (groupDurationSeconds <= 0) throw new ArgumentOutOfRangeException(nameof(groupDurationSeconds), "每组时间长度必须为正数。");
        if (history.Count == 0) return 0;

        int groupCount = 0;
        float accumulatedTotalTime = 0f;
        float accumulatedGroupTime = 0f;
        int currentGroupItemCount = 0;

        for (int i = history.Count - 1; i >= 0; i--)
        {
            TimedVector2 timedVector = history[i];
            if (accumulatedTotalTime + timedVector.Dt > totalDurationSeconds) break;

            accumulatedTotalTime += timedVector.Dt;
            accumulatedGroupTime += timedVector.Dt;
            currentGroupItemCount++;

            if (accumulatedGroupTime >= groupDurationSeconds)
            {
                groupCount++;
                accumulatedGroupTime = 0;
                currentGroupItemCount = 0;
            }
        }

        if (currentGroupItemCount > 0)
        {
            groupCount++;
        }

        return groupCount;
    }
}