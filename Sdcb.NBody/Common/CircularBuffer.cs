using System.Collections;

namespace Sdcb.NBody.Common;

/// <summary>
/// 表示一个固定容量的循环缓冲区（或环形列表）。
/// 当缓冲区满时，添加新元素会覆盖最早的元素。
/// </summary>
/// <typeparam name="T">缓冲区中元素的类型。</typeparam>
public class CircularBuffer<T> : IEnumerable<T>
{
    private readonly T[] _data;
    private int _end; // 指向下一个要写入元素的位置（尾部）

    /// <summary>
    /// 获取缓冲区中实际存储的元素数量。
    /// </summary>
    public int Count { get; private set; }

    /// <summary>
    /// 获取缓冲区的总容量。
    /// </summary>
    public int Capacity => _data.Length;

    /// <summary>
    /// 获取一个值，该值指示缓冲区是否已满。
    /// </summary>
    public bool IsFull => Count == Capacity;

    /// <summary>
    /// 计算并获取第一个元素（头部）在内部数组中的索引。
    /// </summary>
    private int HeadIndex
    {
        get
        {
            if (Count == 0) return 0;
            // 当 Count < Capacity 时，_end 就是 Count，结果为 0。
            // 当缓冲区满时 (Count == Capacity)，_end 会循环，这个公式能正确计算出头部的索引。
            return (_end - Count + Capacity) % Capacity;
        }
    }

    /// <summary>
    /// 初始化 <see cref="CircularBuffer{T}"/> 类的新实例。
    /// </summary>
    /// <param name="capacity">缓冲区的容量。必须为正数。</param>
    /// <exception cref="ArgumentException">当容量小于等于 0 时抛出。</exception>
    public CircularBuffer(int capacity)
    {
        if (capacity <= 0)
        {
            throw new ArgumentException("Capacity must be a positive number.", nameof(capacity));
        }
        _data = new T[capacity];
        _end = 0;
        Count = 0;
    }

    /// <summary>
    /// 将一个新元素添加到缓冲区的尾部。
    /// 如果缓冲区已满，此操作会覆盖最早的元素。
    /// </summary>
    /// <param name="item">要添加的元素。</param>
    public void Add(T item)
    {
        _data[_end] = item;
        _end = (_end + 1) % Capacity;

        if (Count < Capacity)
        {
            Count++;
        }
    }

    /// <summary>
    /// 清空缓冲区中的所有元素。
    /// </summary>
    public void Clear()
    {
        // 只需重置计数器和指针即可，无需清除数组数据
        Count = 0;
        _end = 0;
    }

    /// <summary>
    /// 获取或设置指定逻辑索引处的元素。
    /// 索引 0 是最早的元素，索引 Count-1 是最新的元素。
    /// </summary>
    /// <param name="index">元素的逻辑索引。</param>
    /// <exception cref="IndexOutOfRangeException">当索引超出范围时抛出。</exception>
    public T this[int index]
    {
        get
        {
            if (index < 0 || index >= Count)
            {
                throw new IndexOutOfRangeException("Index is out of the valid range of the buffer.");
            }
            int actualIndex = (HeadIndex + index) % Capacity;
            return _data[actualIndex];
        }
        set
        {
            if (index < 0 || index >= Count)
            {
                throw new IndexOutOfRangeException("Index is out of the valid range of the buffer.");
            }
            int actualIndex = (HeadIndex + index) % Capacity;
            _data[actualIndex] = value;
        }
    }

    /// <summary>
    /// 获取缓冲区中的第一个（最早的）元素。
    /// </summary>
    /// <exception cref="InvalidOperationException">当缓冲区为空时抛出。</exception>
    public T First
    {
        get
        {
            if (Count == 0) throw new InvalidOperationException("Buffer is empty.");
            return _data[HeadIndex];
        }
    }

    /// <summary>
    /// 获取缓冲区中的最后一个（最新的）元素。
    /// </summary>
    /// <exception cref="InvalidOperationException">当缓冲区为空时抛出。</exception>
    public T Last
    {
        get
        {
            if (Count == 0) throw new InvalidOperationException("Buffer is empty.");
            // _end 指向下一个要写入的位置，所以上一个写入的位置是 (_end - 1)
            int lastIndex = (_end - 1 + Capacity) % Capacity;
            return _data[lastIndex];
        }
    }

    /// <summary>
    /// 返回一个循环访问集合的枚举器。
    /// </summary>
    public IEnumerator<T> GetEnumerator()
    {
        int head = HeadIndex;
        for (int i = 0; i < Count; i++)
        {
            yield return _data[(head + i) % Capacity];
        }
    }

    /// <summary>
    /// 返回一个循环访问集合的枚举器。
    /// </summary>
    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }
}
