using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;

namespace Sdcb.NBody.Common;

/// <summary>
/// Represents a fixed-capacity circular buffer that implements IReadOnlyList<T>.
/// When the buffer is full, adding a new element overwrites the oldest element.
/// </summary>
/// <typeparam name="T">The type of elements in the buffer.</typeparam>
public class CircularBuffer<T> : IReadOnlyList<T>
{
    private readonly T[] _data;
    private int _end; // 指向下一个要写入的位置 (尾部)
    private int _start; // 指向第一个元素 (头部)

    /// <summary>
    /// Gets the number of elements contained in the buffer.
    /// </summary>
    public int Count { get; private set; }

    /// <summary>
    /// Gets the total capacity of the buffer.
    /// </summary>
    public int Capacity => _data.Length;

    /// <summary>
    /// Gets a value indicating whether the buffer is full.
    /// </summary>
    public bool IsFull => Count == Capacity;

    public CircularBuffer(int capacity)
    {
        if (capacity <= 0)
        {
            throw new ArgumentException("Capacity must be a positive number.", nameof(capacity));
        }
        _data = new T[capacity];
        _start = 0;
        _end = 0;
        Count = 0;
    }

    /// <summary>
    /// Adds an element to the buffer. If the buffer is full, the oldest element is overwritten.
    /// </summary>
    /// <param name="item">The element to add.</param>
    public void Add(T item)
    {
        _data[_end] = item;
        _end = (_end + 1) % Capacity;

        if (IsFull)
        {
            _start = (_start + 1) % Capacity; // 覆盖：移动头部指针
        }
        else
        {
            Count++;
        }
    }

    /// <summary>
    /// Clears the buffer.
    /// </summary>
    public void Clear()
    {
        Count = 0;
        _start = 0;
        _end = 0;
        Array.Clear(_data, 0, _data.Length);
    }

    /// <summary>
    /// Gets or sets the element at the specified index.
    /// </summary>
    /// <param name="index">The zero-based index of the element to get or set.</param>
    /// <returns>The element at the specified index.</returns>
    public T this[int index]
    {
        get
        {
            if (index < 0 || index >= Count)
            {
                throw new IndexOutOfRangeException("Index is out of the valid range of the buffer.");
            }
            int actualIndex = (_start + index) % Capacity;
            return _data[actualIndex];
        }
        set
        {
            if (index < 0 || index >= Count)
            {
                throw new IndexOutOfRangeException("Index is out of the valid range of the buffer.");
            }
            int actualIndex = (_start + index) % Capacity;
            _data[actualIndex] = value;
        }
    }

    /// <summary>
    /// Gets the first element in the buffer.
    /// </summary>
    public T First
    {
        get
        {
            if (Count == 0) throw new InvalidOperationException("Buffer is empty.");
            return _data[_start];
        }
    }

    /// <summary>
    /// Gets the last element in the buffer.
    /// </summary>
    public T Last
    {
        get
        {
            if (Count == 0) throw new InvalidOperationException("Buffer is empty.");
            int lastIndex = (_end == 0) ? Capacity - 1 : _end - 1;
            return _data[lastIndex];
        }
    }

    /// <summary>
    /// Returns an enumerator that iterates through the buffer.
    /// </summary>
    public IEnumerator<T> GetEnumerator()
    {
        for (int i = 0; i < Count; i++)
        {
            yield return this[i];
        }
    }

    /// <summary>
    /// Returns an enumerator that iterates through the buffer.
    /// </summary>
    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }

    /// <summary>
    /// Returns an enumerator that iterates through the buffer in reverse order.
    /// </summary>
    public IEnumerable<T> Reverse()
    {
        for (int i = Count - 1; i >= 0; i--)
        {
            yield return this[i];
        }
    }
}
