package com.marswars.data_structures;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class ConcurrentFifoQueue<T> {

    /* =========================
       Fields
       ========================= */
    private final AtomicReference<T>[] buffer;
    private final int capacity;
    private final AtomicInteger writeIndex = new AtomicInteger(0);
    private final AtomicInteger readIndex = new AtomicInteger(0);
    private final AtomicInteger elementsWritten = new AtomicInteger(0);

    /* =========================
       Constructor
       ========================= */
    @SuppressWarnings("unchecked")
    public ConcurrentFifoQueue(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("Capacity must be > 0");
        }

        this.capacity = capacity;
        buffer = (AtomicReference<T>[]) new AtomicReference[capacity];
        for (int i = 0; i < capacity; i++) {
            buffer[i] = new AtomicReference<>(null);
        }
    }

    /* =========================
       Add (circular buffer, lock-free FIFO)
       ========================= */
    public void add(T value) {
        // Get the next write position in circular fashion
        int index = writeIndex.getAndUpdate(i -> (i + 1) % capacity);
        
        // Check if we're overwriting an existing value
        T oldValue = buffer[index].getAndSet(value);
        
        // Track total elements written for determining if buffer has been filled
        elementsWritten.incrementAndGet();
        
        if (oldValue != null) {
            // We overwrote data, advance read pointer to maintain FIFO order
            // The read pointer should point to the oldest valid data
            readIndex.set((index + 1) % capacity);
        }
    }

    /* =========================
       Poll (FIFO removal from oldest element)
       ========================= */
    public T poll() {
        // Check if buffer has any elements
        if (elementsWritten.get() == 0) {
            return null;
        }
        
        // Get the oldest element (FIFO)
        int index = readIndex.getAndUpdate(i -> (i + 1) % capacity);
        T value = buffer[index].getAndSet(null);
        
        if (value != null) {
            return value;
        }
        
        // If we got null, the buffer might be in an inconsistent state
        // This shouldn't happen in normal operation, but let's handle it
        return null;
    }

    /* =========================
       Snapshot traversal (FIFO order)
       ========================= */
    public List<T> toList() {
        List<T> result = new ArrayList<>();
        
        // If no elements have been written, return empty list
        if (elementsWritten.get() == 0) {
            return result;
        }
        
        // Start from read index and collect all non-null elements
        int currentReadIndex = readIndex.get();
        int maxElements = Math.min(elementsWritten.get(), capacity);
        
        for (int i = 0; i < maxElements; i++) {
            int index = (currentReadIndex + i) % capacity;
            T value = buffer[index].get();
            if (value != null) {
                result.add(value);
            }
        }
        
        return result;
    }

    /* =========================
       Utility Methods
       ========================= */
    public boolean isEmpty() {
        // Check if any elements have been written and if all slots are null
        if (elementsWritten.get() == 0) {
            return true;
        }
        
        // Quick check: scan buffer for any non-null elements
        for (int i = 0; i < capacity; i++) {
            if (buffer[i].get() != null) {
                return false;
            }
        }
        return true;
    }

    public int capacity() {
        return capacity;
    }

    public boolean isFull() {
        // Buffer is full if we've written at least capacity elements
        // and all slots are occupied
        if (elementsWritten.get() < capacity) {
            return false;
        }
        
        // Check if all slots are occupied
        for (int i = 0; i < capacity; i++) {
            if (buffer[i].get() == null) {
                return false;
            }
        }
        return true;
    }
}
