package com.marswars.data_structures;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class FixedAtomicSinglyLinkedList<T> {

    /* =========================
       Internal Node Definition
       ========================= */
    private static final class Node<T> {
        T value;
        final AtomicReference<Node<T>> next = new AtomicReference<>(null);
    }

    /* =========================
       Fields
       ========================= */
    private final Node<T>[] nodes;
    private final AtomicInteger allocationIndex = new AtomicInteger(0);

    private final AtomicReference<Node<T>> head = new AtomicReference<>(null);
    private final AtomicReference<Node<T>> tail = new AtomicReference<>(null);

    /* =========================
       Constructor
       ========================= */
    @SuppressWarnings("unchecked")
    public FixedAtomicSinglyLinkedList(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("Capacity must be > 0");
        }

        nodes = (Node<T>[]) new Node[capacity];
        for (int i = 0; i < capacity; i++) {
            nodes[i] = new Node<>();
        }
    }

    /* =========================
       Add (bounded, lock-free)
       ========================= */
    public boolean add(T value) {
        int index = allocationIndex.getAndIncrement();
        if (index >= nodes.length) {
            return false; // fixed capacity reached
        }

        Node<T> newNode = nodes[index];
        newNode.value = value;
        newNode.next.set(null);

        while (true) {
            Node<T> currentTail = tail.get();

            // Empty list
            if (currentTail == null) {
                if (head.compareAndSet(null, newNode)) {
                    tail.set(newNode);
                    return true;
                }
            } else {
                // Link new node
                if (currentTail.next.compareAndSet(null, newNode)) {
                    tail.compareAndSet(currentTail, newNode);
                    return true;
                } else {
                    // Help advance tail
                    tail.compareAndSet(currentTail, currentTail.next.get());
                }
            }
        }
    }

    /* =========================
       Poll (destructive removal)
       Overwrites next pointer
       ========================= */
    public T poll() {
        while (true) {
            Node<T> currentHead = head.get();
            if (currentHead == null) {
                return null;
            }

            Node<T> next = currentHead.next.get();

            if (head.compareAndSet(currentHead, next)) {
                // Fix tail if list becomes empty
                if (next == null) {
                    tail.compareAndSet(currentHead, null);
                }

                T value = currentHead.value;

                // 🔥 destructive cleanup
                currentHead.value = null;
                currentHead.next.set(null);

                return value;
            }
        }
    }

    /* =========================
       Snapshot traversal
       ========================= */
    public List<T> toList() {
        List<T> result = new ArrayList<>();

        Node<T> current = head.get();
        while (current != null) {
            T value = current.value;
            if (value != null) {
                result.add(value);
            }
            current = current.next.get();
        }

        return result;
    }

    /* =========================
       Utility Methods
       ========================= */
    public boolean isEmpty() {
        return head.get() == null;
    }

    public int capacity() {
        return nodes.length;
    }

    public int sizeUsed() {
        return Math.min(allocationIndex.get(), nodes.length);
    }
}
