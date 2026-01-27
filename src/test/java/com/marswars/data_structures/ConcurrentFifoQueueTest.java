package com.marswars.data_structures;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

@DisplayName("ConcurrentFifoQueue Tests")
class ConcurrentFifoQueueTest {

    private ConcurrentFifoQueue<String> buffer;

    @BeforeEach
    void setUp() {
        buffer = new ConcurrentFifoQueue<String>(3);
    }

    @Test
    @DisplayName("Should create empty buffer with correct capacity")
    void testConstructor() {
        assertTrue(buffer.isEmpty());
        assertFalse(buffer.isFull());
        assertEquals(3, buffer.capacity());
        assertTrue(buffer.toList().isEmpty());
    }

    @Test
    @DisplayName("Should throw exception for invalid capacity")
    void testInvalidCapacity() {
        assertThrows(IllegalArgumentException.class, () -> {
            new ConcurrentFifoQueue<String>(0);
        });
        
        assertThrows(IllegalArgumentException.class, () -> {
            new ConcurrentFifoQueue<String>(-1);
        });
    }

    @Test
    @DisplayName("Should add elements until capacity is reached")
    void testAddUntilCapacity() {
        // Add first element
        buffer.add("A");
        assertFalse(buffer.isEmpty());
        assertFalse(buffer.isFull());
        assertEquals(List.of("A"), buffer.toList());

        // Add second element
        buffer.add("B");
        assertFalse(buffer.isEmpty());
        assertFalse(buffer.isFull());
        assertEquals(List.of("A", "B"), buffer.toList());

        // Add third element (reach capacity)
        buffer.add("C");
        assertFalse(buffer.isEmpty());
        assertTrue(buffer.isFull());
        assertEquals(List.of("A", "B", "C"), buffer.toList());
    }

    @Test
    @DisplayName("Should overwrite oldest elements when capacity exceeded")
    void testCircularOverwrite() {
        // Fill buffer
        buffer.add("A");
        buffer.add("B");
        buffer.add("C");
        assertEquals(List.of("A", "B", "C"), buffer.toList());

        // Add beyond capacity - should overwrite oldest (A)
        buffer.add("D");
        assertTrue(buffer.isFull());
        assertEquals(List.of("B", "C", "D"), buffer.toList());

        // Add another - should overwrite next oldest (B)
        buffer.add("E");
        assertTrue(buffer.isFull());
        assertEquals(List.of("C", "D", "E"), buffer.toList());
    }

    @Test
    @DisplayName("Should poll elements in FIFO order")
    void testFIFOPolling() {
        // Add elements
        buffer.add("A");
        buffer.add("B");
        buffer.add("C");

        // Poll in FIFO order
        assertEquals("A", buffer.poll());
        assertEquals(List.of("B", "C"), buffer.toList());

        assertEquals("B", buffer.poll());
        assertEquals(List.of("C"), buffer.toList());

        assertEquals("C", buffer.poll());
        assertTrue(buffer.isEmpty());
        assertTrue(buffer.toList().isEmpty());
    }

    @Test
    @DisplayName("Should return null when polling empty buffer")
    void testPollEmptyBuffer() {
        assertNull(buffer.poll());
        assertTrue(buffer.isEmpty());
    }

    @Test
    @DisplayName("Should maintain FIFO order after overflow")
    void testFIFOAfterOverflow() {
        // Fill and overflow
        buffer.add("A");
        buffer.add("B");
        buffer.add("C");
        buffer.add("D"); // Overwrites A
        buffer.add("E"); // Overwrites B
        
        // Should contain C, D, E in that order
        assertEquals(List.of("C", "D", "E"), buffer.toList());

        // Poll should return in FIFO order
        assertEquals("C", buffer.poll());
        assertEquals("D", buffer.poll());
        assertEquals("E", buffer.poll());
        assertTrue(buffer.isEmpty());
    }

    @Test
    @DisplayName("Should handle mixed add and poll operations")
    void testMixedOperations() {
        // Add some elements
        buffer.add("X");
        buffer.add("Y");
        assertEquals(List.of("X", "Y"), buffer.toList());

        // Poll one
        assertEquals("X", buffer.poll());
        assertEquals(List.of("Y"), buffer.toList());

        // Add more
        buffer.add("Z");
        buffer.add("W");
        assertEquals(List.of("Y", "Z", "W"), buffer.toList());

        // Poll one more
        assertEquals("Y", buffer.poll());
        assertEquals(List.of("Z", "W"), buffer.toList());
    }

    @Test
    @DisplayName("Should handle infinite adds with circular behavior")
    void testInfiniteAdds() {
        // Add many more elements than capacity
        for (int i = 1; i <= 10; i++) {
            buffer.add("Item" + i);
        }

        // Should contain only the last 3 elements
        assertEquals(List.of("Item8", "Item9", "Item10"), buffer.toList());
        assertTrue(buffer.isFull());

        // Poll should return in FIFO order
        assertEquals("Item8", buffer.poll());
        assertEquals("Item9", buffer.poll());
        assertEquals("Item10", buffer.poll());
        assertTrue(buffer.isEmpty());
    }

    @Test
    @DisplayName("Should handle null values correctly")
    void testNullValues() {
        // Note: In this implementation, null values are treated as "empty slots"
        // so they won't appear in toList() output but can still be polled
        buffer.add(null);
        buffer.add("B");
        buffer.add(null);

        // toList() only shows non-null values
        List<String> result = buffer.toList();
        assertEquals(1, result.size());
        assertEquals("B", result.get(0));

        // But polling should return actual values including nulls
        assertNull(buffer.poll()); // First null
        assertEquals("B", buffer.poll());
        assertNull(buffer.poll()); // Second null
        assertTrue(buffer.isEmpty());
    }

    @Test
    @DisplayName("Should maintain state consistency after multiple operations")
    void testStateConsistency() {
        // Perform various operations
        buffer.add("1");       // [1]
        buffer.add("2");       // [1, 2]
        assertEquals("1", buffer.poll()); // [2]
        buffer.add("3");       // [2, 3]
        buffer.add("4");       // [2, 3, 4] - buffer full
        buffer.add("5");       // [3, 4, 5] - overwrites "2", readIndex advances

        // After overwrite, we should have [3, 4, 5]
        assertEquals(List.of("3", "4", "5"), buffer.toList());
        assertFalse(buffer.isEmpty());
        assertTrue(buffer.isFull());

        // Poll one element
        assertEquals("3", buffer.poll()); // [4, 5]
        assertEquals(List.of("4", "5"), buffer.toList());
        assertFalse(buffer.isEmpty());
        assertFalse(buffer.isFull());

        // Add one more to fill buffer again
        buffer.add("6");       // [4, 5, 6]
        assertTrue(buffer.isFull());
        assertEquals(List.of("4", "5", "6"), buffer.toList());
    }
}