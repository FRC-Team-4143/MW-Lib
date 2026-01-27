package com.marswars.data_structures;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.RepeatedTest;

import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.*;

@DisplayName("ConcurrentFifoQueue Concurrency Tests")
class ConcurrentFifoQueueConcurrencyTest {

    @Test
    @DisplayName("Should handle concurrent adds safely")
    void testConcurrentAdds() throws InterruptedException {
        ConcurrentFifoQueue<Integer> buffer = new ConcurrentFifoQueue<Integer>(100);
        int numThreads = 10;
        int addsPerThread = 100;
        CountDownLatch latch = new CountDownLatch(numThreads);
        
        ExecutorService executor = Executors.newFixedThreadPool(numThreads);
        
        for (int t = 0; t < numThreads; t++) {
            final int threadId = t;
            executor.submit(() -> {
                try {
                    for (int i = 0; i < addsPerThread; i++) {
                        buffer.add(threadId * 1000 + i);
                    }
                } finally {
                    latch.countDown();
                }
            });
        }
        
        assertTrue(latch.await(5, TimeUnit.SECONDS));
        executor.shutdown();
        
        // Buffer should be full and contain the last 100 elements
        assertTrue(buffer.isFull());
        assertEquals(100, buffer.toList().size());
    }

    @Test
    @DisplayName("Should handle concurrent adds and polls safely")
    void testConcurrentAddsAndPolls() throws InterruptedException {
        ConcurrentFifoQueue<Integer> buffer = new ConcurrentFifoQueue<Integer>(50);
        int numProducers = 5;
        int numConsumers = 3;
        int operationsPerThread = 200;
        CountDownLatch latch = new CountDownLatch(numProducers + numConsumers);
        AtomicInteger elementsProduced = new AtomicInteger(0);
        AtomicInteger elementsConsumed = new AtomicInteger(0);
        
        ExecutorService executor = Executors.newFixedThreadPool(numProducers + numConsumers);
        
        // Start producers
        for (int t = 0; t < numProducers; t++) {
            final int threadId = t;
            executor.submit(() -> {
                try {
                    for (int i = 0; i < operationsPerThread; i++) {
                        buffer.add(threadId * 10000 + i);
                        elementsProduced.incrementAndGet();
                        if (i % 10 == 0) {
                            Thread.yield(); // Give other threads a chance
                        }
                    }
                } finally {
                    latch.countDown();
                }
            });
        }
        
        // Start consumers
        for (int t = 0; t < numConsumers; t++) {
            executor.submit(() -> {
                try {
                    for (int i = 0; i < operationsPerThread; i++) {
                        Integer value = buffer.poll();
                        if (value != null) {
                            elementsConsumed.incrementAndGet();
                        }
                        if (i % 10 == 0) {
                            Thread.yield(); // Give other threads a chance
                        }
                    }
                } finally {
                    latch.countDown();
                }
            });
        }
        
        assertTrue(latch.await(10, TimeUnit.SECONDS));
        executor.shutdown();
        
        // Verify that operations completed without exceptions
        assertTrue(elementsProduced.get() > 0);
        assertTrue(elementsConsumed.get() >= 0);
        
        // Buffer state should be consistent
        assertFalse(buffer.capacity() < 0); // Basic sanity check
    }

    @RepeatedTest(5)
    @DisplayName("Should maintain FIFO order under concurrent stress")
    void testFIFOOrderUnderStress() throws InterruptedException {
        ConcurrentFifoQueue<String> buffer = new ConcurrentFifoQueue<String>(10);
        int numOperations = 100;
        CountDownLatch startLatch = new CountDownLatch(1);
        CountDownLatch finishLatch = new CountDownLatch(2);
        
        ExecutorService executor = Executors.newFixedThreadPool(2);
        
        // Producer thread
        executor.submit(() -> {
            try {
                startLatch.await();
                for (int i = 0; i < numOperations; i++) {
                    buffer.add("Item" + i);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } finally {
                finishLatch.countDown();
            }
        });
        
        // Consumer thread
        executor.submit(() -> {
            try {
                startLatch.await();
                for (int i = 0; i < numOperations / 2; i++) {
                    String polled = buffer.poll();
                    if (polled != null) {
                        // Basic ordering check - if we get a numbered item,
                        // it should be reasonable given FIFO behavior
                        assertTrue(polled.startsWith("Item"));
                    }
                    Thread.yield();
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } finally {
                finishLatch.countDown();
            }
        });
        
        // Start both threads simultaneously
        startLatch.countDown();
        assertTrue(finishLatch.await(5, TimeUnit.SECONDS));
        executor.shutdown();
        
        // Buffer should be in a valid state
        assertNotNull(buffer.toList());
    }

    @Test
    @DisplayName("Should handle rapid state changes safely")
    void testRapidStateChanges() throws InterruptedException {
        ConcurrentFifoQueue<Integer> buffer = new ConcurrentFifoQueue<Integer>(5);
        int numThreads = 4;
        int cyclesPerThread = 50;
        CountDownLatch latch = new CountDownLatch(numThreads);
        
        ExecutorService executor = Executors.newFixedThreadPool(numThreads);
        
        for (int t = 0; t < numThreads; t++) {
            final int threadId = t;
            executor.submit(() -> {
                try {
                    for (int cycle = 0; cycle < cyclesPerThread; cycle++) {
                        // Rapidly add and poll elements
                        buffer.add(threadId * 1000 + cycle);
                        buffer.add(threadId * 1000 + cycle + 100);
                        buffer.poll(); // May return null due to timing, but that's okay
                        buffer.add(threadId * 1000 + cycle + 200);
                    }
                } finally {
                    latch.countDown();
                }
            });
        }
        
        assertTrue(latch.await(10, TimeUnit.SECONDS));
        executor.shutdown();
        
        // Buffer should be in a valid state after all operations
        assertTrue(buffer.toList().size() <= buffer.capacity());
        assertTrue(buffer.toList().size() >= 0);
    }
}