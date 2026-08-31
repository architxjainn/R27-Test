#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include "read.h"

#define QUEUE_SIZE 50

int message_queue_init(Message_Queue *queue) {
    queue->head = 0;
    queue->tail = 0;
    
    if (pthread_mutex_init(&queue->mutex, NULL) != 0) {
        return -1;
    }
    
    // Initialize semaphore for empty slots (queue empty initially)
    if (sem_init(&queue->empty, 0, QUEUE_SIZE) != 0) {
        pthread_mutex_destroy(&queue->mutex);
        return -1;
    }
    
    // Initialize semaphore for full slots (0 initially)
    if (sem_init(&queue->full, 0, 0) != 0) {
        sem_destroy(&queue->empty);
        pthread_mutex_destroy(&queue->mutex);
        return -1;
    }
    
    return 0;
}

void message_destroy(Message_Queue *queue) {
    pthread_mutex_destroy(&queue->mutex);
    sem_destroy(&queue->full);
    sem_destroy(&queue->empty);
}

int message_queue_push(Message_Queue *queue, const Message *msg) {
    /*
     * Make the logic for adding an element to the queue only when the queue is
     * available and not full.
     * Make the queue in such a way that it does not waste any memory and no
     * extra memory is required.
     */
    
    // Validate inputs
    if (queue == NULL || msg == NULL) {
        return -1;
    }
    
    // Wait for an empty slot in the queue
    // This blocks if the queue is full
    sem_wait(&queue->empty);
    
    // Lock the mutex for exclusive access to the queue
    pthread_mutex_lock(&queue->mutex);
    
    // Add the message to the queue at the tail position
    queue->buffer[queue->tail] = *msg;
    
    // Update the tail pointer (circular buffer)
    queue->tail = (queue->tail + 1) % QUEUE_SIZE;
    
    // Unlock the mutex
    pthread_mutex_unlock(&queue->mutex);
    
    // Signal that there is a new full slot in the queue
    sem_post(&queue->full);
    
    return 0;
}

int message_queue_pop(Message_Queue *queue, Message *msg) {
    // Validate inputs
    if (queue == NULL || msg == NULL) {
        return -1;
    }
    
    // Wait for a full slot in the queue
    // This blocks if the queue is empty
    sem_wait(&queue->full);
    
    // Lock the mutex for exclusive access to the queue
    pthread_mutex_lock(&queue->mutex);
    
    // Retrieve the message from the queue at the head position
    *msg = queue->buffer[queue->head];
    
    // Update the head pointer (circular buffer)
    queue->head = (queue->head + 1) % QUEUE_SIZE;
    
    // Unlock the mutex
    pthread_mutex_unlock(&queue->mutex);
    
    // Signal that there is a new empty slot in the queue
    sem_post(&queue->empty);
    
    return 0;
}

// Optional: Function to check if queue is empty
int message_queue_is_empty(const Message_Queue *queue) {
    if (queue == NULL) {
        return -1;
    }
    int empty_count;
    sem_getvalue(&queue->full, &empty_count);
    return (empty_count == 0);
}

// Optional: Function to check if queue is full
int message_queue_is_full(const Message_Queue *queue) {
    if (queue == NULL) {
        return -1;
    }
    int empty_count;
    sem_getvalue(&queue->empty, &empty_count);
    return (empty_count == 0);
}

// Optional: Function to get current queue size
int message_queue_size(const Message_Queue *queue) {
    if (queue == NULL) {
        return -1;
    }
    int full_count;
    sem_getvalue(&queue->full, &full_count);
    return full_count;
}

// Optional: Function to reset the queue (clear all messages)
void message_queue_reset(Message_Queue *queue) {
    if (queue == NULL) {
        return;
    }
    
    // Wait for all slots to be empty
    for (int i = 0; i < QUEUE_SIZE; i++) {
        sem_wait(&queue->full);
    }
    
    pthread_mutex_lock(&queue->mutex);
    queue->head = 0;
    queue->tail = 0;
    pthread_mutex_unlock(&queue->mutex);
    
    // Reset empty semaphore to full capacity
    int current_empty;
    sem_getvalue(&queue->empty, &current_empty);
    for (int i = current_empty; i < QUEUE_SIZE; i++) {
        sem_post(&queue->empty);
    }
}
