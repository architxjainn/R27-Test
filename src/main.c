#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include <math.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include "read.h"
#include "en_dc.h"
#include "read_file.h"
#include "drive.h"

#define NUM_PRODUCERS 1
#define NUM_CONSUMERS 3
#define MAX_MSG_DATA 8

/* Global structures and synchronization primitives */
Message_Queue queue;
Shared_Buffer shared_buffer;
ReadWrite_Lock lock;
pthread_mutex_t message_mutex;
pthread_cond_t message_available;
unsigned long message_generation = 0;
int producer_finished = 0;

/* Structure to pass file names and IDs to threads */
typedef struct {
    int id;
    const char *filename;
    const char *result_filename;
} FileArgs;

/* Producer thread: reads coordinates, encodes them into a message and pushes to queue */
void *producer(void *arg) {
    InputFile input;
    FileArgs *args = (FileArgs *)arg;

    if (input_file_open(&input, args->filename) != 0) {
        fprintf(stderr, "Producer: cannot open %s\n", args->filename);
        return NULL;
    }

    float x_coord, y_coord;
    int msg_id = 0;

    while (input_file_read(&input, &x_coord, &y_coord)) {
        Message msg = {0};

        /*
         * Convert coordinates into a transport message.
         * We pack two floats into a byte array (simple scaling).
         */
        msg.id = msg_id++;
        msg.len = sizeof(float) * 2;
        // Scale coordinates to avoid floating point issues, store as int16_t
        int16_t x_int = (int16_t)(x_coord * 1000.0f);
        int16_t y_int = (int16_t)(y_coord * 1000.0f);
        msg.data[0] = (uint8_t)(x_int & 0xFF);
        msg.data[1] = (uint8_t)((x_int >> 8) & 0xFF);
        msg.data[2] = (uint8_t)(y_int & 0xFF);
        msg.data[3] = (uint8_t)((y_int >> 8) & 0xFF);
        msg.len = 4;  // two int16_t

        /*
         * Store the message in the shared buffer safely.
         * Only one message at a time is pushed; queue handles concurrency.
         */
        pthread_mutex_lock(&message_mutex);
        // Wait if queue is full (optional, but queue_push uses semaphores)
        while (message_queue_is_full(&queue)) {
            pthread_cond_wait(&message_available, &message_mutex);
        }
        pthread_mutex_unlock(&message_mutex);

        // Push the message into the thread-safe queue
        if (message_queue_push(&queue, &msg) != 0) {
            fprintf(stderr, "Producer: failed to push message\n");
            continue;
        }

        /*
         * Notify waiting consumers.
         */
        pthread_cond_signal(&message_available);

        printf("Producer: pushed msg %d (%.2f, %.2f)\n", msg.id, x_coord, y_coord);
    }

    input_file_close(&input);

    /*
     * Notify consumers that production has finished.
     */
    pthread_mutex_lock(&message_mutex);
    producer_finished = 1;
    pthread_cond_broadcast(&message_available);
    pthread_mutex_unlock(&message_mutex);

    printf("Producer: finished\n");
    return NULL;
}

/* Consumer thread: retrieves messages, decodes, and forwards target to drive (only first consumer) */
void *consumer(void *arg) {
    int id = *(int *)arg;
    Message msg;
    float x_coord, y_coord;

    while (1) {
        /*
         * Wait for a new message.
         */
        pthread_mutex_lock(&message_mutex);
        while (message_queue_is_empty(&queue) && !producer_finished) {
            pthread_cond_wait(&message_available, &message_mutex);
        }
        // If queue empty and producer finished, exit
        if (message_queue_is_empty(&queue) && producer_finished) {
            pthread_mutex_unlock(&message_mutex);
            break;
        }
        pthread_mutex_unlock(&message_mutex);

        /*
         * Safely retrieve the message from the shared buffer.
         */
        if (message_queue_pop(&queue, &msg) != 0) {
            continue;
        }

        /*
         * Decode the message.
         */
        int16_t x_int = (int16_t)(msg.data[0] | (msg.data[1] << 8));
        int16_t y_int = (int16_t)(msg.data[2] | (msg.data[3] << 8));
        x_coord = x_int / 1000.0f;
        y_coord = y_int / 1000.0f;

        printf("Consumer %d: decoded target (%.2f, %.2f)\n", id, x_coord, y_coord);

        /*
         * Forward the message to the drive queue.
         * Only consumer with id == 1 forwards to avoid duplicates.
         */
        if (id == 1) {
            struct coordinate target;
            target.latitude = y_coord;
            target.longitude = x_coord;
            target.altitude = 0.0f;

            // Store target in shared_buffer for drive thread
            pthread_mutex_lock(&message_mutex);
            shared_buffer.target_coord = target;
            shared_buffer.has_target = 1;
            pthread_cond_signal(&message_available); // wake up drive thread if waiting
            pthread_mutex_unlock(&message_mutex);
            printf("Consumer %d: forwarded target to drive\n", id);
        }
    }

    printf("Consumer %d: finished\n", id);
    return NULL;
}

/* Drive thread: waits for a target, runs drive_to_target, writes results to file */
void *drive_write(void *arg) {
    FileArgs *args = (FileArgs *)arg;
    InputFile output_file;
    struct coordinate target;
    struct rover_state rover = {0}; // initial position (0,0), heading 0
    enum drive_status result_status;
    int status = 1; // 0 = success, 1 = failure

    if (input_file_open_write(&output_file, args->result_filename) != 0) {
        fprintf(stderr, "Drive: cannot open %s for writing\n", args->result_filename);
        return NULL;
    }

    /* Wait for a target to be set */
    pthread_mutex_lock(&message_mutex);
    while (!shared_buffer.has_target) {
        pthread_cond_wait(&message_available, &message_mutex);
    }
    target = shared_buffer.target_coord;
    shared_buffer.has_target = 0; // consume the target
    pthread_mutex_unlock(&message_mutex);

    printf("Drive: target received (%.2f, %.2f)\n", target.longitude, target.latitude);

    /*
     * Invoke the drive_to_target function.
     */
    result_status = drive_to_target(&rover, &target);

    // Write results: we write 10 steps or until reached
    for (int i = 0; i < 10; i++) {
        float dx = target.longitude - rover.position.longitude;
        float dy = target.latitude - rover.position.latitude;
        float error = hypotf(dx, dy);

        if (result_status == DRIVE_REACHED_TARGET && error <= 0.7f) {
            status = 0;  // success
        } else if (result_status == DRIVE_REACHED_TARGET && error > 0.7f) {
            status = 1;  // reached but error still large? (shouldn't happen)
        } else {
            status = 1;  // failure (max steps or invalid)
        }

        input_file_write(&output_file,
                         &rover.position.latitude,
                         &rover.position.longitude,
                         &error,
                         &status);

        if (status == 0) break;
        usleep(20000); // 20ms delay between steps
    }

    input_file_close(&output_file);

    if (status == 0) {
        printf("Drive: Successfully reached target!\n");
    } else {
        printf("Drive: Failed (status %d)\n", result_status);
    }

    return NULL;
}

int main() {
    pthread_t producers[NUM_PRODUCERS];
    pthread_t consumers[NUM_CONSUMERS];
    pthread_t drive_writers[NUM_PRODUCERS];
    int consumer_id[NUM_CONSUMERS] = {1, 2, 3};

    const char *testcases[] = {
        "input/testcase1.txt",
        "input/testcase2.txt",
        "input/testcase3.txt",
        "input/testcase4.txt"
    };
    const char *result_tc[] = {
        "result/result1.txt",
        "result/result2.txt",
        "result/result3.txt",
        "result/result4.txt"
    };

    /* Initialize synchronization primitives */
    if (rwlock_init(&lock) != 0) {
        printf("Reader-writer lock init failed\n");
        return 1;
    }
    if (message_queue_init(&queue) != 0) {
        printf("Queue init failed\n");
        return 1;
    }
    if (pthread_cond_init(&message_available, NULL) != 0) {
        printf("Condition variable init failed\n");
        return 1;
    }
    if (pthread_mutex_init(&message_mutex, NULL) != 0) {
        printf("Mutex init failed\n");
        return 1;
    }

    /* Process each test case */
    for (int tc = 0; tc < 4; tc++) {
        printf("\n========== Test case %d: %s ==========\n", tc+1, testcases[tc]);

        FileArgs file_args = {
            .id = 1,
            .filename = testcases[tc],
            .result_filename = result_tc[tc]
        };

        // Reset global state for each test case
        producer_finished = 0;
        shared_buffer.has_target = 0;
        message_queue_reset(&queue);  // optional: clear the queue

        // Create threads
        for (int i = 0; i < NUM_PRODUCERS; i++) {
            pthread_create(&producers[i], NULL, producer, &file_args);
        }
        for (int i = 0; i < NUM_CONSUMERS; i++) {
            pthread_create(&consumers[i], NULL, consumer, &consumer_id[i]);
        }
        for (int i = 0; i < NUM_PRODUCERS; i++) {
            pthread_create(&drive_writers[i], NULL, drive_write, &file_args);
        }

        // Wait for all threads to finish
        for (int i = 0; i < NUM_PRODUCERS; i++) {
            pthread_join(producers[i], NULL);
        }
        for (int i = 0; i < NUM_CONSUMERS; i++) {
            pthread_join(consumers[i], NULL);
        }
        for (int i = 0; i < NUM_PRODUCERS; i++) {
            pthread_join(drive_writers[i], NULL);
        }

        printf("Test case %d completed\n\n", tc+1);
    }

    /* Cleanup */
    rwlock_destroy(&lock);
    message_destroy(&queue);
    pthread_cond_destroy(&message_available);
    pthread_mutex_destroy(&message_mutex);

    return 0;
}
