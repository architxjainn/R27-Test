#include <stdio.h>
#include <semaphore.h>
#include <pthread.h>
#include <stdint.h>
#include <unistd.h>

#include "read.h"

int rwlock_init(ReadWrite_Lock *rw){
    rw->reader = 0;

    if (pthread_mutex_init(&rw->reader_count, NULL) != 0)
        return -1;

    if (pthread_mutex_init(&rw->writer_count, NULL) != 0) {
        pthread_mutex_destroy(&rw->reader_count);
        return -1;
    }

    if (sem_init(&rw->resource, 0, 1) != 0) {
        pthread_mutex_destroy(&rw->reader_count);
        pthread_mutex_destroy(&rw->writer_count);
        return -1;
    }

    return 0;
}

/*
 * Reader Entry
 *
 * TODO:
 * - Implement the reader-side synchronization logic.
 * - Multiple readers should be able to access the shared resource
 *   concurrently.
 * - The first reader must ensure that a writer cannot access the
 *   resource while readers are active.
 * - Ensure reader_count is modified safely.
 * - Ensure all acquired synchronization primitives are released
 *   correctly.
 */
void reader_enter(ReadWrite_Lock *lock){
    // Lock writer_count mutex to prevent writers from entering
    // This ensures writers are blocked while readers are active
    pthread_mutex_lock(&lock->writer_count);

    // Lock reader_count mutex to safely modify reader count
    pthread_mutex_lock(&lock->reader_count);

    // Increment the number of active readers
    lock->reader++;

    // If this is the first reader (reader count was 0 before increment),
    // we need to acquire the resource semaphore to block writers
    if (lock->reader == 1) {
        // The first reader acquires the resource semaphore
        // This prevents writers from accessing the resource
        sem_wait(&lock->resource);
    }

    // Unlock reader_count mutex (other readers can now increment count)
    pthread_mutex_unlock(&lock->reader_count);

    // Unlock writer_count mutex (other readers can now enter)
    pthread_mutex_unlock(&lock->writer_count);
}

/*
 * Reader Exit
 *
 * TODO:
 * - Implement the reader exit logic.
 * - Decrement the active reader count safely.
 * - Ensure the resource becomes available to writers when
 *   the last reader exits.
 */
void reader_exit(ReadWrite_Lock *rw){
    // Lock reader_count mutex to safely modify reader count
    pthread_mutex_lock(&rw->reader_count);

    // Decrement the number of active readers
    rw->reader--;

    // If this was the last reader (reader count is now 0),
    // release the resource semaphore to allow writers
    if(rw->reader == 0){
        // Release the resource semaphore so writers can access
        sem_post(&rw->resource);
    }

    // Unlock reader_count mutex
    pthread_mutex_unlock(&rw->reader_count);
}

/*
 * Writer Entry
 *
 * TODO:
 * - Ensure writers obtain exclusive access to the shared resource.
 * - Prevent writers from accessing the resource while readers
 *   are active.
 * - Ensure writer synchronization is handled correctly.
 */
void writer_enter(ReadWrite_Lock *lock){
    // Lock writer_count mutex to ensure exclusive writer access
    // This prevents other writers from entering simultaneously
    pthread_mutex_lock(&lock->writer_count);
    
    // Wait for the resource semaphore
    // This blocks if there are active readers or another writer
    // The semaphore ensures exclusive access to the resource
    sem_wait(&lock->resource);
}

/*
 * Writer Exit
 *
 * TODO:
 * - Release the shared resource.
 * - Release any synchronization primitive acquired by writer_enter().
 */
void writer_exit(ReadWrite_Lock *lock){
    // Release the resource semaphore to allow other threads to access
    // This could be another writer or multiple readers
    sem_post(&lock->resource);
    
    // Unlock writer_count mutex to allow other writers to enter
    pthread_mutex_unlock(&lock->writer_count);
}

void rwlock_destroy(ReadWrite_Lock *rw)
{
    pthread_mutex_destroy(&rw->reader_count);
    pthread_mutex_destroy(&rw->writer_count);
    sem_destroy(&rw->resource);
}
