#include "ring_buffer.h"


/**
 * @brief Initializes the ring buffer variable to their initial values.
 * @param rb Pointer to the ring buffer structure
 * @param buffer Pointer to the memory buffer used for the ring buffer
 * @param capacity The maximum number of elements the buffer can hold
 */ 
void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t capacity) 
{
    rb->buffer = buffer;  // Set the buffer pointer to the provided memory
    rb->head = 0;         // Initialize head index to 0
    rb->tail = 0;         // Initialize tail index to 0
    rb->capacity = capacity; // Set the capacity of the ring buffer
    rb->if_full = false;     // Initialize the full indicator to false (not full)

}

/**
 * @brief Writes a byte of data to the ring buffer, discards the old data if is full.
 * @param rb Pointer to the ring buffer structure
 * @param data The byte of data to write
 * @return true if the write was successful, false if the buffer is full
 */
bool ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
    if (rb->if_full) {
        // If the buffer is full, overwrite the oldest data
        rb->tail = (rb->tail + 1) % rb->capacity; // Move tail forward
    }   
    rb->buffer[rb->head] = data; // Write the new data at the head position
    rb->head = (rb->head + 1) % rb->capacity; // Move head forward
    if (rb->head == rb->tail) {
        rb->if_full = true; // If head meets tail, the buffer is full
    } 
    
    return true; 
}

/**
 * @brief Reads a byte of data from the ring buffer.
 * @param rb Pointer to the ring buffer structure
 * @param data Pointer to the variable where the read data will be stored
 * @return true if the read was successful, false if the buffer is empty
 */
bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data)
{
    if (rb->head == rb->tail && !rb->if_full) {
        // If head equals tail and the buffer is not full, it means the buffer is empty
        return false; 
    }
    *data = rb->buffer[rb->tail]; // Read the data from the tail position
    rb->tail = (rb->tail + 1) % rb->capacity; // Move tail forward
    rb->if_full = false; // After reading, the buffer cannot be full

    return true;
}

/**
 * @brief Returns the number of elements currently in the ring buffer.
 * @param rb Pointer to the ring buffer structure
 * @return The number of elements in the buffer
 */
uint16_t ring_buffer_count(ring_buffer_t *rb)
{
    if (rb->if_full) {
        return rb->capacity; // If the buffer is full, return its capacity
    }
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail; // If head is ahead of tail, return the difference
    } else {
        return (rb->capacity - rb->tail) + rb->head; // If tail is ahead of head, calculate the count
    }

}

/**
 * @brief Checks if the ring buffer is empty.
 * @param rb Pointer to the ring buffer structure
 * @return true if the buffer is empty, false otherwise
 */
bool ring_buffer_is_empty(ring_buffer_t *rb)
{
    return (rb->head == rb->tail && !rb->if_full); // Buffer is empty if head equals tail and not full      

}

/**
 * @brief Checks if the ring buffer is full.
 * @param rb Pointer to the ring buffer structure
 * @return true if the buffer is full, false otherwise
 */
bool ring_buffer_is_full(ring_buffer_t *rb)
{
    return rb->if_full; // Buffer is full if the full indicator is true
}

/**
 * @brief Flushes the ring buffer, clearing all data.
 * @param rb Pointer to the ring buffer structure
 */
void ring_buffer_flush(ring_buffer_t *rb)
{
    rb->head = 0;         // Reset head index to 0
    rb->tail = 0;         // Reset tail index to 0
    rb->if_full = false;  // Set full indicator to false
    // No need to clear the buffer memory, as it will be overwritten on next write
}