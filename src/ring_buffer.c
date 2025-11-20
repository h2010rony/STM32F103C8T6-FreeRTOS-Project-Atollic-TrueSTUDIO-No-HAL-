#include "ring_buffer.h"

void ring_buffer_init(ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

bool ring_buffer_put(ring_buffer_t *rb, uint8_t data)
{
    if (rb->count >= RING_BUFFER_SIZE) {
        return false; // Buffer full
    }

    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
    rb->count++;
    return true;
}

bool ring_buffer_get(ring_buffer_t *rb, uint8_t *data)
{
    if (rb->count == 0) {
        return false; // Buffer empty
    }

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    rb->count--;
    return true;
}

uint32_t ring_buffer_available(ring_buffer_t *rb)
{
    return rb->count;
}

bool ring_buffer_is_empty(ring_buffer_t *rb)
{
    return (rb->count == 0);
}

bool ring_buffer_is_full(ring_buffer_t *rb)
{
    return (rb->count >= RING_BUFFER_SIZE);
}

