#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define RING_BUFFER_SIZE 256

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
} ring_buffer_t;

void ring_buffer_init(ring_buffer_t *rb);
bool ring_buffer_put(ring_buffer_t *rb, uint8_t data);
bool ring_buffer_get(ring_buffer_t *rb, uint8_t *data);
uint32_t ring_buffer_available(ring_buffer_t *rb);
bool ring_buffer_is_empty(ring_buffer_t *rb);
bool ring_buffer_is_full(ring_buffer_t *rb);

#endif /* RING_BUFFER_H */
