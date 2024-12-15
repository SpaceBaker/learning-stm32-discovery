#include "ringbuffer.h"

void ringbuffer_init(ringbuffer_t *rb, uint8_t*buffer, size_t size) {
    rb->buffer = buffer;
    rb->max = size;
    ringbuffer_reset(rb);
}

void ringbuffer_reset(ringbuffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->full = false;
}

void ringbuffer_put(ringbuffer_t *rb, uint8_t item) {
    rb->buffer[rb->head] = item;

    if (rb->full) {
        rb->tail = (rb->tail + 1) % rb->max;
    }

    rb->head = (rb->head + 1) % rb->max;

    rb->full = (rb->head == rb->tail);
}

uint8_t ringbuffer_get(ringbuffer_t *rb) {
    if (ringbuffer_isEmpty(rb)) {
        return -1; // Handle empty buffer case as needed
    }

    uint8_t item = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->max;

    rb->full = false;

    return item;
}

uint8_t ringbuffer_peek(ringbuffer_t *rb)
{
    uint8_t item;

    if (ringbuffer_isEmpty(rb)) {
        item = rb->buffer[rb->head];
    }
    else if (rb->head == 0) {
        item = rb->buffer[rb->max-1];
    }
    else {
        item = rb->buffer[rb->head-1];
    }
    
    return item;
}

size_t ringbuffer_length(ringbuffer_t *rb) {
    size_t length = rb->max;

    if (!rb->full) {
        if (rb->head >= rb->tail) {
            length = rb->head - rb->tail;
        } else {
            length = rb->max + rb->head - rb->tail;
        }
    }

    return length;
}

bool ringbuffer_isEmpty(ringbuffer_t *rb) {
    return (!rb->full && (rb->head == rb->tail));
}

bool ringbuffer_isFull(ringbuffer_t *rb) {
    return rb->full;
}