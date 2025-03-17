#include "ringbuffer.h"

void ringbuffer_init(ringbuffer_t *rb, uint8_t *buffer) {
    rb->buffer = buffer;
    ringbuffer_reset(rb);
}

void ringbuffer_reset(ringbuffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
}

uint8_t ringbuffer_put(ringbuffer_t *rb, uint8_t item) {
    uint32_t next_head = (rb->head + 1) % RINGBUFFER_SIZE;

    if (next_head == rb->tail) {
        return -1;
    }

    rb->buffer[rb->head] = item;
    rb->head = next_head;

    return 0;
}

uint8_t ringbuffer_get(ringbuffer_t *rb) {
    if (ringbuffer_isEmpty(rb)) {
        return -1;
    }

    uint8_t item = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RINGBUFFER_SIZE;

    return item;
}

uint8_t ringbuffer_peek(ringbuffer_t *rb) {
    uint8_t item;

    if (ringbuffer_isEmpty(rb)) {
        item = rb->buffer[rb->head];
    }
    else if (rb->head == 0) {
        item = rb->buffer[RINGBUFFER_SIZE-1];
    }
    else {
        item = rb->buffer[rb->head-1];
    }
    
    return item;
}

inline bool ringbuffer_isEmpty(ringbuffer_t *rb) {
    return (rb->head == rb->tail);
}

inline bool ringbuffer_isFull(ringbuffer_t *rb) {
    return (((rb->head + 1) % RINGBUFFER_SIZE) == rb->tail);
}