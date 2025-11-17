//
// Created by felix on 16.11.25.
//

#ifndef T2V_MOD_FW_QUEUE_HANDLE_VEC_H
#define T2V_MOD_FW_QUEUE_HANDLE_VEC_H

#include "FreeRTOS.h"
#include "queue.h"

typedef struct QueueHandleVec
{
    size_t capacity;
    size_t len;
    QueueHandle_t *pointer;
} QueueHandleVec_t;

inline void queue_handle_vec_with_capacity(QueueHandleVec_t *qv, size_t capacity)
{
    qv->capacity = capacity;
    qv->len = 0;
    qv->pointer = malloc(sizeof(QueueHandle_t) * capacity);
}

inline void queue_handle_vec_from_components_unchecked(QueueHandleVec_t *qv, size_t capacity, size_t len, QueueHandle_t *pointer)
{
    qv->capacity = capacity;
    qv->len = len;
    qv->pointer = pointer;
}

inline bool queue_handle_vec_push(QueueHandleVec_t *qv, QueueHandle_t q)
{
    if (qv->len < qv->capacity)
    {
        qv->pointer[qv->len] = q;
        qv->len++;
        return true;
    }
    return false;
}

inline void queue_handle_vec_drop(const QueueHandleVec_t *qv)
{
    free(qv->pointer);
}

#endif //T2V_MOD_FW_QUEUE_HANDLE_VEC_H