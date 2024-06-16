/*
 * thread_safe_queue.h
 *
 *  Created on: Jun 9, 2024
 *      Author: muzix
 */

#ifndef INC_THREAD_SAFE_QUEUE_H_
#define INC_THREAD_SAFE_QUEUE_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "lwip.h"
#include "lwip/udp.h"

#define QUEUE_LENGTH 20

//void initQueue(QueueHandle_t* queue) {
//    *queue = xQueueCreate(QUEUE_LENGTH, sizeof(struct pbuf *));
//}
//
//// push function
//BaseType_t enqueuePbuf(struct pbuf *pbuf, TickType_t ticksToWait, QueueHandle_t* queue) {
//    return xQueueSend(*queue, &pbuf, ticksToWait);
//}
//
//// pop function
//BaseType_t dequeuePbuf(struct pbuf *pbuf, TickType_t ticksToWait, QueueHandle_t* queue) {
//    return xQueueReceive(*queue, pbuf, ticksToWait);
//}
//
//// Network  buffer queue
//typedef struct MessageQueue
//{
//	void (*init)(QueueHandle_t* queue);
//	BaseType_t (*enqueue)(struct pbuf *pbuf, TickType_t ticksToWait, QueueHandle_t* queue);
//	BaseType_t (*dequeue)(struct pbuf *pbuf, TickType_t ticksToWait, QueueHandle_t* queue);
//	QueueHandle_t pbufQueue;
//	//sys_mutex_t mutex;
//} MessageQueue;

typedef struct Node {
	struct pbuf *data;
	struct Node *next;
} Node;

typedef struct Queue {
	Node* front;
	Node* rear;
} Queue;

void initQueue(Queue *q) {
	q->front = q->rear = NULL;
}

void enqueue(Queue* q, struct pbuf *value)
{
	Node *newNode = (Node *)malloc(sizeof(Node));
	if (newNode == NULL) {
		printf("Memory allocation failed.\n");
		return;
	}
	newNode->data = value;
	newNode->next = NULL;

	if (q->rear == NULL) {
		q->front = q->rear = newNode;
	} else {
		q->rear->next = newNode;
		q->rear = newNode;
	}
}

int dequeue(Queue *q, struct pbuf **value) {
    if (q->front == NULL) {
        return 0;  // 队列为空
    }

    Node *temp = q->front;
    *value = temp->data;
    q->front = q->front->next;
    if (q->front == NULL) {
        q->rear = NULL;
    }

    free(temp);
    return 1;  // 成功出队
}

typedef struct MessageQueue
{
	void (*init)(Queue* queue);
	void (*enqueue)(Queue* q, struct pbuf *value);
	int (*dequeue)(Queue *q, struct pbuf **value);
	Queue pbufQueue;
	struct udp_pcb *udp_instace;
	ip_addr_t addr;
	u16_t port;
	//sys_mutex_t mutex; //Technical debt, thread safe, not important for now
} MessageQueue;
#endif /* INC_THREAD_SAFE_QUEUE_H_ */
