/*
 * tsqueue.h
 *
 *  Created on: Jan 21, 2024
 *      Author: evanm
 */

#pragma once

#ifndef INC_TSQUEUE_H_
#define INC_TSQUEUE_H_

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include <freertos.h>
#include <semphr.h>

/*
 *
 * Thread Safe Queue
 *
 */

typedef void(*CallbackFree)(void *);

typedef struct _node {
	void *data;
	struct _node *next;
} Node;

typedef struct {
	int count;
	int data_size; // max data size for node
	Node *head;
	Node *tail;
	CallbackFree callback_free;
	xSemaphoreHandle mutex;
	xSemaphoreHandle msgs;
	xSemaphoreHandle remainingSpace;
} List;

#define MAX_QUEUE_SIZE 100 // CURRENTLY NOT IMPLEMENTED IN CODE

// Wrapper functions

// Allocates new node at end of list and copies data
void list_push(List *list, void* data);

// Copies data into buf from node at front of list and frees the node with the CallbackFree
void list_pop(List *list, void* data);


List* list_create(int dataSize, CallbackFree free_callback);
void list_destroy(List *list);
void list_add(List *list, void *data);
void list_remove(List *list, void* data);


#endif /* INC_TSQUEUE_H_ */
