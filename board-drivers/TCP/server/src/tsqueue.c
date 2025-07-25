/*
 * tsqueue.c
 *
 *  Created on: Jan 21, 2024
 *      Author: evanm
 */

#include "../inc/tsqueue.h"

/*
 *
 * Thread Safe Queue
 *
 */

int list_push(List *list, void* data, TickType_t wait) {
	return list_add(list, data, wait); // push to front
}

int list_pop(List *list, void* data, TickType_t wait) {
	return list_remove(list, data, wait); // pop from end
}

void free_node_data(CallbackFree free_callback, void *data) {
	if (free_callback != NULL)
		free_callback(data);
	else
		free(data);
}

List *list_create(int dataSize, CallbackFree free_callback) {
	assert(dataSize > 0);

	List *list = (List *)malloc(sizeof(List));
	list->count = 0;
	list->data_size = dataSize;
	list->head = calloc(sizeof(Node), 1); // next init to NULL
	list->callback_free = free_callback;
	list->mutex = xSemaphoreCreateMutex();
	list->msgs = xSemaphoreCreateCounting(MAX_QUEUE_SIZE, 0);
	list->remainingSpace = xSemaphoreCreateCounting(MAX_QUEUE_SIZE, MAX_QUEUE_SIZE);

	return list;
}

void list_destroy(List *list){

	assert(list != NULL);

	xSemaphoreTake(list->mutex, portMAX_DELAY);

	Node *node = list->head;
	while (node->next != NULL){
		list->head = node->next;
		free_node_data(list->callback_free, node->data);
		free(node);
		list->count--;
		node = list->head;
	}

	xSemaphoreGive(list->mutex);

	free(list);
}


int list_add(List *list, void *data, TickType_t wait) { // push to front

	assert(list != NULL);

	if(xSemaphoreTake(list->remainingSpace, wait) == pdFAIL) { // Only add to queue if there is space
		return 1;
	}

	xSemaphoreTake(list->mutex, portMAX_DELAY); // Lock mutex

	assert(data != NULL);

	Node *newNode = malloc(sizeof(Node));
	newNode->data = malloc(list->data_size);
	memcpy(newNode->data, data, list->data_size);

	newNode->next = list->head;
	list->head = newNode;
	list->count++;

	xSemaphoreGive(list->mutex); // Unlock mutex

	xSemaphoreGive(list->msgs); // Notify waiting consumers
	return 0;
}

// Return 0 if normal, 1 if an early return was triggered
int list_remove(List *list, void* data, TickType_t wait) {

	assert(list != NULL);

	if(xSemaphoreTake(list->msgs, wait) == pdFAIL)  { // Remove message when at least one exists
		return 1;
	}

	xSemaphoreTake(list->mutex, portMAX_DELAY); // Lock mutex

	assert(list->count > 0);

	Node *prev_node = NULL;
	Node *curr_node = list->head;

	// Condition OK because count > 0
	while (curr_node->next->next != NULL){ // look for last node in list
		prev_node = curr_node;
		curr_node = prev_node->next;
	}

	memcpy(data, curr_node->data, list->data_size);

	if (prev_node != NULL) {
		prev_node->next = curr_node->next;
	} else {
		list->head = curr_node->next;
	}
	free_node_data(list->callback_free, curr_node->data);
	free(curr_node);

	list->count--;

	xSemaphoreGive(list->mutex); // Unlock mutex

	xSemaphoreGive(list->remainingSpace); // Notify waiting producers

	return 0;
}
