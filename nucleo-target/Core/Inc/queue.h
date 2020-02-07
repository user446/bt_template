#ifndef __QUEUE_H
#define __QUEUE_H

#include <stdbool.h>

#define MAX_STRING_LENGTH 128
#define MAX_QLEN 10
/**
	*	@brief	��� �������� ������� �����
	*	@note		
**/
//
typedef struct Node
{
	struct Node* next; 				/// ������ �� ��������� �������
	unsigned short len;				/// ����� ������ � ��������
	unsigned char string[MAX_STRING_LENGTH];		/// ������ � ��������
}Node;

/**
	*	@brief	��� ������� �����
	*	@note		
**/
//
typedef struct Queue
{
	struct Node* front; 	/// ������ �� ��������� �������
	struct Node* last;		/// ������ �� ������ ���������� �������
	unsigned int size;		/// ������ �������
	unsigned short max_len;		/// ������������ ����� �������
}Queue;

void queue_init(struct Queue* q);
void queue_clear(struct Queue* q);
void queue_pop(struct Queue* q);
bool queue_isempty(struct Queue* q);
void queue_push(struct Queue* q, unsigned char* string, unsigned short len);

void queue_get_front(struct Queue* q, unsigned char* string, unsigned short from, unsigned short to);
unsigned short queue_get_frontl(struct Queue* q);
unsigned char queue_get_bytefrom(struct Queue* q, unsigned short index);
unsigned char* queue_get_first_ptr(struct Queue* q);

#endif
