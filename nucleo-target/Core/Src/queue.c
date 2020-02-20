#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <queue.h>

/**
	*	@brief	������������� �������
	*	@note		
	*	@param	q ������ �� ��� �������
	*	@retval	
**/
//
void queue_init(struct Queue* q)
{
//	q->front = (struct Node*) malloc(sizeof(struct Node));
//	q->last = (struct Node*) malloc(sizeof(struct Node));
	q->front = NULL;
	q->last = NULL;
	q->max_len = MAX_QLEN;
//	free(q->front);
//	free(q->last);
	q->size = 0;
}
//

/**
	*	@brief	������� �������
	*	@note		
	*	@param	q ������ �� ��� �������
	*	@retval	
**/
//
void queue_clear(struct Queue* q)
{
	while(q->size--)
		queue_pop(q);
}
//

/**
	*	@brief	������� ��������� ���������� �������
	*	@note		
	*	@param	q ������ �� ��� �������
	*	@retval	
**/
//
void queue_pop(struct Queue* q)
{
	if(q->size>0)
	{
		q->size--;
		struct Node* tmp = q->front;
		q->front = q->front->next;
		free(tmp);
	}
}
//

/**
	*	@brief	�������� �� �������
	*	@note		
	*	@param	q ������ �� ��� �������
	*	@retval	
**/
//
bool queue_isempty(struct Queue* q)
{
	return q->size == 0;
}
//

/**
	*	@brief	�������� ������� � �������
	*	@note		
	*	@param	q ������ �� ��� �������
	*	@param	string ������
	*	@param	len ����� ������
	*	@retval	���
**/
//
void queue_push(struct Queue* q, unsigned char* string, unsigned short len)
{
	if(q->size + 1 > q->max_len)
	{
		queue_pop(q);
	}
		q->size++;
		if(q->front == NULL)
		{
			q->front = (struct Node*) malloc(sizeof(struct Node));
			memset(q->front->string, 0x00, MAX_STRING_LENGTH);
			memcpy(q->front->string, string, len);
			q->front->len = len;
			q->front->next = NULL;
			q->last = q->front;
		}
		else
		{
			q->last->next = (struct Node*) malloc(sizeof(struct Node));
			memset(q->last->next->string, 0x00, MAX_STRING_LENGTH);
			memcpy(q->last->next->string, string, len);
			q->last->next->len = len;
			q->last->next->next = NULL;
			q->last = q->last->next;
		}
}
//

/**
	*	@brief	�������� ������ ���������� ������ ��� �� �����
	*	@note		
	*	@param	q ������ �� ��� �������
	*	@param	string ������ �� ������, � ������� ������������ ������
	*	@param	from ��������� ������� � �������� ���������� ������
	*	@param	to ���������� ���������� ���������
	*	@retval	���
**/
//
void queue_get_front(struct Queue* q, unsigned char* string, unsigned short from, unsigned short num)
{
	memcpy(string, q->front->string+from, num);
}
//

/**
	*	@brief	�������� ����� ������ ���������� ������
	*	@note		
	*	@param	q ������ �� ��� �������
	*	@retval	����� ������ ���������� ������
**/
//
unsigned short queue_get_frontl(struct Queue* q)
{
	return q->front->len;
}
//

/**
	*	@brief	�������� ���� �� ������ �� �������
	*	@note		
	*	@param	q ������ �� ��� �������
	*	@param	index ������ �������� ������, ������� ����� ��������
	*	@retval	����� ������ ���������� ������
**/
//
unsigned char queue_get_bytefrom(struct Queue* q, unsigned short index)
{
	return q->front->string[index];
}
//

unsigned char* queue_get_first_ptr(struct Queue* q)
{
	return &q->front->string[0];
}
//
