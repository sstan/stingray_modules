/*
 * cbuf.c
 *
 *  Created on: Feb 10, 2015
 *      Author: sstan
 */

#include <stdint.h>

#include "cbuf.h"

typedef struct cbuf_entry_t cbuf_entry_t;

/* Data Structure Node. */
struct cbuf_entry_t {
	uint32_t      delta;
	cbuf_entry_t* next;
	cbuf_entry_t* prev;
};

/* Buffer Structure */
struct cbuf_t {
	cbuf_entry_t  buf_elem[CBUF_ENTRIES];
	/* Always points to the node that will store the next data. */
	cbuf_entry_t* head_p;
	int           size;
};

static struct cbuf_t buf_g[CBUF_NUMBER_OF_BUFFERS];

void cbuf_reset(uint8_t cbuf_id)
{
	int i;

	/* pointer to the desired buffer */

	struct cbuf_t* b_p;
	b_p = &buf_g[cbuf_id];

	b_p->size   = 0;
	b_p->head_p = &b_p->buf_elem[0];

	/* The nodes are doubly and circularly linked */

	/* Link the first element */

	b_p->buf_elem[0].next = &b_p->buf_elem[1];
	b_p->buf_elem[0].prev = &b_p->buf_elem[CBUF_ENTRIES-1];

	/* Link the last element */

	b_p->buf_elem[CBUF_ENTRIES-1].next = &b_p->buf_elem[0];
	b_p->buf_elem[CBUF_ENTRIES-1].prev = &b_p->buf_elem[CBUF_ENTRIES-2];

	/* Link all the remaining elements */

	for (i = 1; i < CBUF_ENTRIES - 1; i++)
	{
		b_p->buf_elem[i].next = &b_p->buf_elem[i+1];
		b_p->buf_elem[i].prev = &b_p->buf_elem[i-1];
	}

	/* Initialize the data */

	for (i = 0; i < CBUF_ENTRIES; i++)
	{
		b_p->buf_elem[i].delta = 0;
	}
}

void cbuf_write(uint8_t cbuf_id, uint32_t delta)
{
	struct cbuf_t* b_p = &buf_g[cbuf_id]; /* pointer to the desired buffer */

	b_p->head_p->delta = delta;

	/* set the head node pointer to the next node that will receive data */

	b_p->head_p = b_p->head_p->next;

	if (b_p->size < CBUF_ENTRIES)
	{
		b_p->size++;
	}
}

int cbuf_read(uint8_t cbuf_id, uint32_t tbl[])
{
	cbuf_entry_t* ent_p;
	int size;
	int i;

	size  = buf_g[cbuf_id].size;

	ent_p = buf_g[cbuf_id].head_p->prev; /* The most recently written data */

	/* Copy the data from the most recently written node
	 * to least recently written one.
	 */
	for (i = 0; i < size; i++)
	{
		tbl[i] = ent_p->delta;
		ent_p  = ent_p->prev;
	}

	buf_g[cbuf_id].size -= size;

	return size;
}

int cbuf_get_size(uint8_t cbuf_id)
{
	return buf_g[cbuf_id].size;
}
