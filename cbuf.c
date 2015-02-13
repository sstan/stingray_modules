/*
 * cbuf.c
 *
 *  Created on: Feb 10, 2015
 *      Author: sstan
 */

#include <stdint.h>

#include "cbuf.h"

typedef struct cbuf_entry_t cbuf_entry_t;

struct cbuf_entry_t {
	uint16_t tcnt;
	uint32_t ovf_cntr;
	cbuf_entry_t* next;
	cbuf_entry_t* prev;
};

struct cbuf_t {
	cbuf_entry_t buf_elem[CBUF_ENTRIES];
	cbuf_entry_t* head_p;
	uint16_t cycles_per_ovf;
	uint32_t count;
};

static struct cbuf_t buf_g[CBUF_NUMBER_OF_BUFFERS];

void cbuf_reset(uint8_t cbuf_id, uint16_t cycles_per_ovf)
{
	int i;

	buf_g[cbuf_id].head_p = &buf_g[cbuf_id].buf_elem[0];
	buf_g[cbuf_id].cycles_per_ovf = cycles_per_ovf;
	buf_g[cbuf_id].count = 0;

	buf_g[cbuf_id].buf_elem[0].next =&buf_g[cbuf_id].buf_elem[1];
	buf_g[cbuf_id].buf_elem[0].prev =&buf_g[cbuf_id].buf_elem[CBUF_ENTRIES-1];

	buf_g[cbuf_id].buf_elem[CBUF_ENTRIES-1].next =&buf_g[cbuf_id].buf_elem[0];
	buf_g[cbuf_id].buf_elem[CBUF_ENTRIES-1].prev =&buf_g[cbuf_id].buf_elem[CBUF_ENTRIES-2];

	for (i = 1; i < CBUF_ENTRIES - 1; i++)
	{
		buf_g[cbuf_id].buf_elem[i].next = &buf_g[cbuf_id].buf_elem[i+1];
		buf_g[cbuf_id].buf_elem[i].prev = &buf_g[cbuf_id].buf_elem[i-1];
	}

	for (i = 0; i < CBUF_ENTRIES; i++)
	{
		buf_g[cbuf_id].buf_elem[i].ovf_cntr = 0;
		buf_g[cbuf_id].buf_elem[i].tcnt     = 0;
	}
}

void cbuf_write(uint8_t cbuf_id, uint16_t tcnt, uint32_t ovf_cntr)
{
	buf_g[cbuf_id].head_p->ovf_cntr = ovf_cntr;
	buf_g[cbuf_id].head_p->tcnt     = tcnt;
	buf_g[cbuf_id].head_p           = buf_g[cbuf_id].head_p->next;
	buf_g[cbuf_id].count++;
}

void cbuf_read_n_timestamps(uint8_t cbuf_id, int n, uint32_t timestamp[])
{
	cbuf_entry_t* ent_p;

	uint16_t cpo;

	int i;

	cpo = buf_g[cbuf_id].cycles_per_ovf;

	ent_p = buf_g[cbuf_id].head_p->prev;

	for (i = 0; i < n; i++)
	{
		timestamp[i] = ent_p->tcnt + cpo*ent_p->ovf_cntr;
		ent_p = ent_p->prev;
	}

}

uint32_t cbuf_count(uint8_t cbuf_id)
{
	return buf_g[cbuf_id].count;
}

uint16_t cbuf_get_cycles_per_ovf(uint8_t cbuf_id)
{
	return buf_g[cbuf_id].cycles_per_ovf;
}
