/*
 * cbuf.c
 *
 *  Created on: Feb 10, 2015
 *      Author: sstan
 */

#include <stdint.h>

#include "cbuf.h"

struct cbuf_t {
	uint16_t tcnt[CBUF_ENTRIES];
	uint32_t ovf_cntr[CBUF_ENTRIES];
	uint8_t  head_ix;
	uint8_t  tail_ix;
	uint8_t  size;
	uint16_t cycles_per_ovf;
	uint16_t count;
};

static struct cbuf_t cbuf_buffer_g[CBUF_NUMBER_OF_BUFFERS];

void cbuf_reset(uint8_t cbuf_id, uint16_t cycles_per_ovf)
{
	int i;

	cbuf_buffer_g[cbuf_id].head_ix        = 0;
	cbuf_buffer_g[cbuf_id].tail_ix        = 0;
	cbuf_buffer_g[cbuf_id].size           = 0;
	cbuf_buffer_g[cbuf_id].cycles_per_ovf = cycles_per_ovf;
	cbuf_buffer_g[cbuf_id].count          = 0;

	for (i = 0; i < CBUF_ENTRIES; i++)
	{
		cbuf_buffer_g[cbuf_id].ovf_cntr[i] = 0;
		cbuf_buffer_g[cbuf_id].tcnt    [i] = 0;
	}
}

void cbuf_write(uint8_t cbuf_id, uint16_t tcnt, uint32_t ovf_cntr)
{
	uint8_t next_ix = cbuf_buffer_g[cbuf_id].head_ix + 1;

	if (next_ix == CBUF_ENTRIES)
	{
		next_ix = 0;
	}

	cbuf_buffer_g[cbuf_id].tcnt    [next_ix] = tcnt;
	cbuf_buffer_g[cbuf_id].ovf_cntr[next_ix] = ovf_cntr;

	cbuf_buffer_g[cbuf_id].head_ix = next_ix;
	cbuf_buffer_g[cbuf_id].count++;
	cbuf_buffer_g[cbuf_id].size++;
}

int cbuf_read_timestamp(uint8_t cbuf_id, uint32_t* timestamp)
{
	uint16_t tcnt;
	uint32_t ovf_cnt;
	uint16_t cpo;
	uint8_t tail;

	if (cbuf_buffer_g[cbuf_id].size == 0)
	{
		return 0;
	}

	tail    = cbuf_buffer_g[cbuf_id].tail_ix;

	tcnt    = cbuf_buffer_g[cbuf_id].tcnt          [tail];
	ovf_cnt = cbuf_buffer_g[cbuf_id].ovf_cntr      [tail];
	cpo     = cbuf_buffer_g[cbuf_id].cycles_per_ovf;

	*timestamp = (uint32_t)tcnt + cpo*ovf_cnt;


	tail += 1;

	if (tail == CBUF_ENTRIES)
	{
		tail = 0;
	}

	cbuf_buffer_g[cbuf_id].tail_ix = tail;
	cbuf_buffer_g[cbuf_id].size--;

	return 1;
}

uint8_t cbuf_size(uint8_t cbuf_id)
{
	return cbuf_buffer_g[cbuf_id].size;
}

uint16_t cbuf_count(uint8_t cbuf_id)
{
	return cbuf_buffer_g[cbuf_id].count;
}
