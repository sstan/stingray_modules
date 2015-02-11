/*
 * cbuf.h
 *
 *  Created on: Feb 10, 2015
 *      Author: Nco
 */

#ifndef STINGRAY_MODULES_CBUF_H_
#define STINGRAY_MODULES_CBUF_H_

#define CBUF_ENTRIES           20
#define CBUF_NUMBER_OF_BUFFERS 2

void cbuf_reset(uint8_t cbuf_id, uint16_t cycles_per_ovf);

void cbuf_write(uint8_t cbuf_id, uint16_t tcnt, uint32_t ovf_cntr);

int cbuf_read_timestamp(uint8_t cbuf_id, uint32_t* timestamp);

uint8_t cbuf_size(uint8_t cbuf_id);

uint16_t cbuf_count(uint8_t cbuf_id);

#endif /* STINGRAY_MODULES_CBUF_H_ */
