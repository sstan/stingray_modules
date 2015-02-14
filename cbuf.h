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

void cbuf_reset(uint8_t cbuf_id);

void cbuf_write(uint8_t cbuf_id, uint32_t delta);

int cbuf_read(uint8_t cbuf_id, uint32_t tbl[]);

int cbuf_get_size(uint8_t cbuf_id);

#endif /* STINGRAY_MODULES_CBUF_H_ */
