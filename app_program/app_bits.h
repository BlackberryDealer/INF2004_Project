/**
 * @file    app_bits.h
 * @brief   Mask-and-shift packing helpers. Pure C, no kernel includes.
 * @owner   Buddy 1
 * @kind    algo
 */

#ifndef APP_BITS_H
#define APP_BITS_H

#include <stdint.h>
#include <stdbool.h>

/* Field access on one 16- or 32-bit word. width must be 1..31 and
   (pos + width) <= 32; callers pass generated layout macros only.   */
static inline uint32_t app_bits_get(uint32_t word, uint32_t pos,
                                    uint32_t width)
{
    return ((word >> pos) & ((1u << width) - 1u));
}

static inline uint32_t app_bits_put(uint32_t word, uint32_t pos,
                                    uint32_t width, uint32_t value)
{
    uint32_t const mask = ((1u << width) - 1u) << pos;

    return ((word & ~mask) | ((value << pos) & mask));
}

/* Clamp value to [min, max] (signed), add offset, return as unsigned.
   *pb_was_clamped (may be NULL) is set true only if a clamp happened. */
extern uint32_t app_bits_encode(int32_t value, int32_t min, int32_t max,
                                int32_t offset, bool * pb_was_clamped);

/* Inverse of encode: raw - offset. */
extern int32_t  app_bits_decode(uint32_t raw, int32_t offset);

/* Unsigned clamp to [0, max]. */
extern uint32_t app_bits_clamp_u(uint32_t value, uint32_t max,
                                 bool * pb_was_clamped);

/* LSB-first bit-stream writer for byte payloads (MQTT, K7 details).
   init zero-fills the buffer. A write past the end sets b_is_overflow
   and writes nothing.                                                 */
typedef struct
{
    uint8_t * p_buf;
    uint32_t  size_bytes;
    uint32_t  bit_pos;
    bool      b_is_overflow;
} app_bits_writer_t;

extern void     app_bits_writer_init(app_bits_writer_t * p_writer,
                                     uint8_t * p_buf, uint32_t size_bytes);
extern void     app_bits_write(app_bits_writer_t * p_writer, uint32_t value,
                               uint32_t width);          /* width 1..31 */
extern uint32_t app_bits_writer_bytes(app_bits_writer_t const * p_writer);

typedef struct
{
    uint8_t const * p_buf;
    uint32_t        size_bytes;
    uint32_t        bit_pos;
    bool            b_is_overflow;
} app_bits_reader_t;

extern void     app_bits_reader_init(app_bits_reader_t * p_reader,
                                     uint8_t const * p_buf,
                                     uint32_t size_bytes);
extern uint32_t app_bits_read(app_bits_reader_t * p_reader, uint32_t width);

#endif /* APP_BITS_H */

/*** end of file ***/
