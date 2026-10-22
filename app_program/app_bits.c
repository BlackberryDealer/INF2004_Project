/**
 * @file    app_bits.c
 * @brief   Mask-and-shift packing helpers: clamp and offset encoding, and an
 *          LSB-first bit-stream writer and reader for byte payloads.
 * @owner   Buddy 1 (communications)
 * @kind    algo
 * @reqs    Minimum-bit requirement items 7-8 (compact binary messages and
 *          payloads; clamp, never wrap)
 */

#include "app_bits.h"          /* own header first (BARR-C 4.3.c) */

#include <stddef.h>            /* NULL */

/* ---- private constants and types ------------------------------------ */

#define APP_BITS_PER_BYTE    (8u)   /* stream bit n = bit n%8 of byte n/8 */
#define APP_BITS_WIDTH_MIN   (1u)
#define APP_BITS_WIDTH_MAX   (31u)  /* widest field in tools/layout.csv   */

/* ---- private function prototypes ------------------------------------- */

static bool app_bits_width_is_valid(uint32_t width);

/* ---- public functions ------------------------------------------------- */

/**
 * app_bits_encode
 * @brief   Clamps value to [min, max], adds offset and returns the stored
 *          (non-negative) code. No bitwise operator touches a signed value.
 * @param   value           value in stored LSB units
 * @param   min             lowest valid value (layout _MIN macro)
 * @param   max             highest valid value (layout _MAX macro)
 * @param   offset          offset added after the clamp (layout _OFFSET)
 * @param   pb_was_clamped  set true if a clamp happened, else false
 *                          (may be NULL)
 * @return  clamp(value, min, max) + offset
 * @context any
 */
uint32_t app_bits_encode(int32_t value, int32_t min, int32_t max,
                         int32_t offset, bool * pb_was_clamped)
{
    int32_t clamped = value;
    bool    b_was   = false;
    int32_t sum     = 0;

    if (clamped < min)
    {
        clamped = min;
        b_was   = true;
    }
    else if (clamped > max)
    {
        clamped = max;
        b_was   = true;
    }
    else
    {
        /* in range: stored unchanged */
    }

    if (NULL != pb_was_clamped)
    {
        *pb_was_clamped = b_was;
    }

    sum = clamped + offset;

    return (uint32_t) sum;     /* cast: sum >= 0 by layout check */
}

/**
 * app_bits_decode
 * @brief   Inverse of app_bits_encode: raw - offset.
 * @param   raw     stored code (at most 2^18, from a layout field)
 * @param   offset  offset used by the encoder (layout _OFFSET macro)
 * @return  value in stored LSB units
 * @context any
 */
int32_t app_bits_decode(uint32_t raw, int32_t offset)
{
    return ((int32_t) raw) - offset;   /* cast: raw <= 2^18, fits int32 */
}

/**
 * app_bits_clamp_u
 * @brief   Unsigned clamp to [0, max].
 * @param   value           value to clamp
 * @param   max             highest valid value
 * @param   pb_was_clamped  set true if a clamp happened, else false
 *                          (may be NULL)
 * @return  min(value, max)
 * @context any
 */
uint32_t app_bits_clamp_u(uint32_t value, uint32_t max,
                          bool * pb_was_clamped)
{
    uint32_t clamped = value;
    bool     b_was   = false;

    if (clamped > max)
    {
        clamped = max;
        b_was   = true;
    }

    if (NULL != pb_was_clamped)
    {
        *pb_was_clamped = b_was;
    }

    return clamped;
}

/**
 * app_bits_writer_init
 * @brief   Starts an LSB-first bit-stream writer and zero-fills the buffer.
 * @param   p_writer    writer state
 * @param   p_buf       output buffer
 * @param   size_bytes  buffer size in bytes
 * @return  none
 * @context any
 */
void app_bits_writer_init(app_bits_writer_t * p_writer, uint8_t * p_buf,
                          uint32_t size_bytes)
{
    uint32_t idx = 0u;

    if (NULL != p_writer)
    {
        p_writer->p_buf         = p_buf;
        p_writer->size_bytes    = (NULL != p_buf) ? size_bytes : 0u;
        p_writer->bit_pos       = 0u;
        p_writer->b_is_overflow = false;

        for (idx = 0u; idx < p_writer->size_bytes; idx++)
        {
            p_buf[idx] = 0u;
        }
    }
}

/**
 * app_bits_write
 * @brief   Appends the low width bits of value to the stream, bit 0 first.
 *          A write that does not fit (or an invalid width) sets
 *          b_is_overflow and writes nothing. Overflow is sticky: once set,
 *          later writes are ignored so no field lands at a wrong position.
 * @param   p_writer  writer state
 * @param   value     bits to write (bit 0 first)
 * @param   width     1..31
 * @return  none
 * @context any
 */
void app_bits_write(app_bits_writer_t * p_writer, uint32_t value,
                    uint32_t width)
{
    uint32_t idx      = 0u;
    uint32_t byte_idx = 0u;
    uint32_t bit_mask = 0u;

    if ((NULL != p_writer) && (!p_writer->b_is_overflow))
    {
        if ((!app_bits_width_is_valid(width)) ||
            ((p_writer->bit_pos + width) >
             (p_writer->size_bytes * APP_BITS_PER_BYTE)))
        {
            p_writer->b_is_overflow = true;
        }
        else
        {
            /* Bit-by-bit is fine: at most 144 bits per payload, at 5 Hz. */
            for (idx = 0u; idx < width; idx++)
            {
                if (0u != ((value >> idx) & 1u))
                {
                    byte_idx = p_writer->bit_pos / APP_BITS_PER_BYTE;
                    bit_mask = 1u << (p_writer->bit_pos % APP_BITS_PER_BYTE);
                    p_writer->p_buf[byte_idx] =
                        (uint8_t) (p_writer->p_buf[byte_idx] | bit_mask);
                        /* cast: bit_mask < 256, so the OR fits a byte */
                }
                p_writer->bit_pos++;
            }
        }
    }
}

/**
 * app_bits_writer_bytes
 * @brief   Number of bytes written so far, the last one rounded up.
 * @param   p_writer  writer state
 * @return  (bit_pos + 7) / 8, or 0 if p_writer is NULL
 * @context any
 */
uint32_t app_bits_writer_bytes(app_bits_writer_t const * p_writer)
{
    uint32_t bytes = 0u;

    if (NULL != p_writer)
    {
        bytes = (p_writer->bit_pos + (APP_BITS_PER_BYTE - 1u)) /
                APP_BITS_PER_BYTE;
    }

    return bytes;
}

/**
 * app_bits_reader_init
 * @brief   Starts an LSB-first bit-stream reader.
 * @param   p_reader    reader state
 * @param   p_buf       input buffer
 * @param   size_bytes  buffer size in bytes
 * @return  none
 * @context any
 */
void app_bits_reader_init(app_bits_reader_t * p_reader,
                          uint8_t const * p_buf, uint32_t size_bytes)
{
    if (NULL != p_reader)
    {
        p_reader->p_buf         = p_buf;
        p_reader->size_bytes    = (NULL != p_buf) ? size_bytes : 0u;
        p_reader->bit_pos       = 0u;
        p_reader->b_is_overflow = false;
    }
}

/**
 * app_bits_read
 * @brief   Reads the next width bits from the stream (bit 0 first).
 *          Reading past the end (or an invalid width) sets b_is_overflow
 *          and returns 0. Overflow is sticky, like the writer's.
 * @param   p_reader  reader state
 * @param   width     1..31
 * @return  the bits read, or 0 on overflow
 * @context any
 */
uint32_t app_bits_read(app_bits_reader_t * p_reader, uint32_t width)
{
    uint32_t idx      = 0u;
    uint32_t byte_idx = 0u;
    uint32_t bit      = 0u;
    uint32_t value    = 0u;

    if ((NULL != p_reader) && (!p_reader->b_is_overflow))
    {
        if ((!app_bits_width_is_valid(width)) ||
            ((p_reader->bit_pos + width) >
             (p_reader->size_bytes * APP_BITS_PER_BYTE)))
        {
            p_reader->b_is_overflow = true;
        }
        else
        {
            for (idx = 0u; idx < width; idx++)
            {
                byte_idx = p_reader->bit_pos / APP_BITS_PER_BYTE;
                bit      = ((uint32_t) p_reader->p_buf[byte_idx] >>
                            (p_reader->bit_pos % APP_BITS_PER_BYTE)) & 1u;
                            /* cast: widen the byte before shifting */
                value   |= bit << idx;
                p_reader->bit_pos++;
            }
        }
    }

    return value;
}

/* ---- private functions ------------------------------------------------ */

/**
 * app_bits_width_is_valid
 * @brief   True if width is a legal field width (1..31).
 * @param   width  requested field width in bits
 * @return  true if 1 <= width <= 31
 * @context any
 */
static bool app_bits_width_is_valid(uint32_t width)
{
    return ((width >= APP_BITS_WIDTH_MIN) && (width <= APP_BITS_WIDTH_MAX));
}

/*** end of file ***/
