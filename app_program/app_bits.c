/**
 * @file    app_bits.c
 * @brief   Phase 0 STUB of the packing helpers: every function returns 0 or
 *          does nothing. Buddy 1 replaces this file in B1 Phase 0.
 * @owner   Buddy 1 (communications)
 * @kind    algo
 * @reqs    Minimum-bit requirement items 7-8 (implemented by Buddy 1)
 */

#include "app_bits.h"          /* own header first (BARR-C 4.3.c) */

/* ---- public functions ------------------------------------------------- */

/**
 * app_bits_encode
 * @brief   STUB. Clamp, add offset, return as unsigned.
 * @param   value           value to encode
 * @param   min             lowest valid value
 * @param   max             highest valid value
 * @param   offset          offset added after the clamp
 * @param   pb_was_clamped  set true if a clamp happened (may be NULL)
 * @return  0 (stub)
 * @context any
 */
uint32_t app_bits_encode(int32_t value, int32_t min, int32_t max,
                         int32_t offset, bool * pb_was_clamped)
{
    (void) value;           /* stub: unused until Buddy 1's version */
    (void) min;
    (void) max;
    (void) offset;
    (void) pb_was_clamped;

    return 0u;
}

/**
 * app_bits_decode
 * @brief   STUB. Inverse of encode: raw - offset.
 * @param   raw     stored value
 * @param   offset  offset used by the encoder
 * @return  0 (stub)
 * @context any
 */
int32_t app_bits_decode(uint32_t raw, int32_t offset)
{
    (void) raw;             /* stub: unused until Buddy 1's version */
    (void) offset;

    return 0;
}

/**
 * app_bits_clamp_u
 * @brief   STUB. Unsigned clamp to [0, max].
 * @param   value           value to clamp
 * @param   max             highest valid value
 * @param   pb_was_clamped  set true if a clamp happened (may be NULL)
 * @return  0 (stub)
 * @context any
 */
uint32_t app_bits_clamp_u(uint32_t value, uint32_t max,
                          bool * pb_was_clamped)
{
    (void) value;           /* stub: unused until Buddy 1's version */
    (void) max;
    (void) pb_was_clamped;

    return 0u;
}

/**
 * app_bits_writer_init
 * @brief   STUB. Starts an LSB-first bit-stream writer.
 * @param   p_writer    writer state
 * @param   p_buf       output buffer
 * @param   size_bytes  buffer size in bytes
 * @return  none
 * @context any
 */
void app_bits_writer_init(app_bits_writer_t * p_writer, uint8_t * p_buf,
                          uint32_t size_bytes)
{
    (void) p_writer;        /* stub: unused until Buddy 1's version */
    (void) p_buf;
    (void) size_bytes;
}

/**
 * app_bits_write
 * @brief   STUB. Appends width bits of value to the stream.
 * @param   p_writer  writer state
 * @param   value     bits to write (bit 0 first)
 * @param   width     1..31
 * @return  none
 * @context any
 */
void app_bits_write(app_bits_writer_t * p_writer, uint32_t value,
                    uint32_t width)
{
    (void) p_writer;        /* stub: unused until Buddy 1's version */
    (void) value;
    (void) width;
}

/**
 * app_bits_writer_bytes
 * @brief   STUB. Number of bytes written so far (rounded up).
 * @param   p_writer  writer state
 * @return  0 (stub)
 * @context any
 */
uint32_t app_bits_writer_bytes(app_bits_writer_t const * p_writer)
{
    (void) p_writer;        /* stub: unused until Buddy 1's version */

    return 0u;
}

/**
 * app_bits_reader_init
 * @brief   STUB. Starts an LSB-first bit-stream reader.
 * @param   p_reader    reader state
 * @param   p_buf       input buffer
 * @param   size_bytes  buffer size in bytes
 * @return  none
 * @context any
 */
void app_bits_reader_init(app_bits_reader_t * p_reader,
                          uint8_t const * p_buf, uint32_t size_bytes)
{
    (void) p_reader;        /* stub: unused until Buddy 1's version */
    (void) p_buf;
    (void) size_bytes;
}

/**
 * app_bits_read
 * @brief   STUB. Reads the next width bits from the stream.
 * @param   p_reader  reader state
 * @param   width     1..31
 * @return  0 (stub)
 * @context any
 */
uint32_t app_bits_read(app_bits_reader_t * p_reader, uint32_t width)
{
    (void) p_reader;        /* stub: unused until Buddy 1's version */
    (void) width;

    return 0u;
}

/*** end of file ***/
