/**
 * @file    code39.c
 * @brief   Pure Code 39 width-to-character decoder in both directions.
 * @owner   Buddy 3 (line, barcode)
 * @kind    algo
 * @reqs    FR2, NFR5
 */

#include "code39.h"
#include <stddef.h>
#include "cfg_barcode.h"

/* ---- private constants and types ------------------------------------ */

#define CODE39_TABLE_SIZE      (44u)
#define CODE39_CHAR_ELEMENTS   (9u)
#define CODE39_CHAR0_START     (0u)
#define CODE39_GAP1_IDX        (9u)
#define CODE39_CHAR1_START     (10u)
#define CODE39_GAP2_IDX        (19u)
#define CODE39_CHAR2_START     (20u)
#define CODE39_NARROW_MAX_IDX  (19u)
#define CODE39_WIDE_MIN_IDX    (20u)
#define CODE39_RATIO_SCALE     (10u)
#define CODE39_NOT_FOUND       (0xFFu)
#define CODE39_ASCII_UNKNOWN   ((uint32_t) '?')

/* ---- private data ---------------------------------------------------- */

/* Code 39 9-bit element patterns (bit 8 = first element, 1 = wide). */
static uint16_t const g_code39_patterns[CODE39_TABLE_SIZE] =
{
    0x034u, 0x121u, 0x061u, 0x160u, 0x031u,
    0x130u, 0x070u, 0x025u, 0x124u, 0x064u,
    0x109u, 0x049u, 0x148u, 0x019u, 0x118u,
    0x058u, 0x00Du, 0x10Cu, 0x04Cu, 0x01Cu,
    0x103u, 0x043u, 0x142u, 0x013u, 0x112u,
    0x052u, 0x007u, 0x106u, 0x046u, 0x016u,
    0x181u, 0x0C1u, 0x1C0u, 0x091u, 0x190u,
    0x0D0u, 0x085u, 0x184u, 0x0C4u, 0x0A8u,
    0x0A2u, 0x08Au, 0x02Au, 0x094u
};

/* ASCII character table matching indices 0..43. */
static uint8_t const g_code39_ascii[CODE39_TABLE_SIZE] =
{
    (uint8_t) '0', (uint8_t) '1', (uint8_t) '2', (uint8_t) '3',
    (uint8_t) '4', (uint8_t) '5', (uint8_t) '6', (uint8_t) '7',
    (uint8_t) '8', (uint8_t) '9', (uint8_t) 'A', (uint8_t) 'B',
    (uint8_t) 'C', (uint8_t) 'D', (uint8_t) 'E', (uint8_t) 'F',
    (uint8_t) 'G', (uint8_t) 'H', (uint8_t) 'I', (uint8_t) 'J',
    (uint8_t) 'K', (uint8_t) 'L', (uint8_t) 'M', (uint8_t) 'N',
    (uint8_t) 'O', (uint8_t) 'P', (uint8_t) 'Q', (uint8_t) 'R',
    (uint8_t) 'S', (uint8_t) 'T', (uint8_t) 'U', (uint8_t) 'V',
    (uint8_t) 'W', (uint8_t) 'X', (uint8_t) 'Y', (uint8_t) 'Z',
    (uint8_t) '-', (uint8_t) '.', (uint8_t) ' ', (uint8_t) '$',
    (uint8_t) '/', (uint8_t) '+', (uint8_t) '%', (uint8_t) '*'
};

/* ---- private function prototypes ------------------------------------- */

static void     code39_sort_widths(uint8_t const * p_src, uint8_t * p_sorted);
static uint16_t code39_build_pattern(uint8_t const * p_widths,
                                     uint32_t start_idx,
                                     uint32_t threshold);
static uint32_t code39_lookup_pattern(uint16_t pattern);
static uint32_t code39_try_direction(uint8_t const * p_widths,
                                     uint32_t threshold,
                                     uint32_t * p_char_index);

static uint32_t code39_try_both(uint8_t const * p_widths,
                                uint32_t threshold,
                                uint32_t * p_char_index);

/* ---- public functions ------------------------------------------------- */

/**
 * code39_decode
 * @brief   Decodes a 29-element `*<char>*` Code 39 capture in either
 *          forward or reverse order.
 * @param   p_widths      Array of 29 element widths (0.1 mm units).
 * @param   count         Number of elements captured (must be 29).
 * @param   p_char_index  Output data character index (0..42) on success.
 * @return  CODE39_OK (0) on valid decode, CODE39_INVALID (1) otherwise.
 * @context barcode_task
 */
uint32_t code39_decode(uint8_t const * p_widths, uint32_t count,
                       uint32_t * p_char_index)
{
    uint32_t result = CODE39_INVALID;

    if ((NULL != p_widths) && (NULL != p_char_index) &&
        (BAR_ELEMENTS == count))
    {
        uint8_t  sorted[BAR_ELEMENTS] = {0u};
        uint32_t narrow_max           = 0u;
        uint32_t wide_min             = 0u;

        code39_sort_widths(p_widths, sorted);
        narrow_max = (uint32_t) sorted[CODE39_NARROW_MAX_IDX]; /* cast: widen */
        wide_min   = (uint32_t) sorted[CODE39_WIDE_MIN_IDX];   /* cast: widen */

        if ((narrow_max > 0u) &&
            ((wide_min * CODE39_RATIO_SCALE) >=
             (narrow_max * BAR_WIDE_RATIO_X10_MIN)))
        {
            uint32_t const threshold = ((narrow_max + wide_min) + 1u) / 2u;

            result = code39_try_both(p_widths, threshold, p_char_index);
        }
        else
        {
            /* Insufficient wide-to-narrow ratio. */
        }
    }
    else
    {
        /* Null pointer or element count != 29. */
    }

    return result;
}

/**
 * code39_char_ascii
 * @brief   Returns the ASCII character code for a Code 39 index (0..43).
 * @param   char_index  Character index in 0..43.
 * @return  ASCII byte value as uint32_t, or '?' if out of range.
 * @context any
 */
uint32_t code39_char_ascii(uint32_t char_index)
{
    uint32_t ascii_ch = CODE39_ASCII_UNKNOWN;

    if (char_index < CODE39_TABLE_SIZE)
    {
        /* cast: widen 8-bit ASCII code to uint32_t */
        ascii_ch = (uint32_t) g_code39_ascii[char_index];
    }
    else
    {
        /* Out-of-range character index returns '?'. */
    }

    return ascii_ch;
}

/* ---- private functions ------------------------------------------------ */

/**
 * code39_sort_widths
 * @brief   Copies 29 element widths and insertion-sorts them ascending.
 * @param   p_src     Input array of 29 widths.
 * @param   p_sorted  Output sorted array of 29 widths.
 * @return  none
 * @context barcode_task
 */
static void code39_sort_widths(uint8_t const * p_src, uint8_t * p_sorted)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < BAR_ELEMENTS; idx++)
    {
        uint8_t const val = p_src[idx];
        uint32_t      pos = idx;

        while ((pos > 0u) && (p_sorted[pos - 1u] > val))
        {
            p_sorted[pos] = p_sorted[pos - 1u];
            pos--;
        }
        p_sorted[pos] = val;
    }
}

/**
 * code39_build_pattern
 * @brief   Builds a 9-bit wide/narrow pattern starting at start_idx.
 * @param   p_widths   Array of 29 element widths.
 * @param   start_idx  Starting element index (0, 10, or 20).
 * @param   threshold  Wide element threshold.
 * @return  9-bit pattern (bit 8 = first element, bit 0 = ninth element).
 * @context barcode_task
 */
static uint16_t code39_build_pattern(uint8_t const * p_widths,
                                     uint32_t start_idx,
                                     uint32_t threshold)
{
    uint16_t pattern = 0u;
    uint32_t idx     = 0u;

    for (idx = 0u; idx < CODE39_CHAR_ELEMENTS; idx++)
    {
        uint32_t const width =
            (uint32_t) p_widths[start_idx + idx]; /* cast: widen */

        pattern = (uint16_t) (pattern << 1u); /* cast: 9-bit pattern */
        if (width >= threshold)
        {
            pattern = (uint16_t) (pattern | 1u); /* cast: 9-bit pattern */
        }
        else
        {
            /* Narrow element leaves bit 0 clear. */
        }
    }

    return pattern;
}

/**
 * code39_lookup_pattern
 * @brief   Looks up a 9-bit pattern in g_code39_patterns.
 * @param   pattern  9-bit wide/narrow bitmask.
 * @return  Character index 0..43, or CODE39_NOT_FOUND (0xFF).
 * @context barcode_task
 */
static uint32_t code39_lookup_pattern(uint16_t pattern)
{
    uint32_t found_idx = CODE39_NOT_FOUND;
    uint32_t idx       = 0u;

    for (idx = 0u; idx < CODE39_TABLE_SIZE; idx++)
    {
        if (g_code39_patterns[idx] == pattern)
        {
            found_idx = idx;
        }
        else
        {
            /* Continue scan. */
        }
    }

    return found_idx;
}

/**
 * code39_try_direction
 * @brief   Checks inter-character gaps and decodes start, data, and stop
 *          characters in the given element order.
 * @param   p_widths      Array of 29 element widths.
 * @param   threshold     Wide element threshold.
 * @param   p_char_index  Output data character index (0..42) if valid.
 * @return  CODE39_OK or CODE39_INVALID.
 * @context barcode_task
 */
static uint32_t code39_try_direction(uint8_t const * p_widths,
                                     uint32_t threshold,
                                     uint32_t * p_char_index)
{
    uint32_t       result = CODE39_INVALID;
    uint32_t const gap1   = (uint32_t) p_widths[CODE39_GAP1_IDX]; /* cast */
    uint32_t const gap2   = (uint32_t) p_widths[CODE39_GAP2_IDX]; /* cast */

    if ((gap1 < threshold) && (gap2 < threshold))
    {
        uint16_t const pat0 =
            code39_build_pattern(p_widths, CODE39_CHAR0_START, threshold);
        uint16_t const pat1 =
            code39_build_pattern(p_widths, CODE39_CHAR1_START, threshold);
        uint16_t const pat2 =
            code39_build_pattern(p_widths, CODE39_CHAR2_START, threshold);
        uint32_t const ch0  = code39_lookup_pattern(pat0);
        uint32_t const ch1  = code39_lookup_pattern(pat1);
        uint32_t const ch2  = code39_lookup_pattern(pat2);

        if ((CODE39_STAR == ch0) && (CODE39_STAR == ch2) &&
            (ch1 < APP_C39_CHAR_COUNT))
        {
            *p_char_index = ch1;
            result        = CODE39_OK;
        }
        else
        {
            /* Delimiters or data character did not match. */
        }
    }
    else
    {
        /* Inter-character gap was wide: invalid framing. */
    }

    return result;
}

/**
 * code39_try_both
 * @brief   Attempts forward decode, then reversed element order if needed.
 * @param   p_widths      Array of 29 element widths.
 * @param   threshold     Wide element threshold.
 * @param   p_char_index  Output data character index (0..42) if valid.
 * @return  CODE39_OK or CODE39_INVALID.
 * @context barcode_task
 */
static uint32_t code39_try_both(uint8_t const * p_widths,
                                uint32_t threshold,
                                uint32_t * p_char_index)
{
    uint32_t result = code39_try_direction(p_widths, threshold, p_char_index);

    if (CODE39_OK != result)
    {
        uint8_t  rev[BAR_ELEMENTS] = {0u};
        uint32_t idx               = 0u;

        for (idx = 0u; idx < BAR_ELEMENTS; idx++)
        {
            rev[idx] = p_widths[(BAR_ELEMENTS - 1u) - idx];
        }
        result = code39_try_direction(rev, threshold, p_char_index);
    }
    else
    {
        /* Forward direction matched. */
    }

    return result;
}

/*** end of file ***/
