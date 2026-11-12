/**
 * @file    code39.c
 * @brief   Pure Code 39 width-to-character decoder in both directions.
 * @owner   Buddy 3 (line, barcode)
 * @kind    algo
 * @reqs    FR2, NFR5
 */

#include "code39.h"
#include "cfg_barcode.h"

/* ---- private constants and types ------------------------------------ */

#define CODE39_TABLE_SIZE      (44u)
#define CODE39_WIDTHS_NULL     ((uint8_t const *) 0)
#define CODE39_CHAR_NULL       ((uint32_t *) 0)
#define CODE39_CHAR_ELEMENTS   (9u)
#define CODE39_BARS_PER_CHAR   (5u)
#define CODE39_SPACES_PER_CHAR (4u)
#define CODE39_BAR_NARROW_LAST (2u)
#define CODE39_BAR_WIDE_FIRST  (3u)
#define CODE39_BAR_MAX_IDX     (4u)
#define CODE39_SP_NARROW_LAST  (2u)
#define CODE39_SP_WIDE_FIRST   (3u)
#define CODE39_SP_SPEC_WIDE    (1u)
#define CODE39_CHAR0_START     (0u)
#define CODE39_GAP1_IDX        (9u)
#define CODE39_CHAR1_START     (10u)
#define CODE39_GAP2_IDX        (19u)
#define CODE39_CHAR2_START     (20u)
#define CODE39_NARROW_MAX_IDX  (19u)
#define CODE39_WIDE_MIN_IDX    (20u)
#define CODE39_RATIO_SCALE     (10u)
/* Uneven black widths on the 14:34 run compressed the bar ratio to 1.42.
   Allow that ratio for a retry trigger. Decoding uses the standard ratio. */
#define CODE39_GUARD_BAR_RATIO_X10 (14u)
#define CODE39_NOT_FOUND       (0xFFu)
/* Bars that are narrow in every valid frame, read either way: the first
   and last bar of each '*' guard (elements 1, 9, 21 and 29). */
#define CODE39_FIRST_BAR_IDX   (0u)
#define CODE39_START_END_BAR   (8u)
#define CODE39_STOP_FIRST_BAR  (20u)
#define CODE39_LAST_BAR_IDX    (28u)
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

static void     code39_sort_n(uint8_t const * p_src, uint8_t * p_sorted,
                              uint32_t len);
static uint16_t code39_build_pattern(uint8_t const * p_widths,
                                     uint32_t start_idx,
                                     uint32_t thr_bar,
                                     uint32_t thr_space);
static uint32_t code39_lookup_pattern(uint16_t pattern);
static uint32_t code39_try_direction(uint8_t const * p_widths,
                                     uint32_t threshold,
                                     uint32_t * p_char_index);
static uint32_t code39_try_both(uint8_t const * p_widths,
                                uint32_t threshold,
                                uint32_t * p_char_index);
static uint32_t code39_decode_char_split(uint8_t const * p_widths,
                                         uint32_t start_idx,
                                         uint32_t * p_char_out,
                                         uint32_t * p_thr_space);
static uint32_t code39_try_split_dir(uint8_t const * p_widths,
                                     uint32_t * p_char_index);
static uint32_t code39_try_split_both(uint8_t const * p_widths,
                                      uint32_t * p_char_index);

static bool     code39_guard_matches(uint8_t const * p_widths,
                                       uint32_t first, bool b_reverse);
static void     code39_narrow_ends(uint8_t const * p_widths,
                                   uint8_t * p_adjusted);

/* ---- public functions ------------------------------------------------- */

/**
 * code39_decode
 * @brief   Decodes a 29-element `*<char>*` Code 39 capture in either
 *          forward or reverse order. Falls back to separate per-character
 *          bar/space thresholding when optical spot bias shifts bar vs
 *          space widths on small or high-contrast barcodes. The first
 *          and last bars are capped to a narrow guard bar first (S3-U43).
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

    if ((CODE39_WIDTHS_NULL != p_widths) &&
        (CODE39_CHAR_NULL != p_char_index) &&
        (BAR_ELEMENTS == count))
    {
        uint8_t  adjusted[BAR_ELEMENTS] = {0u};
        uint8_t  sorted[BAR_ELEMENTS]   = {0u};
        uint32_t narrow_max             = 0u;
        uint32_t wide_min               = 0u;

        code39_narrow_ends(p_widths, adjusted);
        code39_sort_n(adjusted, sorted, BAR_ELEMENTS);
        narrow_max = (uint32_t) sorted[CODE39_NARROW_MAX_IDX]; /* cast: widen */
        wide_min   = (uint32_t) sorted[CODE39_WIDE_MIN_IDX];   /* cast: widen */

        if ((narrow_max > 0u) &&
            ((wide_min * CODE39_RATIO_SCALE) >=
             (narrow_max * BAR_WIDE_RATIO_X10_MIN)))
        {
            uint32_t const threshold = ((narrow_max + wide_min) + 1u) / 2u;

            result = code39_try_both(adjusted, threshold, p_char_index);
        }
        else
        {
            /* Pooled ratio < 1.8; try separate bar/space thresholds below. */
        }

        if (CODE39_OK != result)
        {
            result = code39_try_split_both(adjusted, p_char_index);
        }
        else
        {
            /* Pooled global threshold already decoded valid frame. */
        }
    }
    else
    {
        /* Null pointer or element count != 29. */
    }

    return result;
}

/**
 * code39_has_start_guard
 * @brief   Check a Code 39 start guard and narrow inter-character gap.
 *          A nine-width input is a candidate whose first narrow bar
 *          was missed. This can request a reread, never a decode.
 * @param   p_widths  Observed widths in travel order.
 * @param   count     Guard and gap count, with or without first bar.
 * @return  true if the measured widths match either guard orientation.
 * @context barcode_task
 */
bool code39_has_start_guard(uint8_t const * p_widths, uint32_t count)
{
    bool b_matches = false;

    if ((CODE39_WIDTHS_NULL != p_widths)
        && ((CODE39_START_ELEMENTS == count)
            || ((CODE39_START_ELEMENTS - 1u) == count)))
    {
        uint32_t const first = CODE39_START_ELEMENTS - count;

        b_matches = code39_guard_matches(p_widths, first, false)
                    || code39_guard_matches(p_widths, first, true);
    }

    return b_matches;
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
 * code39_narrow_ends
 * @brief   Copies the 29 widths and caps the first and the last bar at the
 *          other end bar of the same '*' guard. Every valid frame is *X*,
 *          and read in either direction each '*' begins and ends with a
 *          narrow bar, so a long first or last bar (10.8 mm against 7 mm
 *          narrow bars on-car 12 Nov) is a measuring error. The full
 *          pattern check of both guards and the data character still runs.
 * @param   p_widths    Array of 29 element widths (0.1 mm units).
 * @param   p_adjusted  Output array of 29 widths with the end bars capped.
 * @return  none
 * @context barcode_task
 */
static void code39_narrow_ends(uint8_t const * p_widths,
                               uint8_t * p_adjusted)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < BAR_ELEMENTS; idx++)
    {
        p_adjusted[idx] = p_widths[idx];
    }

    if ((0u != p_widths[CODE39_START_END_BAR]) &&
        (p_widths[CODE39_START_END_BAR] < p_widths[CODE39_FIRST_BAR_IDX]))
    {
        p_adjusted[CODE39_FIRST_BAR_IDX] = p_widths[CODE39_START_END_BAR];
    }
    else
    {
        /* First bar already no wider than the guard's last bar. */
    }

    if ((0u != p_widths[CODE39_STOP_FIRST_BAR]) &&
        (p_widths[CODE39_STOP_FIRST_BAR] < p_widths[CODE39_LAST_BAR_IDX]))
    {
        p_adjusted[CODE39_LAST_BAR_IDX] = p_widths[CODE39_STOP_FIRST_BAR];
    }
    else
    {
        /* Last bar already no wider than the guard's first bar. */
    }
}

/**
 * code39_sort_n
 * @brief   Copies len element widths and insertion-sorts them ascending.
 * @param   p_src     Input array of len widths.
 * @param   p_sorted  Output sorted array of len widths.
 * @param   len       Number of elements to sort.
 * @return  none
 * @context barcode_task
 */
static void code39_sort_n(uint8_t const * p_src, uint8_t * p_sorted,
                          uint32_t len)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < len; idx++)
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
 * @brief   Builds a 9-bit wide/narrow pattern starting at start_idx using
 *          thr_bar for even offsets (bars) and thr_space for odd (spaces).
 * @param   p_widths   Array of 29 element widths.
 * @param   start_idx  Starting element index (0, 10, or 20).
 * @param   thr_bar    Wide threshold for bar elements (even offsets).
 * @param   thr_space  Wide threshold for space elements (odd offsets).
 * @return  9-bit pattern (bit 8 = first element, bit 0 = ninth element).
 * @context barcode_task
 */
static uint16_t code39_build_pattern(uint8_t const * p_widths,
                                     uint32_t start_idx,
                                     uint32_t thr_bar,
                                     uint32_t thr_space)
{
    uint16_t pattern = 0u;
    uint32_t idx     = 0u;

    for (idx = 0u; idx < CODE39_CHAR_ELEMENTS; idx++)
    {
        uint32_t const width =
            (uint32_t) p_widths[start_idx + idx]; /* cast: widen */
        uint32_t const thr = (0u == (idx & 1u)) ? thr_bar : thr_space;

        pattern = (uint16_t) (pattern << 1u); /* cast: 9-bit pattern */
        if (width >= thr)
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
 *          characters in the given element order using one threshold.
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
        uint16_t const pat0 = code39_build_pattern(
            p_widths, CODE39_CHAR0_START, threshold, threshold);
        uint16_t const pat1 = code39_build_pattern(
            p_widths, CODE39_CHAR1_START, threshold, threshold);
        uint16_t const pat2 = code39_build_pattern(
            p_widths, CODE39_CHAR2_START, threshold, threshold);
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

/**
 * code39_decode_char_split
 * @brief   Decodes one 9-element character using separate thresholds for
 *          its 5 bars (even offsets) and 4 spaces (odd offsets).
 * @param   p_widths     Array of 29 element widths.
 * @param   start_idx    Character start offset (0, 10, or 20).
 * @param   p_char_out   Output character index (0..43).
 * @param   p_thr_space  Output space threshold for inter-char gap check.
 * @return  CODE39_OK or CODE39_INVALID.
 * @context barcode_task
 */
static uint32_t code39_decode_char_split(uint8_t const * p_widths,
                                         uint32_t start_idx,
                                         uint32_t * p_char_out,
                                         uint32_t * p_thr_space)
{
    uint8_t  raw_b[CODE39_BARS_PER_CHAR]     = {0u};
    uint8_t  raw_s[CODE39_SPACES_PER_CHAR]   = {0u};
    uint8_t  bars[CODE39_BARS_PER_CHAR]      = {0u};
    uint8_t  spaces[CODE39_SPACES_PER_CHAR]  = {0u};
    uint32_t idx                             = 0u;
    uint32_t result                          = CODE39_INVALID;

    for (idx = 0u; idx < CODE39_BARS_PER_CHAR; idx++)
    {
        raw_b[idx] = p_widths[start_idx + (idx * 2u)];
    }
    for (idx = 0u; idx < CODE39_SPACES_PER_CHAR; idx++)
    {
        raw_s[idx] = p_widths[start_idx + (idx * 2u) + 1u];
    }

    code39_sort_n(raw_b, bars, CODE39_BARS_PER_CHAR);
    code39_sort_n(raw_s, spaces, CODE39_SPACES_PER_CHAR);

    if ((bars[0] > 0u) && (spaces[0] > 0u))
    {
        /* casts: widen 8-bit element widths to uint32_t */
        uint32_t const b_nar = (uint32_t) bars[CODE39_BAR_NARROW_LAST];
        uint32_t const b_wid = (uint32_t) bars[CODE39_BAR_WIDE_FIRST];
        uint32_t const s_nar = (uint32_t) spaces[CODE39_SP_NARROW_LAST];
        uint32_t const s_wid = (uint32_t) spaces[CODE39_SP_WIDE_FIRST];
        uint32_t       thr_b = 0u;
        uint32_t       thr_s = 0u;

        if (((b_wid * CODE39_RATIO_SCALE) >=
             (b_nar * BAR_WIDE_RATIO_X10_MIN)) &&
            ((s_wid * CODE39_RATIO_SCALE) >=
             (s_nar * BAR_WIDE_RATIO_X10_MIN)))
        {
            thr_b = ((b_nar + b_wid) + 1u) / 2u;
            thr_s = ((s_nar + s_wid) + 1u) / 2u;
        }
        else if ((((uint32_t) spaces[CODE39_SP_SPEC_WIDE] *
                   CODE39_RATIO_SCALE) >=
                  ((uint32_t) spaces[0] * BAR_WIDE_RATIO_X10_MIN)) &&
                 (((uint32_t) bars[CODE39_BAR_MAX_IDX] * CODE39_RATIO_SCALE) <
                  ((uint32_t) bars[0] * BAR_WIDE_RATIO_X10_MIN)))
        {
            /* Special chars ($, /, +, %): 5 narrow bars + 3 wide spaces. */
            thr_b = (uint32_t) bars[CODE39_BAR_MAX_IDX] + 1u; /* cast: widen */
            thr_s = (((uint32_t) spaces[0] +
                      (uint32_t) spaces[CODE39_SP_SPEC_WIDE]) + 1u) / 2u;
        }
        else
        {
            /* Neither standard (2B+1S) nor special (0B+3S) ratio met. */
        }

        if ((thr_b > 0u) && (thr_s > 0u))
        {
            uint16_t const pat =
                code39_build_pattern(p_widths, start_idx, thr_b, thr_s);
            uint32_t const ch  = code39_lookup_pattern(pat);

            if (CODE39_NOT_FOUND != ch)
            {
                *p_char_out  = ch;
                *p_thr_space = thr_s;
                result       = CODE39_OK;
            }
            else
            {
                /* Pattern did not match any Code 39 character. */
            }
        }
        else
        {
            /* Ratios below minimum threshold. */
        }
    }
    else
    {
        /* Zero-width element is invalid. */
    }

    return result;
}

/**
 * code39_try_split_dir
 * @brief   Decodes a 29-element frame in one direction using separate
 *          per-character bar and space thresholds.
 * @param   p_widths      Array of 29 element widths.
 * @param   p_char_index  Output data character index (0..42) if valid.
 * @return  CODE39_OK or CODE39_INVALID.
 * @context barcode_task
 */
static uint32_t code39_try_split_dir(uint8_t const * p_widths,
                                     uint32_t * p_char_index)
{
    uint32_t result = CODE39_INVALID;
    uint32_t ch0    = CODE39_NOT_FOUND;
    uint32_t ch1    = CODE39_NOT_FOUND;
    uint32_t ch2    = CODE39_NOT_FOUND;
    uint32_t thr_s0 = 0u;
    uint32_t thr_s1 = 0u;
    uint32_t thr_s2 = 0u;

    if ((CODE39_OK == code39_decode_char_split(
            p_widths, CODE39_CHAR0_START, &ch0, &thr_s0)) &&
        (CODE39_OK == code39_decode_char_split(
            p_widths, CODE39_CHAR1_START, &ch1, &thr_s1)) &&
        (CODE39_OK == code39_decode_char_split(
            p_widths, CODE39_CHAR2_START, &ch2, &thr_s2)))
    {
        uint32_t const gap1 = (uint32_t) p_widths[CODE39_GAP1_IDX]; /* cast */
        uint32_t const gap2 = (uint32_t) p_widths[CODE39_GAP2_IDX]; /* cast */

        if ((CODE39_STAR == ch0) && (CODE39_STAR == ch2) &&
            (ch1 < APP_C39_CHAR_COUNT) &&
            (gap1 < thr_s0) && (gap1 < thr_s1) &&
            (gap2 < thr_s1) && (gap2 < thr_s2))
        {
            *p_char_index = ch1;
            result        = CODE39_OK;
        }
        else
        {
            /* Delimiters, data char range, or inter-char gaps invalid. */
        }
    }
    else
    {
        /* At least one character failed split bar/space decode. */
    }

    return result;
}

/**
 * code39_try_split_both
 * @brief   Attempts split bar/space decode in forward then reverse order.
 * @param   p_widths      Array of 29 element widths.
 * @param   p_char_index  Output data character index (0..42) if valid.
 * @return  CODE39_OK or CODE39_INVALID.
 * @context barcode_task
 */
static uint32_t code39_try_split_both(uint8_t const * p_widths,
                                      uint32_t * p_char_index)
{
    uint32_t result = code39_try_split_dir(p_widths, p_char_index);

    if (CODE39_OK != result)
    {
        uint8_t  rev[BAR_ELEMENTS] = {0u};
        uint32_t idx               = 0u;

        for (idx = 0u; idx < BAR_ELEMENTS; idx++)
        {
            rev[idx] = p_widths[(BAR_ELEMENTS - 1u) - idx];
        }
        result = code39_try_split_dir(rev, p_char_index);
    }
    else
    {
        /* Forward split decode succeeded. */
    }

    return result;
}

/**
 * code39_guard_matches
 * @brief   Compare the widest narrow and narrowest wide element in each
 *          of the bar and space groups against the expected guard mask.
 * @context barcode_task
 */
static bool code39_guard_matches(uint8_t const * p_widths,
                                  uint32_t first, bool b_reverse)
{
    uint32_t const star = (uint32_t) g_code39_patterns[CODE39_STAR];
    uint32_t const pattern = b_reverse ? star : (star << 1u);
    uint32_t narrow_max[2] = {0u, 0u};
    uint32_t wide_min[2] = {UINT8_MAX, UINT8_MAX};
    bool b_valid = true;

    for (uint32_t idx = first; idx < CODE39_START_ELEMENTS; idx++)
    {
        uint32_t const width = (uint32_t) p_widths[idx - first];
        uint32_t const group = idx & 1u;
        /* Forward uses a trailing zero for the gap. In reverse the gap
           is one bit past the high end of the guard. */
        uint32_t const bit = b_reverse ? idx
                                     : (CODE39_START_ELEMENTS - 1u) - idx;

        if (0u == width)
        {
            b_valid = false;
        }
        if (0u != (pattern & (1u << bit)))
        {
            wide_min[group] = (width < wide_min[group]) ? width
                                                       : wide_min[group];
        }
        else
        {
            narrow_max[group] = (width > narrow_max[group]) ? width
                                                           : narrow_max[group];
        }
    }

    return b_valid
        && ((wide_min[0] * CODE39_RATIO_SCALE)
            >= (narrow_max[0] * CODE39_GUARD_BAR_RATIO_X10))
        && ((wide_min[1] * CODE39_RATIO_SCALE)
            >= (narrow_max[1] * BAR_WIDE_RATIO_X10_MIN));
}

/*** end of file ***/
