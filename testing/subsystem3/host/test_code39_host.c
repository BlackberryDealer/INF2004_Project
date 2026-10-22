/*
 *  test_code39_host.c
 *  Host unit tests for subsystem 3's pure code39 module:
 *    S3-U03  Full-scale widths decode all 43 characters both directions
 *    S3-U04  Half-scale widths decode all 43 characters both directions
 *    S3-U05  28-element capture rejected; pattern table has 3 set bits per
 *            entry and all 44 entries are unique; invalid ratio/gaps rejected
 */

#include "host_test.h"
#include "cfg_barcode.h"
#include "code39.h"

#define TABLE_COUNT (44u)

/* Reference pattern table from spec B3.2.5 to build synthetic captures. */
static uint16_t const g_ref_patterns[TABLE_COUNT] =
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

static void encode_char_widths(uint16_t pattern, uint8_t narrow_w,
                               uint8_t wide_w, uint8_t * p_out)
{
    uint32_t bit = 0u;

    for (bit = 0u; bit < 9u; bit++)
    {
        uint16_t const mask = (uint16_t) (1u << (8u - bit));
        /* Add +-1 unit deterministic jitter to mimic real edge capture. */
        uint8_t const  jit  = (0u == (bit & 1u)) ? 1u : 0u;

        p_out[bit] = (0u != (pattern & mask))
                         ? (uint8_t) (wide_w - jit)
                         : (uint8_t) (narrow_w + jit);
    }
}

static void build_frame(uint32_t char_idx, uint8_t narrow_w, uint8_t wide_w,
                        bool b_reverse, uint8_t * p_widths)
{
    uint8_t  fwd[BAR_ELEMENTS] = {0u};
    uint32_t idx               = 0u;

    encode_char_widths(g_ref_patterns[CODE39_STAR], narrow_w, wide_w, &fwd[0]);
    fwd[9] = narrow_w;
    encode_char_widths(g_ref_patterns[char_idx], narrow_w, wide_w, &fwd[10]);
    fwd[19] = narrow_w;
    encode_char_widths(g_ref_patterns[CODE39_STAR], narrow_w, wide_w, &fwd[20]);

    for (idx = 0u; idx < BAR_ELEMENTS; idx++)
    {
        p_widths[idx] = b_reverse ? fwd[(BAR_ELEMENTS - 1u) - idx] : fwd[idx];
    }
}

static bool check_all_chars(uint8_t narrow_w, uint8_t wide_w, bool b_reverse)
{
    uint32_t ch    = 0u;
    bool     b_all = true;

    for (ch = 0u; ch < APP_C39_CHAR_COUNT; ch++)
    {
        uint8_t  widths[BAR_ELEMENTS] = {0u};
        uint32_t out_idx              = 99u;

        build_frame(ch, narrow_w, wide_w, b_reverse, widths);
        if ((CODE39_OK != code39_decode(widths, BAR_ELEMENTS, &out_idx)) ||
            (out_idx != ch))
        {
            b_all = false;
        }
    }

    return b_all;
}

static void check_pattern_table(bool * p_three_bits, bool * p_unique)
{
    uint32_t idx_a        = 0u;
    bool     b_three_bits = true;
    bool     b_unique     = true;

    for (idx_a = 0u; idx_a < TABLE_COUNT; idx_a++)
    {
        uint16_t pat   = g_ref_patterns[idx_a];
        uint32_t ones  = 0u;
        uint32_t bit   = 0u;
        uint32_t idx_b = 0u;

        for (bit = 0u; bit < 9u; bit++)
        {
            if (0u != (pat & (uint16_t) (1u << bit)))
            {
                ones++;
            }
        }
        if (3u != ones)
        {
            b_three_bits = false;
        }

        for (idx_b = idx_a + 1u; idx_b < TABLE_COUNT; idx_b++)
        {
            if (g_ref_patterns[idx_b] == pat)
            {
                b_unique = false;
            }
        }
    }

    *p_three_bits = b_three_bits;
    *p_unique     = b_unique;
}

int main(void)
{
    uint8_t  widths[BAR_ELEMENTS] = {0u};
    uint32_t out_idx              = 0u;
    bool     b_three_bits         = false;
    bool     b_unique             = false;

    HOST_INIT();

    /* S3-U03: Full scale (narrow = 3.0 mm = 30, wide = 7.5 mm = 75) */
    HOST_CHECK("S3-U03", check_all_chars(30u, 75u, false),
               "full-scale forward decodes all 43 Code 39 characters");
    HOST_CHECK("S3-U03", check_all_chars(30u, 75u, true),
               "full-scale reverse decodes all 43 Code 39 characters");
    HOST_CHECK("S3-U03",
               ((uint32_t) 'A' == code39_char_ascii(10u)) &&
               ((uint32_t) '*' == code39_char_ascii(CODE39_STAR)) &&
               ((uint32_t) '?' == code39_char_ascii(99u)),
               "code39_char_ascii maps indices and out-of-range");

    /* S3-U04: Half scale (narrow = 1.5 mm = 15, wide = 3.8 mm = 38) */
    HOST_CHECK("S3-U04", check_all_chars(15u, 38u, false),
               "half-scale forward decodes all 43 Code 39 characters");
    HOST_CHECK("S3-U04", check_all_chars(15u, 38u, true),
               "half-scale reverse decodes all 43 Code 39 characters");

    /* S3-U05: Table invariants and invalid inputs */
    check_pattern_table(&b_three_bits, &b_unique);
    HOST_CHECK("S3-U05", b_three_bits,
               "every Code 39 pattern has exactly three wide (1) bits");
    HOST_CHECK("S3-U05", b_unique,
               "all 44 Code 39 patterns are unique");

    build_frame(10u, 20u, 50u, false, widths);
    HOST_CHECK("S3-U05",
               CODE39_INVALID == code39_decode(widths, 28u, &out_idx),
               "28-element capture is rejected");
    HOST_CHECK("S3-U05",
               CODE39_INVALID == code39_decode(widths, 30u, &out_idx),
               "30-element capture is rejected");
    HOST_CHECK("S3-U05",
               CODE39_INVALID == code39_decode((uint8_t const *) 0,
                                               BAR_ELEMENTS, &out_idx),
               "NULL widths pointer is rejected");

    /* Low wide/narrow ratio: 20 and 35 -> 35/20 = 1.75 < 1.8 */
    build_frame(10u, 20u, 35u, false, widths);
    HOST_CHECK("S3-U05",
               CODE39_INVALID == code39_decode(widths, BAR_ELEMENTS, &out_idx),
               "wide/narrow ratio < 1.8 is rejected");

    /* Wide inter-character gap */
    build_frame(10u, 20u, 50u, false, widths);
    widths[9] = 50u;
    HOST_CHECK("S3-U05",
               CODE39_INVALID == code39_decode(widths, BAR_ELEMENTS, &out_idx),
               "wide inter-character gap is rejected");

    return (0 == HOST_SUMMARY("code39")) ? 0 : 1;
}
