/**
 * @file    test_line.c
 * @brief   Phase 1 on-target ADC verification and bench measurement helpers.
 * @owner   Buddy 3 (line, barcode)
 * @kind    test
 * @reqs    FR1, NFR1
 */

#include "test_line.h"
#include "../app.h"

#if (0 != APP_TEST)

#include <stddef.h>
#include "../test_report.h"
#include "cfg_line.h"
#include "drv_ir_adc.h"

/* ---- private constants and types ------------------------------------ */

#define TEST_LINE_ADC_MAX_CODE       (4095u)
#define TEST_LINE_BENCH_SAMPLES      (10u)
#define TEST_LINE_NOISE_SAMPLES      (1000u)
#define TEST_LINE_ENABLE_NOISE_BENCH (0)

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

static void test_line_adc_sanity(void);
static void test_line_noise_bench(void);

/* ---- public functions ------------------------------------------------- */

/**
 * test_line_run
 * @brief   Runs Phase 1 on-target checks for drv_ir_adc.
 * @return  none
 * @context usermain (APP_PRI_TEST)
 */
void test_line_run(void)
{
    TEST_INIT();
    test_line_adc_sanity();
    test_line_noise_bench();
    TEST_SUMMARY("line");
}

/* ---- private functions ------------------------------------------------ */

/**
 * test_line_adc_sanity
 * @brief   Verifies drv_ir_adc_read returns 12-bit values and logs samples
 *          for S3-I05 contrast bench checks.
 * @return  none
 * @context usermain (APP_PRI_TEST)
 */
static void test_line_adc_sanity(void)
{
    uint32_t raw_l    = 0u;
    uint32_t raw_r    = 0u;
    uint32_t idx      = 0u;
    bool     b_in_rng = true;

    drv_ir_adc_init();

    for (idx = 0u; idx < TEST_LINE_BENCH_SAMPLES; idx++)
    {
        drv_ir_adc_read(&raw_l, &raw_r);
        if ((raw_l > TEST_LINE_ADC_MAX_CODE) ||
            (raw_r > TEST_LINE_ADC_MAX_CODE))
        {
            b_in_rng = false;
        }
        else
        {
            /* within valid 12-bit ADC range */
        }
        app_log("S3-I05 sample %d: L=%d R=%d\n",
                (int32_t) idx,   /* cast: idx < 10 fits int32_t */
                (int32_t) raw_l, /* cast: 12-bit ADC fits int32_t */
                (int32_t) raw_r  /* cast: 12-bit ADC fits int32_t */);
        (void) tk_dly_tsk(10); /* ignored: bench delay only */
    }

    /* Null pointer safety check */
    drv_ir_adc_read(&raw_l, NULL);
    drv_ir_adc_read(NULL, &raw_r);

    TEST_CHECK("S3-I05-SANITY", b_in_rng, "ADC reads within 12-bit range");
}

/**
 * test_line_noise_bench
 * @brief   Optional 1000-sample min/max/mean logger for S3-I06 (A19 noise).
 * @return  none
 * @context usermain (APP_PRI_TEST)
 */
static void test_line_noise_bench(void)
{
#if (0 != TEST_LINE_ENABLE_NOISE_BENCH)
    uint32_t idx   = 0u;
    uint32_t min_l = TEST_LINE_ADC_MAX_CODE;
    uint32_t max_l = 0u;
    uint32_t min_r = TEST_LINE_ADC_MAX_CODE;
    uint32_t max_r = 0u;

    for (idx = 0u; idx < TEST_LINE_NOISE_SAMPLES; idx++)
    {
        uint32_t raw_l = 0u;
        uint32_t raw_r = 0u;

        drv_ir_adc_read(&raw_l, &raw_r);
        if (raw_l < min_l)
        {
            min_l = raw_l;
        }
        else
        {
            /* keep min_l */
        }
        if (raw_l > max_l)
        {
            max_l = raw_l;
        }
        else
        {
            /* keep max_l */
        }
        if (raw_r < min_r)
        {
            min_r = raw_r;
        }
        else
        {
            /* keep min_r */
        }
        if (raw_r > max_r)
        {
            max_r = raw_r;
        }
        else
        {
            /* keep max_r */
        }
        (void) tk_dly_tsk(1); /* ignored: yield between samples */
    }

    app_log("S3-I06 L min=%d max=%d span=%d\n",
            (int32_t) min_l, /* cast: 12-bit fits int32_t */
            (int32_t) max_l, /* cast: 12-bit fits int32_t */
            (int32_t) (max_l - min_l) /* cast: 12-bit fits int32_t */);
    app_log("S3-I06 R min=%d max=%d span=%d\n",
            (int32_t) min_r, /* cast: 12-bit fits int32_t */
            (int32_t) max_r, /* cast: 12-bit fits int32_t */
            (int32_t) (max_r - min_r) /* cast: 12-bit fits int32_t */);
#endif
}

#else /* 0 == APP_TEST */

/**
 * test_line_run
 * @brief   No-op when built without TEST=1.
 * @return  none
 * @context usermain
 */
void test_line_run(void)
{
    /* empty in production build */
}

#endif /* APP_TEST */

/*** end of file ***/
