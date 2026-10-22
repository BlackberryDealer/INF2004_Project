/**
 * @file    test_barcode.c
 * @brief   Phase 1 on-target barcode pin and configuration sanity checks.
 * @owner   Buddy 3 (line, barcode)
 * @kind    test
 * @reqs    FR2, NFR5
 */

#include "test_barcode.h"
#include "../app.h"

#if (0 != APP_TEST)

#include "../test_report.h"
#include "../board_pins.h"
#include "cfg_barcode.h"
#include "hardware/gpio.h"

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * test_barcode_run
 * @brief   Runs Phase 1 on-target barcode pin and polarity checks (S3-I07).
 * @return  none
 * @context usermain (APP_PRI_TEST)
 */
void test_barcode_run(void)
{
    bool     b_pin_level = false;
    uint32_t is_black    = 0u;

    TEST_INIT();

    gpio_init(PIN_BAR_DO);
    gpio_set_dir(PIN_BAR_DO, GPIO_IN);
    b_pin_level = gpio_get(PIN_BAR_DO);

    if (BAR_DO_BLACK_IS_HIGH == b_pin_level)
    {
        is_black = 1u;
    }
    else
    {
        is_black = 0u;
    }

    app_log("S3-I07 PIN_BAR_DO raw=%d black=%d\n",
            (int32_t) b_pin_level, /* cast: bool 0/1 fits int32_t */
            (int32_t) is_black,    /* cast: 0/1 fits int32_t */
            0);

    TEST_CHECK("S3-I07-PIN", (7u == PIN_BAR_DO), "Barcode DO is on GP7");
    TEST_SUMMARY("barcode");
}

/* ---- private functions ------------------------------------------------ */

#else /* 0 == APP_TEST */

/**
 * test_barcode_run
 * @brief   No-op when built without TEST=1.
 * @return  none
 * @context usermain
 */
void test_barcode_run(void)
{
    /* empty in production build */
}

#endif /* APP_TEST */

/*** end of file ***/
