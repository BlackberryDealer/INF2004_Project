/**
 * @file    test_mission.h
 * @brief   On-target tests for the mission module (TEST=1 builds).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    test
 */

#ifndef TEST_MISSION_H
#define TEST_MISSION_H

#include <stdint.h>
#include "../app.h"

extern void test_mission_run(void);   /* called by usermain in TEST=1 builds */

#endif /* TEST_MISSION_H */

/*** end of file ***/
