/* Host stand-in for app_program/app.h: the few kernel types, constants
   and calls subsystem 1's kernel-side files use, so mqtt_bridge.c can be
   tested on the laptop. Host-only, outside BARR-C scope (D7).          */
#ifndef APP_H
#define APP_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef int ID;
typedef int ER;
typedef int INT;
typedef unsigned UINT;
typedef int TMO;
typedef int PRI;
typedef int SZ;
typedef void (*FP)(void);
#define E_OK      0
#define E_TMOUT   (-50)
#define TMO_POL   0
#define TMO_FEVR  (-1)
#define APP_TEST  0
#define APP_PROBE 0
INT tk_rcv_mbf(ID id, void * p_msg, TMO tmout);
ER  tk_snd_mbf(ID id, void const * p_msg, INT size, TMO tmout);
void app_log(char const * p_format, int32_t arg_a, int32_t arg_b,
             int32_t arg_c);
#endif
