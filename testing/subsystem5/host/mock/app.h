/* Host-simulation stand-in for app_program/app.h: the few kernel types,
   constants and calls Buddy 5 uses. Host-only, outside BARR-C scope. */
#ifndef APP_H
#define APP_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef int ID; typedef int ER; typedef int INT; typedef unsigned UINT;
typedef void (*FP)(void); typedef int PRI; typedef int TMO; typedef int SZ;
typedef unsigned ATR; typedef unsigned RELTIM;
#define E_OK 0
#define E_TMOUT (-50)
#define E_PAR (-17)
#define E_LIMIT (-34)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_TFIFO 0
#define TA_WMUL 8
#define TA_USERBUF 0x20
#define TA_INHERIT 2
#define APP_TEST 0
#define APP_PROBE 0
#define APP_PRI_MISSION 16
#define APP_PRI_OBSTACLE 15
#define APP_SYS_STACK_BYTES 128u
#define APP_STACK_MISSION_BYTES (2048u + 128u)
#define APP_STACK_OBSTACLE_BYTES (1024u + 128u)
typedef struct { void *exinf; ATR mbfatr; SZ bufsz; INT maxmsz; void *bufptr; } T_CMBF;
typedef struct { void *exinf; ID wtsk; ID stsk; INT msgsz; SZ frbufsz; INT maxmsz; } T_RMBF;
typedef struct { void *exinf; ATR flgatr; UINT iflgptn; } T_CFLG;
typedef struct { void *exinf; ID wtsk; UINT flgptn; } T_RFLG;
typedef struct { void *exinf; ATR mtxatr; } T_CMTX;
ID  tk_cre_mbf(T_CMBF const *p); ER tk_snd_mbf(ID id, void const *m, INT sz, TMO t);
INT tk_rcv_mbf(ID id, void *m, TMO t); ER tk_ref_mbf(ID id, T_RMBF *r);
ID  tk_cre_flg(T_CFLG const *p); ER tk_set_flg(ID id, UINT p); ER tk_clr_flg(ID id, UINT p);
ER  tk_ref_flg(ID id, T_RFLG *r);
ID  tk_cre_mtx(T_CMTX const *p); ER tk_loc_mtx(ID id, TMO t); ER tk_unl_mtx(ID id);
ID  app_start_task(FP e, PRI p, uint64_t *s, uint32_t b, char const *n);
void app_check(ER e, char const *w);
void app_log(char const *f, int32_t a, int32_t b, int32_t c);
#endif
