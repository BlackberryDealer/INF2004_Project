/*
 *  mock/tk/tkernel.h
 *  Host-only stand-in for the micro T-Kernel header, so subsystem 2's
 *  kernel-side modules (safety.c, fault.c, motion_task.c) can be compiled
 *  and tested on a laptop against isolated objects. Outside BARR-C scope.
 *
 *  Only what those modules use. Values match include/tk/ (typedef.h,
 *  errno.h, syscall.h); W/UW are 32-bit there, so int32_t/uint32_t here.
 *  Event flags are implemented in ../../mock_kernel.c with the kernel's
 *  semantics: tk_set_flg ORs, tk_clr_flg ANDs (keeps the given bits).
 */

#ifndef MOCK_TK_TKERNEL_H
#define MOCK_TK_TKERNEL_H

#include <stdint.h>

typedef int         INT;
typedef unsigned    UINT;
typedef int32_t     W;
typedef uint32_t    UW;
typedef INT         ID;
typedef INT         ER;
typedef INT         PRI;
typedef W           SZ;
typedef W           TMO;
typedef UW          ATR;
typedef UW          RELTIM;
typedef void        (*FP)(void);

#define E_OK        (0)
#define E_PAR       (-17)
#define E_ID        (-18)
#define E_TMOUT     (-50)
#define TMO_POL     (0)
#define TMO_FEVR    (-1)
#define TSK_SELF    (0)
#define TA_TFIFO    (0x00000000u)
#define TA_WMUL     (0x00000008u)

typedef struct
{
    void * exinf;
    ATR    flgatr;
    UINT   iflgptn;
} T_CFLG;

typedef struct
{
    void * exinf;
    ID     wtsk;
    UINT   flgptn;
} T_RFLG;

ID tk_cre_flg(T_CFLG const * pk_cflg);
ER tk_set_flg(ID flgid, UINT setptn);
ER tk_clr_flg(ID flgid, UINT clrptn);
ER tk_ref_flg(ID flgid, T_RFLG * pk_rflg);
ER tk_slp_tsk(TMO tmout);
ID tk_get_tid(void);

#endif /* MOCK_TK_TKERNEL_H */
