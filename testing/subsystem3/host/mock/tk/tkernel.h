/*
 *  mock/tk/tkernel.h
 *  Minimal micro T-Kernel types and prototypes for host simulation of
 *  subsystem 3's task modules (line.c, barcode.c).
 */

#ifndef MOCK_TKERNEL_H
#define MOCK_TKERNEL_H

#include <stdint.h>

typedef int32_t  ID;
typedef int32_t  ER;
typedef int32_t  INT;
typedef int32_t  TMO;
typedef int32_t  PRI;
typedef uint32_t UINT;
typedef uint32_t UW;
typedef void (*FP)(void);

#define E_OK      (0)
#define TMO_FEVR  (-1)
#define TA_TFIFO  (0u)
#define GPIO_IN   (0xD0000004u)

typedef struct
{
    void *   exinf;
    UINT     sematr;
    INT      isemcnt;
    INT      maxsem;
} T_CSEM;

extern ID tk_cre_sem(T_CSEM const * pk_csem);
extern ER tk_sig_sem(ID semid, INT cnt);
extern ER tk_wai_sem(ID semid, INT cnt, TMO tmout);
extern ER tk_slp_tsk(TMO tmout);
extern ID tk_get_tid(void);
extern UW in_w(UW port);

#endif /* MOCK_TKERNEL_H */
