/*
 *  mock_kernel.c
 *  Host-only event flags with micro T-Kernel semantics, for the isolated
 *  tests of safety.c and fault.c. Outside BARR-C scope.
 *
 *  tk_set_flg ORs the pattern in; tk_clr_flg ANDs (keeps only the bits
 *  given), which is why safety_clear()/fault_clear() pass ~mask.
 */

#include "tk/tkernel.h"

#define MOCK_FLG_MAX  (8)

static UINT g_flg_ptn[MOCK_FLG_MAX + 1];
static int  g_flg_used = 0;

ID tk_cre_flg(T_CFLG const * pk_cflg)
{
    ID id = E_PAR;

    if ((0 != pk_cflg) && (g_flg_used < MOCK_FLG_MAX))
    {
        g_flg_used++;
        g_flg_ptn[g_flg_used] = pk_cflg->iflgptn;
        id = g_flg_used;
    }
    return id;
}

static int mock_flg_valid(ID flgid)
{
    return (flgid >= 1) && (flgid <= g_flg_used);
}

ER tk_set_flg(ID flgid, UINT setptn)
{
    ER ercd = E_ID;

    if (mock_flg_valid(flgid))
    {
        g_flg_ptn[flgid] |= setptn;
        ercd = E_OK;
    }
    return ercd;
}

ER tk_clr_flg(ID flgid, UINT clrptn)
{
    ER ercd = E_ID;

    if (mock_flg_valid(flgid))
    {
        g_flg_ptn[flgid] &= clrptn;   /* kernel semantics: AND */
        ercd = E_OK;
    }
    return ercd;
}

ER tk_ref_flg(ID flgid, T_RFLG * pk_rflg)
{
    ER ercd = E_ID;

    if (mock_flg_valid(flgid) && (0 != pk_rflg))
    {
        pk_rflg->exinf  = 0;
        pk_rflg->wtsk   = 0;
        pk_rflg->flgptn = g_flg_ptn[flgid];
        ercd = E_OK;
    }
    return ercd;
}
