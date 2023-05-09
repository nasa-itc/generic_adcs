/*******************************************************************************
** Purpose:
**   This file has utility functions used by ADCS.
**
*******************************************************************************/
#ifndef _GENERIC_ADCS_UTILITIES_H_
#define _GENERIC_ADCS_UTILITIES_H_

double MAGV(double v[3]);
void QxV(double QAB[4],double Vb[3],double Va[3]);
void QTxV(double QAB[4],double Va[3],double Vb[3]);
void UNITV(double V[3]);

#endif
