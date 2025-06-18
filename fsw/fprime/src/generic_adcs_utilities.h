/*******************************************************************************
** Purpose:
**   This file has utility functions used by ADCS.
**
*******************************************************************************/
#ifndef _GENERIC_ADCS_UTILITIES_H_
#define _GENERIC_ADCS_UTILITIES_H_

double arccos(double x);
double VoV(double A[3], double B[3]);
void   VxV(double A[3], double B[3], double C[3]);
void   SxV(double S, double V[3], double W[3]);
double MAGV(double v[3]);
void   UNITV(double V[3]);
double CopyUnitV(double V[3], double W[3]);
void   QxQ(const double A[4], const double B[4], double C[4]);
void   QxQT(const double A[4], const double B[4], double C[4]);
void   QxV(const double QAB[4], const double Vb[3], double Va[3]);
void   QTxV(const double QAB[4], const double Va[3], double Vb[3]);
void   UNITQ(double Q[4]);
void   RECTIFYQ(double Q[4]);
double Limit(double x, double min, double max);

#endif
