/*******************************************************************************
** Purpose:
**   This file has utility functions used by ADCS.
**
*******************************************************************************/
#ifndef _GENERIC_ADCS_UTILITIES_H_
#define _GENERIC_ADCS_UTILITIES_H_

#ifndef D2R
   #define D2R (1.74532925199433E-2)
#endif
#ifndef EPS32
   #define EPS32 (1.0E-32)
#endif

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

void skewM(double A[3], double B[3][3]);
void M66xM66(double A[6][6], double B[6][6], double C[6][6]);
void M66xM66T(double A[6][6], double B[6][6], double C[6][6]);
void M66xM36T(double A[6][6], double B[3][6], double C[6][3]);
void M36TxM33(double A[3][6], double B[3][3], double C[6][3]);
void M36xM63(double A[3][6], double B[6][3], double C[3][3]);
void M66xM63(double A[6][6], double B[6][3], double C[6][3]);
void M63xM36(double A[6][3], double B[3][6], double C[6][6]);
void M36xV6(double A[3][6], double B[6], double C[3]);
void M63xV3(double A[6][3], double B[3], double C[6]);
void M44XV4(double M[4][4], double v[4], double output[4]);

void MxM(double A[3][3], double B[3][3], double C[3][3]);
void MxV (double M[3][3], double V[3], double W[3]);
void MINV3(double A[3][3], double B[3][3]);

void Q2C(double Q[4], double C[3][3]);
void QW2QDOT(double Q[4],double W[3],double QDOT[4]);

#endif
