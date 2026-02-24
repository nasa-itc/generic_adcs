/*******************************************************************************
** Purpose:
**   This file has utility functions used by ADCS.
**
*******************************************************************************/
#ifndef _GENERIC_ADCS_UTILITIES_H_
#define _GENERIC_ADCS_UTILITIES_H_

#ifndef EPS16
   #define EPS16 (1.0E-16)
#endif
#ifndef EPS32
   #define EPS32 (1.0E-32)
#endif
#ifndef TWOPI
   #define TWOPI (6.283185307179586)
#endif
#ifndef D2R
   #define D2R (1.74532925199433E-2)
#endif
#ifndef RE
   #define RE (6378137.0)
#endif
#ifndef NANO2TSLA
   #define NANO2TSLA (1.0E-9)  /* nano to Tesla */
#endif
#ifndef M2KM
   #define M2KM (0.001)
#endif

typedef struct 
{
  float      Comp[3];
} Vector3f;

typedef struct
{
  float      Comp[3][3];
} Matrix3x3f;

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

void SimpRot(double AXIS[3], double THETA, double C[3][3]);

void Q2C(double Q[4], double C[3][3]);
void QW2QDOT(double Q[4],double W[3],double QDOT[4]);

void Matrix3x3f_MultVec (Vector3f *Result, const Matrix3x3f *Left, const Vector3f *Right);

void HiFiEarthPrecNute(double JD, double C_TETE_J2000[3][3]);

double GpsTime_TO_JD(long GpsRollover, long GpsWeek, double GpsSecond);
double JD_TO_GMST(double JD);
double GpsDateToGpsTime(long GpsRollover, long GpsWeek, double GpsSecond);
void TimeToDate(double Time, long *Year, long *Month, long *Day,
                long *Hour, long *Minute, double *Second, double LSB);
#endif
