/*******************************************************************************
** Purpose:
**   This file implements utility functions used by ADCS.
**
*******************************************************************************/

#include <math.h>
#include <stdio.h>
#include "generic_adcs_utilities.h"

/**********************************************************************/
/*  Vector Dot Product                                                */
double VoV(double A[3], double B[3])
{
    return (A[0] * B[0] + A[1] * B[1] + A[2] * B[2]);
}
/**********************************************************************/
/*  Vector Cross Product                                              */
void VxV(double A[3], double B[3], double C[3])
{
    C[0] = A[1] * B[2] - A[2] * B[1];
    C[1] = A[2] * B[0] - A[0] * B[2];
    C[2] = A[0] * B[1] - A[1] * B[0];
}
// Magnitude of v vector
double MAGV(double v[3])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**********************************************************************/
/*  Quaternion product                                                */
void QxQ(const double A[4], const double B[4], double C[4])
{
    C[0] = A[3] * B[0] + A[2] * B[1] - A[1] * B[2] + A[0] * B[3];
    C[1] = -A[2] * B[0] + A[3] * B[1] + A[0] * B[2] + A[1] * B[3];
    C[2] = A[1] * B[0] - A[0] * B[1] + A[3] * B[2] + A[2] * B[3];
    C[3] = -A[0] * B[0] - A[1] * B[1] - A[2] * B[2] + A[3] * B[3];
}

/**********************************************************************/
/* Product of a Quaternion (A) with the Complement of a Quaternion (B)*/
void QxQT(const double A[4], const double B[4], double C[4])
{
    C[0] = -A[3] * B[0] - A[2] * B[1] + A[1] * B[2] + A[0] * B[3];
    C[1] = A[2] * B[0] - A[3] * B[1] - A[0] * B[2] + A[1] * B[3];
    C[2] = -A[1] * B[0] + A[0] * B[1] - A[3] * B[2] + A[2] * B[3];
    C[3] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2] + A[3] * B[3];
}

/**********************************************************************/
/* Find components of V in A, given components of V in B, and qab     */
void QxV(const double QAB[4], const double Vb[3], double Va[3])
{
    double qq[4][4];
    long   i, j;

    for (i = 0; i < 4; i++)
    {
        for (j = i; j < 4; j++)
            qq[i][j] = QAB[i] * QAB[j];
    }

    Va[0] = (qq[0][0] - qq[1][1] - qq[2][2] + qq[3][3]) * Vb[0] +
            2.0 * ((qq[0][1] + qq[2][3]) * Vb[1] + (qq[0][2] - qq[1][3]) * Vb[2]);
    Va[1] = (-qq[0][0] + qq[1][1] - qq[2][2] + qq[3][3]) * Vb[1] +
            2.0 * ((qq[1][2] + qq[0][3]) * Vb[2] + (qq[0][1] - qq[2][3]) * Vb[0]);
    Va[2] = (-qq[0][0] - qq[1][1] + qq[2][2] + qq[3][3]) * Vb[2] +
            2.0 * ((qq[0][2] + qq[1][3]) * Vb[0] + (qq[1][2] - qq[0][3]) * Vb[1]);
}
/**********************************************************************/
/* Find components of V in B, given components of V in A, and qab     */
void QTxV(const double QAB[4], const double Va[3], double Vb[3])
{
    double qq[4][4];
    long   i, j;

    for (i = 0; i < 4; i++)
    {
        for (j = i; j < 4; j++)
            qq[i][j] = QAB[i] * QAB[j];
    }

    Vb[0] = (qq[0][0] - qq[1][1] - qq[2][2] + qq[3][3]) * Va[0] +
            2.0 * ((qq[0][1] - qq[2][3]) * Va[1] + (qq[0][2] + qq[1][3]) * Va[2]);
    Vb[1] = (-qq[0][0] + qq[1][1] - qq[2][2] + qq[3][3]) * Va[1] +
            2.0 * ((qq[1][2] - qq[0][3]) * Va[2] + (qq[0][1] + qq[2][3]) * Va[0]);
    Vb[2] = (-qq[0][0] - qq[1][1] + qq[2][2] + qq[3][3]) * Va[2] +
            2.0 * ((qq[0][2] - qq[1][3]) * Va[0] + (qq[1][2] + qq[0][3]) * Va[1]);
}
#ifndef EPS16
#define EPS16 (1.0E-16)
#endif
/**********************************************************************/
/*  Normalize a quaternion                                            */
void UNITQ(double Q[4])
{
    double A;

    A = sqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
    if (A < EPS16)
    {
        Q[0] = 0.0;
        Q[1] = 0.0;
        Q[2] = 0.0;
        Q[3] = 1.0;
#ifndef ACS_IN_FSW
        printf("Divide by zero in UNITQ (Line %d of mathkit.c).  You'll want to fix that.\n", __LINE__);
#endif
    }
    else
    {
        Q[0] /= A;
        Q[1] /= A;
        Q[2] /= A;
        Q[3] /= A;
    }
}
/**********************************************************************/
/*  Rectify a quaternion, forcing q[3] to be positive                 */
void RECTIFYQ(double Q[4])
{
    if (Q[3] < 0.0)
    {
        Q[0] = -Q[0];
        Q[1] = -Q[1];
        Q[2] = -Q[2];
        Q[3] = -Q[3];
    }
}
/**********************************************************************/
/*  Normalize a 3-vector if it is non-zero.                           */
void UNITV(double V[3])
{
    double A;

    A = sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
    if (A > 0.0)
    {
        V[0] /= A;
        V[1] /= A;
        V[2] /= A;
    }
}
double arccos(double x)
{
    if (x > 1.0)
        x = 1.0;
    if (x < -1.0)
        x = -1.0;

    return (acos(x));
}
/**********************************************************************/
double Limit(double x, double min, double max)
{
    return (x < min ? min : (x > max ? max : x));
}
/**********************************************************************/
/*  Scalar times 3x1 Vector                                           */
void SxV(double S, double V[3], double W[3])
{
    W[0] = S * V[0];
    W[1] = S * V[1];
    W[2] = S * V[2];
}
/**********************************************************************/
/*  Copy and normalize a 3-vector.  Return its magnitude              */
double CopyUnitV(double V[3], double W[3])
{
    double A;

    A = sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
    if (A > 0.0)
    {
        W[0] = V[0] / A;
        W[1] = V[1] / A;
        W[2] = V[2] / A;
    }
    else
    {
        W[0] = 0.0;
        W[1] = 0.0;
        W[2] = 0.0;
    }
    return (A);
}

void skewM(double A[3], double B[3][3])
{

   B[0][0] =   0.0;
   B[1][1] =   0.0;
   B[2][2] =   0.0;
   B[2][1] =  A[0];
   B[0][2] =  A[1];
   B[1][0] =  A[2];
   B[1][2] = -A[0];
   B[2][0] = -A[1];
   B[0][1] = -A[2];

}

void M66xM66(double A[6][6], double B[6][6], double C[6][6])
{

   long i,j,k;

   for(i = 0; i < 6; i++) {
      for(j = 0; j < 6; j++) {
         C[i][j] = 0.0;
         for(k = 0; k < 6; k++) {
            C[i][j] += A[i][k]*B[k][j];
         }
      }
   }

}

void M66xM66T(double A[6][6], double B[6][6], double C[6][6])
{

   long i,j,k;

   for(i = 0; i < 6; i++) {
      for(j = 0; j < 6; j++) {
         C[i][j] = 0.0;
         for(k = 0; k < 6; k++) {
            C[i][j] += A[i][k]*B[j][k];
         }
      }
   }

}

void M66xM36T(double A[6][6], double B[3][6], double C[6][3])
{

   long i,j,k;

   for(i = 0; i < 6; i++) {
      for(j = 0; j < 3; j++) {
         C[i][j] = 0.0;
         for(k = 0; k < 6; k++) {
            C[i][j] += A[i][k]*B[j][k];
         }
      }
   }

}

void M36TxM33(double A[3][6], double B[3][3], double C[6][3])
{

   long i,j,k;

   for(i = 0; i < 6; i++) {
      for(j = 0; j < 3; j++) {
         C[i][j] = 0.0;
         for(k = 0; k < 3; k++) {
            C[i][j] += A[k][i]*B[k][j];
         }
      }
   }

}

void M36xM63(double A[3][6], double B[6][3], double C[3][3])
{

   long i,j,k;

   for(i = 0; i < 3; i++) {
      for(j = 0; j < 3; j++) {
         C[i][j] = 0.0;
         for(k = 0; k < 6; k++) {
            C[i][j] += A[i][k]*B[k][j];
         }
      }
   }

}

void M66xM63(double A[6][6], double B[6][3], double C[6][3])
{

   long i,j,k;

   for(i = 0; i < 6; i++) {
      for(j = 0; j < 3; j++) {
         C[i][j] = 0.0;
         for(k = 0; k < 6; k++) {
            C[i][j] += A[i][k]*B[k][j];
         }
      }
   }

}

void M63xM36(double A[6][3], double B[3][6], double C[6][6])
{

   long i,j,k;

   for(i = 0; i < 6; i++) {
      for(j = 0; j < 6; j++) {
         C[i][j] = 0.0;
         for(k = 0; k < 3; k++) {
            C[i][j] += A[i][k]*B[k][j];
         }
      }
   }

}

void M36xV6(double A[3][6], double B[6], double C[3])
{

   long i,k;

   for(i = 0; i < 3; i++) {
      C[i] = 0.0;
      for(k = 0; k < 6; k++) {
         C[i] += A[i][k]*B[k];
      }
   }

}

void M63xV3(double A[6][3], double B[3], double C[6])
{

   long i,k;

   for(i = 0; i < 6; i++) {
      C[i] = 0.0;
      for(k = 0; k < 3; k++) {
         C[i] += A[i][k]*B[k];
      }
   }

}

void M44XV4(double M[4][4], double v[4], double output[4]) {
   output[0] = M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2] + M[0][3]*v[3];
   output[1] = M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2] + M[1][3]*v[3];
   output[2] = M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2] + M[2][3]*v[3];
   output[3] = M[3][0]*v[0] + M[3][1]*v[1] + M[3][2]*v[2] + M[3][3]*v[3];
}

/**********************************************************************/
/*   3x3 Matrix Product                                               */
void MxM(double A[3][3], double B[3][3], double C[3][3])
{

      C[0][0]=A[0][0]*B[0][0]+A[0][1]*B[1][0]+A[0][2]*B[2][0];
      C[0][1]=A[0][0]*B[0][1]+A[0][1]*B[1][1]+A[0][2]*B[2][1];
      C[0][2]=A[0][0]*B[0][2]+A[0][1]*B[1][2]+A[0][2]*B[2][2];
      C[1][0]=A[1][0]*B[0][0]+A[1][1]*B[1][0]+A[1][2]*B[2][0];
      C[1][1]=A[1][0]*B[0][1]+A[1][1]*B[1][1]+A[1][2]*B[2][1];
      C[1][2]=A[1][0]*B[0][2]+A[1][1]*B[1][2]+A[1][2]*B[2][2];
      C[2][0]=A[2][0]*B[0][0]+A[2][1]*B[1][0]+A[2][2]*B[2][0];
      C[2][1]=A[2][0]*B[0][1]+A[2][1]*B[1][1]+A[2][2]*B[2][1];
      C[2][2]=A[2][0]*B[0][2]+A[2][1]*B[1][2]+A[2][2]*B[2][2];
}
/**********************************************************************/
/*  3x3 Matrix times 3x1 Vector                                       */
void MxV (double M[3][3], double V[3], double W[3])
{
      W[0]=V[0]*M[0][0]+V[1]*M[0][1]+V[2]*M[0][2];
      W[1]=V[0]*M[1][0]+V[1]*M[1][1]+V[2]*M[1][2];
      W[2]=V[0]*M[2][0]+V[1]*M[2][1]+V[2]*M[2][2];
}
/******************************************************************************/
/*  Inverse of a 3x3 Matrix                                                   */
void MINV3(double A[3][3], double B[3][3])
{
      double DET;

      DET=A[0][0]*A[1][1]*A[2][2]+A[0][1]*A[1][2]*A[2][0]
         +A[0][2]*A[1][0]*A[2][1]-A[2][0]*A[1][1]*A[0][2]
         -A[2][1]*A[1][2]*A[0][0]-A[2][2]*A[1][0]*A[0][1];

      if (DET < EPS32) {
         B[0][0]=0.0;
         B[0][1]=0.0;
         B[0][2]=0.0;
         B[1][0]=0.0;
         B[1][1]=0.0;
         B[1][2]=0.0;
         B[2][0]=0.0;
         B[2][1]=0.0;
         B[2][2]=0.0;
         #ifndef ACS_IN_FSW
         printf("DET = %le \n", DET);
         printf("Attempted inversion of singular matrix in MINV3.  Bailing out.\n");
         #endif
      }
      else {
         B[0][0]=(A[1][1]*A[2][2]-A[2][1]*A[1][2])/DET;
         B[0][1]=(A[2][1]*A[0][2]-A[0][1]*A[2][2])/DET;
         B[0][2]=(A[0][1]*A[1][2]-A[1][1]*A[0][2])/DET;
         B[1][0]=(A[2][0]*A[1][2]-A[1][0]*A[2][2])/DET;
         B[1][1]=(A[0][0]*A[2][2]-A[2][0]*A[0][2])/DET;
         B[1][2]=(A[1][0]*A[0][2]-A[0][0]*A[1][2])/DET;
         B[2][0]=(A[1][0]*A[2][1]-A[2][0]*A[1][1])/DET;
         B[2][1]=(A[2][0]*A[0][1]-A[0][0]*A[2][1])/DET;
         B[2][2]=(A[0][0]*A[1][1]-A[1][0]*A[0][1])/DET;
      }
}

/**********************************************************************/
/*  Convert quaternion to direction cosine matrix                     */

void Q2C(double Q[4], double C[3][3])
{
      double TwoQ00,TwoQ11,TwoQ22;
      double TwoQ01,TwoQ02,TwoQ03;
      double TwoQ12,TwoQ13,TwoQ23;

      TwoQ00 = 2.0*Q[0]*Q[0];
      TwoQ11 = 2.0*Q[1]*Q[1];
      TwoQ22 = 2.0*Q[2]*Q[2];
      TwoQ01 = 2.0*Q[0]*Q[1];
      TwoQ02 = 2.0*Q[0]*Q[2];
      TwoQ03 = 2.0*Q[0]*Q[3];
      TwoQ12 = 2.0*Q[1]*Q[2];
      TwoQ13 = 2.0*Q[1]*Q[3];
      TwoQ23 = 2.0*Q[2]*Q[3];

      C[0][0] = 1.0-(TwoQ11+TwoQ22);
      C[0][1] = TwoQ01+TwoQ23;
      C[0][2] = TwoQ02-TwoQ13;
      C[1][0] = TwoQ01-TwoQ23;
      C[1][1] = 1.0-(TwoQ22+TwoQ00);
      C[1][2] = TwoQ12+TwoQ03;
      C[2][0] = TwoQ02+TwoQ13;
      C[2][1] = TwoQ12-TwoQ03;
      C[2][2] = 1.0-(TwoQ00+TwoQ11);
}
/**********************************************************************/
/*  Given body rates and quaternion, find qdot.  Ref Kane, 1.13       */
void QW2QDOT(double Q[4],double W[3],double QDOT[4])
{

      QDOT[0] = 0.5*( W[0]*Q[3]-W[1]*Q[2]+W[2]*Q[1]);
      QDOT[1] = 0.5*( W[0]*Q[2]+W[1]*Q[3]-W[2]*Q[0]);
      QDOT[2] = 0.5*(-W[0]*Q[1]+W[1]*Q[0]+W[2]*Q[3]);
      QDOT[3] = 0.5*(-W[0]*Q[0]-W[1]*Q[1]-W[2]*Q[2]);

}
