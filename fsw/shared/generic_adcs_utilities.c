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

    B[0][0] = 0.0;
    B[1][1] = 0.0;
    B[2][2] = 0.0;
    B[2][1] = A[0];
    B[0][2] = A[1];
    B[1][0] = A[2];
    B[1][2] = -A[0];
    B[2][0] = -A[1];
    B[0][1] = -A[2];
}

void M66xM66(double A[6][6], double B[6][6], double C[6][6])
{

    long i, j, k;

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            C[i][j] = 0.0;
            for (k = 0; k < 6; k++)
            {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void M66xM66T(double A[6][6], double B[6][6], double C[6][6])
{

    long i, j, k;

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            C[i][j] = 0.0;
            for (k = 0; k < 6; k++)
            {
                C[i][j] += A[i][k] * B[j][k];
            }
        }
    }
}

void M66xM36T(double A[6][6], double B[3][6], double C[6][3])
{

    long i, j, k;

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 3; j++)
        {
            C[i][j] = 0.0;
            for (k = 0; k < 6; k++)
            {
                C[i][j] += A[i][k] * B[j][k];
            }
        }
    }
}

void M36TxM33(double A[3][6], double B[3][3], double C[6][3])
{

    long i, j, k;

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 3; j++)
        {
            C[i][j] = 0.0;
            for (k = 0; k < 3; k++)
            {
                C[i][j] += A[k][i] * B[k][j];
            }
        }
    }
}

void M36xM63(double A[3][6], double B[6][3], double C[3][3])
{

    long i, j, k;

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            C[i][j] = 0.0;
            for (k = 0; k < 6; k++)
            {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void M66xM63(double A[6][6], double B[6][3], double C[6][3])
{

    long i, j, k;

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 3; j++)
        {
            C[i][j] = 0.0;
            for (k = 0; k < 6; k++)
            {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void M63xM36(double A[6][3], double B[3][6], double C[6][6])
{

    long i, j, k;

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            C[i][j] = 0.0;
            for (k = 0; k < 3; k++)
            {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void M36xV6(double A[3][6], double B[6], double C[3])
{

    long i, k;

    for (i = 0; i < 3; i++)
    {
        C[i] = 0.0;
        for (k = 0; k < 6; k++)
        {
            C[i] += A[i][k] * B[k];
        }
    }
}

void M63xV3(double A[6][3], double B[3], double C[6])
{

    long i, k;

    for (i = 0; i < 6; i++)
    {
        C[i] = 0.0;
        for (k = 0; k < 3; k++)
        {
            C[i] += A[i][k] * B[k];
        }
    }
}

void M44XV4(double M[4][4], double v[4], double output[4])
{
    output[0] = M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2] + M[0][3] * v[3];
    output[1] = M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2] + M[1][3] * v[3];
    output[2] = M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2] + M[2][3] * v[3];
    output[3] = M[3][0] * v[0] + M[3][1] * v[1] + M[3][2] * v[2] + M[3][3] * v[3];
}

/**********************************************************************/
/*   3x3 Matrix Product                                               */
void MxM(double A[3][3], double B[3][3], double C[3][3])
{

    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
    C[0][2] = A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
    C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
    C[1][2] = A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2];
    C[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0];
    C[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1];
    C[2][2] = A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2];
}
/**********************************************************************/
/*  3x3 Matrix times 3x1 Vector                                       */
void MxV(double M[3][3], double V[3], double W[3])
{
    W[0] = V[0] * M[0][0] + V[1] * M[0][1] + V[2] * M[0][2];
    W[1] = V[0] * M[1][0] + V[1] * M[1][1] + V[2] * M[1][2];
    W[2] = V[0] * M[2][0] + V[1] * M[2][1] + V[2] * M[2][2];
}
/******************************************************************************/
/*  Inverse of a 3x3 Matrix                                                   */
void MINV3(double A[3][3], double B[3][3])
{
    double DET;

    DET = A[0][0] * A[1][1] * A[2][2] + A[0][1] * A[1][2] * A[2][0] + A[0][2] * A[1][0] * A[2][1] -
          A[2][0] * A[1][1] * A[0][2] - A[2][1] * A[1][2] * A[0][0] - A[2][2] * A[1][0] * A[0][1];

    if (DET < EPS32)
    {
        B[0][0] = 0.0;
        B[0][1] = 0.0;
        B[0][2] = 0.0;
        B[1][0] = 0.0;
        B[1][1] = 0.0;
        B[1][2] = 0.0;
        B[2][0] = 0.0;
        B[2][1] = 0.0;
        B[2][2] = 0.0;
#ifndef ACS_IN_FSW
        printf("DET = %le \n", DET);
        printf("Attempted inversion of singular matrix in MINV3.  Bailing out.\n");
#endif
    }
    else
    {
        B[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / DET;
        B[0][1] = (A[2][1] * A[0][2] - A[0][1] * A[2][2]) / DET;
        B[0][2] = (A[0][1] * A[1][2] - A[1][1] * A[0][2]) / DET;
        B[1][0] = (A[2][0] * A[1][2] - A[1][0] * A[2][2]) / DET;
        B[1][1] = (A[0][0] * A[2][2] - A[2][0] * A[0][2]) / DET;
        B[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) / DET;
        B[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / DET;
        B[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) / DET;
        B[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / DET;
    }
}

/**********************************************************************/
/* Compute direction cosine matrix corresponding to a                 */
/* simple rotation of THETA radians about a unit vector               */
/* parallel to AXIS                                                   */

void SimpRot(double AXIS[3], double THETA, double C[3][3])
{
    double CTH, STH, CTH1, AX[3];

    CTH  = cos(THETA);
    STH  = sin(THETA);
    CTH1 = 1.0 - CTH;
    CopyUnitV(AXIS, AX);

    C[0][0] = CTH + AX[0] * AX[0] * CTH1;
    C[1][0] = -AX[2] * STH + AX[0] * AX[1] * CTH1;
    C[2][0] = AX[1] * STH + AX[2] * AX[0] * CTH1;
    C[0][1] = AX[2] * STH + AX[0] * AX[1] * CTH1;
    C[1][1] = CTH + AX[1] * AX[1] * CTH1;
    C[2][1] = -AX[0] * STH + AX[1] * AX[2] * CTH1;
    C[0][2] = -AX[1] * STH + AX[2] * AX[0] * CTH1;
    C[1][2] = AX[0] * STH + AX[1] * AX[2] * CTH1;
    C[2][2] = CTH + AX[2] * AX[2] * CTH1;
}

/**********************************************************************/
/*  Convert quaternion to direction cosine matrix                     */

void Q2C(double Q[4], double C[3][3])
{
    double TwoQ00, TwoQ11, TwoQ22;
    double TwoQ01, TwoQ02, TwoQ03;
    double TwoQ12, TwoQ13, TwoQ23;

    TwoQ00 = 2.0 * Q[0] * Q[0];
    TwoQ11 = 2.0 * Q[1] * Q[1];
    TwoQ22 = 2.0 * Q[2] * Q[2];
    TwoQ01 = 2.0 * Q[0] * Q[1];
    TwoQ02 = 2.0 * Q[0] * Q[2];
    TwoQ03 = 2.0 * Q[0] * Q[3];
    TwoQ12 = 2.0 * Q[1] * Q[2];
    TwoQ13 = 2.0 * Q[1] * Q[3];
    TwoQ23 = 2.0 * Q[2] * Q[3];

    C[0][0] = 1.0 - (TwoQ11 + TwoQ22);
    C[0][1] = TwoQ01 + TwoQ23;
    C[0][2] = TwoQ02 - TwoQ13;
    C[1][0] = TwoQ01 - TwoQ23;
    C[1][1] = 1.0 - (TwoQ22 + TwoQ00);
    C[1][2] = TwoQ12 + TwoQ03;
    C[2][0] = TwoQ02 + TwoQ13;
    C[2][1] = TwoQ12 - TwoQ03;
    C[2][2] = 1.0 - (TwoQ00 + TwoQ11);
}
/**********************************************************************/
/*  Given body rates and quaternion, find qdot.  Ref Kane, 1.13       */
void QW2QDOT(double Q[4], double W[3], double QDOT[4])
{

    QDOT[0] = 0.5 * (W[0] * Q[3] - W[1] * Q[2] + W[2] * Q[1]);
    QDOT[1] = 0.5 * (W[0] * Q[2] + W[1] * Q[3] - W[2] * Q[0]);
    QDOT[2] = 0.5 * (-W[0] * Q[1] + W[1] * Q[0] + W[2] * Q[3]);
    QDOT[3] = 0.5 * (-W[0] * Q[0] - W[1] * Q[1] - W[2] * Q[2]);
}

/***********************************************************************************
**
** Function: MatrixMxNf_Mult
**
** Notes:   None
*/
void MatrixMxNf_Mult(float Result[], const float Left[], unsigned int LeftRowSize, unsigned int LeftColSize,
                     const float Right[], unsigned int RightRowSize, unsigned int RightColSize)
{
    float       *ResultPtr;
    unsigned int Row, Col, Element;
    unsigned int RightOffset, LeftOffset;

    ResultPtr   = Result;
    LeftOffset  = 0;
    RightOffset = 0;
    for (Row = 0; Row < LeftRowSize; Row++)
    {
        for (Col = 0; Col < RightColSize; Col++)
        {
            RightOffset = Col;
            *ResultPtr  = 0.0;
            for (Element = 0; Element < RightRowSize; Element++)
            {
                *ResultPtr += *(Left + LeftOffset) * *(Right + RightOffset);
                LeftOffset += 1;
                RightOffset += RightColSize;
            }
            ResultPtr++;
            LeftOffset = 0;
        }
        Left += LeftColSize;
        RightOffset = 0;
    }

} /* End MatrixMxNf_Mult */

void MatrixMxNf_Copy(float Result[], const float Input[], unsigned int RowSize, unsigned int ColSize)
{
    unsigned int Element;
    unsigned int NumElements;

    NumElements = RowSize * ColSize;
    for (Element = 0; Element < NumElements; Element++)
    {
        *(Result + Element) = *(Input + Element);
    }
} /* End MatrixMxNf_Copy */

void Matrix3x3f_MultVec(Vector3f *Result, const Matrix3x3f *Left, const Vector3f *Right)
{
    Vector3f Rslt;

    MatrixMxNf_Mult(&Rslt.Comp[0], &Left->Comp[0][0], 3, 3, &Right->Comp[0], 3, 1);
    MatrixMxNf_Copy(&Result->Comp[0], &Rslt.Comp[0], 3, 1);

} /* End Matrix3x3f_MultVec() */

/**********************************************************************/
/* Ref Montenbruck and Gill, "Satellite Orbits: Models, Methods,      */
/* Applications", TL1080.M66                                          */
void HiFiEarthPrecNute(double JD, double C_TETE_J2000[3][3])
{

    double        P[3][3], N[3][3];
    static long   First = 1;
    static double Al, Bl, Alp, Blp, AF, BF, AD, BD, AOm, BOm;
    static double A2R;
    long          i;
    double        T, zeta, z, theta;
    double        cos_zeta, sin_zeta, cos_theta, sin_theta, cos_z, sin_z;
    double        dpsi, de, l, lp, F, D, Om, phi, e, ep;
    double        cos_e, sin_e, cos_ep, sin_ep, cos_dpsi, sin_dpsi;

    static double pl[106]  = {0, 0, -2, 2,  -2, 1,  0,  2,  0, 0, 0, 0, 0,  2,  0,  0,  0,  0,  0, -2, 0,  2,
                              0, 1, 2,  0,  0,  0,  -1, 0,  0, 1, 0, 1, 1,  -1, 0,  1,  -1, -1, 1, 0,  2,  1,
                              2, 0, -1, -1, 1,  -1, 1,  0,  0, 1, 1, 2, 0,  0,  1,  0,  1,  2,  0, 1,  0,  1,
                              1, 1, -1, -2, 3,  0,  1,  -1, 2, 1, 3, 0, -1, 1,  -2, -1, 2,  1,  1, -2, -1, 1,
                              2, 2, 1,  0,  3,  1,  0,  -1, 0, 0, 0, 1, 0,  1,  1,  2,  0,  0};
    static double plp[106] = {0, 0, 0, 0, 0, -1, -2, 0, 0, 1, 1, -1, 0, 0,  0,  2, 1, 2,  -1, 0,  -1, 0, 1, 0, 1, 0,  1,
                              1, 0, 1, 0, 0, 0,  0,  0, 0, 0, 0, 0,  0, 0,  0,  0, 0, 0,  0,  0,  0,  0, 0, 1, 1, -1, 0,
                              0, 0, 0, 0, 0, 0,  -1, 0, 1, 0, 0, 1,  0, -1, -1, 0, 0, -1, 1,  0,  0,  0, 0, 0, 0, 0,  0,
                              0, 0, 1, 0, 0, 0,  -1, 0, 0, 0, 0, 0,  0, 1,  -1, 0, 0, 1,  0,  -1, 1,  0, 0, 0, 1};
    static double pD[106]  = {0, 0, 2, -2, 2, 0, 2, -2, 2, 0,  2, 2, 2, 0, 2, 0,  0, 2, 0,  0, 2, 0,  2, 0, 0, -2, -2,
                              0, 0, 2, 2,  0, 2, 2, 0,  2, 0,  0, 0, 2, 2, 2, 0,  2, 2, 2,  2, 0, 0,  2, 0, 2, 2,  2,
                              0, 2, 0, 2,  2, 0, 0, 2,  0, -2, 0, 0, 2, 2, 2, 0,  2, 2, 2,  2, 0, 0,  0, 2, 0, 0,  2,
                              2, 0, 2, 2,  2, 4, 0, 2,  2, 0,  4, 2, 2, 2, 0, -2, 2, 0, -2, 2, 0, -2, 0, 2, 0};
    static double pF[106]  = {0,  0,  0,  0,  0,  -1, -2, 0,  -2, 0,  -2, -2, -2, -2, -2, 0,  0,  -2, 0,  2, -2, -2,
                              -2, -1, -2, 2,  2,  0,  1,  -2, 0,  0,  0,  0,  -2, 0,  2,  0,  0,  2,  0,  2, 0,  -2,
                              0,  0,  0,  2,  -2, 2,  -2, 0,  0,  2,  0,  -2, 2,  2,  -2, -2, 0,  0,  -2, 0, 1,  0,
                              0,  0,  2,  0,  0,  2,  0,  -2, 0,  0,  0,  1,  0,  -4, 2,  4,  -4, -2, 2,  4, 0,  -2,
                              -2, 2,  2,  -2, -2, -2, 0,  2,  0,  -1, 2,  -2, 0,  -2, 2,  2,  4,  1};
    static double pOm[106] = {1, 2, 1, 0, 2, 0, 1, 1, 2, 0, 2, 2, 1, 0, 0, 0, 1, 2, 1, 1, 1, 1, 1, 0, 0, 1, 0,
                              2, 1, 0, 2, 0, 1, 2, 0, 2, 0, 1, 1, 2, 1, 2, 0, 2, 2, 0, 1, 1, 1, 1, 0, 2, 2, 2,
                              0, 2, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 2, 2, 1, 2, 2, 2, 1, 1, 2, 0, 2, 2, 0, 2,
                              2, 0, 2, 1, 2, 2, 0, 1, 2, 1, 2, 2, 0, 1, 1, 1, 2, 0, 0, 1, 1, 0, 0, 2, 0};
    static double dp0[106] = {-171996, 2062, 46,   11,  -3, -3, -2,  1,   -13187, 1426, -517, 217, 129, 48, -22,   17,
                              -15,     -16,  -12,  -6,  -5, 4,  4,   -4,  1,      1,    -1,   1,   1,   -1, -2274, 712,
                              -386,    -301, -158, 123, 63, 63, -58, -59, -51,    -38,  29,   29,  -31, 26, 21,    16,
                              -13,     -10,  -7,   7,   -7, -8, 6,   6,   -6,     -7,   6,    -5,  5,   -5, -4,    4,
                              -4,      -3,   3,    -3,  -3, -2, -3,  -3,  2,      -2,   2,    -2,  2,   2,  1,     -1,
                              1,       -2,   -1,   1,   -1, -1, 1,   1,   1,      -1,   -1,   1,   1,   -1, 1,     1,
                              -1,      -1,   -1,   -1,  -1, -1, -1,  1,   -1,     1};
    static double dp1[40]  = {-174.2, 0.2, 0, 0, 0, 0, 0, 0, -1.6, -3.4, 1.2,  -0.5, 0.1,  0, 0, -0.1, 0, 0.1, 0,    0,
                              0,      0,   0, 0, 0, 0, 0, 0, 0,    0,    -0.2, 0.1,  -0.4, 0, 0, 0,    0, 0.1, -0.1, 0};
    static double de0[106] = {92025, -895, -24, 0,  1,  0,  1,  0,   5736, 54, 224, -95, -70, 1,  0,   0,   9,  7,
                              6,     3,    3,   -2, -2, 0,  0,  0,   0,    0,  0,   0,   977, -7, 200, 129, -1, -53,
                              -2,    -33,  32,  26, 27, 16, -1, -12, 13,   -1, -10, -8,  7,   5,  0,   -3,  3,  3,
                              0,     -3,   3,   3,  -3, 3,  0,  3,   0,    0,  0,   0,   0,   1,  1,   1,   1,  1,
                              -1,    1,    -1,  1,  0,  -1, -1, 0,   -1,   1,  0,   -1,  1,   1,  0,   0,   -1, 0,
                              0,     0,    0,   0,  0,  0,  0,  0,   0,    0,  0,   0,   0,   0,  0,   0};
    static double de1[40]  = {8.9, 0.5, 0, 0, 0, 0, 0, 0, -3.1, -0.1, -0.6, 0.3, 0, 0,    0, 0, 0, 0, 0, 0,
                              0,   0,   0, 0, 0, 0, 0, 0, 0,    0,    -0.5, 0,   0, -0.1, 0, 0, 0, 0, 0, 0};

    if (First)
    {
        First = 0;
        A2R   = D2R / 3600.0;
        Al    = 134.0 * 3600.0 + 57.0 * 60.0 + 46.733;
        Bl    = 477198.0 * 3600.0 + 52 * 60.0 + 2.633;
        Alp   = 357.0 * 3600.0 + 31.0 * 60.0 + 39.804;
        Blp   = 35999.0 * 3600.0 + 3.0 * 60.0 + 1.224;
        AF    = 93.0 * 3600.0 + 16.0 * 60.0 + 18.877;
        BF    = 483202.0 * 3600.0 + 1.0 * 60.0 + 3.137;
        AD    = 297.0 * 3600.0 + 51.0 * 60.0 + 1.307;
        BD    = 445267.0 * 3600.0 + 6.0 * 60.0 + 41.328;
        AOm   = 125.0 * 3600.0 + 2.0 * 60 + 40.280;
        BOm   = -(1934.0 * 3600.0 + 8.0 * 60.0 + 10.539);
    }

    T     = (JD - 2451545.0) / 36525.0;
    zeta  = (2306.2181 + (0.30188 + 0.017998 * T) * T) * T * A2R;
    theta = (2004.3109 - (0.42665 + 0.041833 * T) * T) * T * A2R;
    z     = zeta + (0.79280 + 0.000205 * T) * T * T * A2R;

    cos_zeta  = cos(zeta);
    sin_zeta  = sin(zeta);
    cos_theta = cos(theta);
    sin_theta = sin(theta);
    cos_z     = cos(z);
    sin_z     = sin(z);
    P[0][0]   = -sin_z * sin_zeta + cos_zeta * cos_theta * cos_z;
    P[1][0]   = cos_z * sin_zeta + sin_z * cos_theta * cos_zeta;
    P[2][0]   = sin_theta * cos_zeta;
    P[0][1]   = -sin_z * cos_zeta - cos_z * cos_theta * sin_zeta;
    P[1][1]   = cos_z * cos_zeta - sin_z * cos_theta * sin_zeta;
    P[2][1]   = -sin_theta * sin_zeta;
    P[0][2]   = -cos_z * sin_theta;
    P[1][2]   = -sin_z * sin_theta;
    P[2][2]   = cos_theta;

    dpsi = 0.0;
    de   = 0.0;
    l    = Al + (Bl + (31.310 + 0.064 * T) * T) * T;
    lp   = Alp + (Blp + (-0.577 - 0.012 * T) * T) * T;
    F    = AF + (BF + (-13.257 + 0.011 * T) * T) * T;
    D    = AD + (BD + (-6.891 + 0.019 * T) * T) * T;
    Om   = AOm + (BOm + (7.455 + 0.008 * T) * T) * T;
    for (i = 0; i < 40; i++)
    {
        phi = (pl[i] * l + plp[i] * lp + pD[i] * D + pF[i] * F + pOm[i] * Om) * A2R;
        dpsi += (dp0[i] + dp1[i] * T) * sin(phi);
        de += (de0[i] + de1[i] * T) * cos(phi);
    }
    for (i = 40; i < 106; i++)
    {
        phi = (pl[i] * l + plp[i] * lp + pF[i] * F + pD[i] * D + pOm[i] * Om) * A2R;
        dpsi += dp0[i] * sin(phi);
        de += de0[i] * cos(phi);
    }
    dpsi *= 1.0E-4 * A2R;
    de *= 1.0E-4 * A2R;

    e  = (23.43929111 + (-46.8150 + (-0.00059 + 0.001813 * T) * T) * T / 3600.0) * D2R;
    ep = e + de;

    cos_e    = cos(e);
    sin_e    = sin(e);
    cos_ep   = cos(ep);
    sin_ep   = sin(ep);
    cos_dpsi = cos(dpsi);
    sin_dpsi = sin(dpsi);
    N[0][0]  = cos_dpsi;
    N[1][0]  = cos_ep * sin_dpsi;
    N[2][0]  = sin_ep * sin_dpsi;
    N[0][1]  = -cos_e * sin_dpsi;
    N[1][1]  = cos_e * cos_ep * cos_dpsi + sin_e * sin_ep;
    N[2][1]  = cos_e * sin_ep * cos_dpsi - sin_e * cos_ep;
    N[0][2]  = -sin_e * sin_dpsi;
    N[1][2]  = sin_e * cos_ep * cos_dpsi - cos_e * sin_ep;
    N[2][2]  = sin_e * sin_ep * cos_dpsi + cos_e * cos_ep;

    C_TETE_J2000[0][0] = N[0][0] * P[0][0] + N[0][1] * P[1][0] + N[0][2] * P[2][0];
    C_TETE_J2000[0][1] = N[0][0] * P[0][1] + N[0][1] * P[1][1] + N[0][2] * P[2][1];
    C_TETE_J2000[0][2] = N[0][0] * P[0][2] + N[0][1] * P[1][2] + N[0][2] * P[2][2];
    C_TETE_J2000[1][0] = N[1][0] * P[0][0] + N[1][1] * P[1][0] + N[1][2] * P[2][0];
    C_TETE_J2000[1][1] = N[1][0] * P[0][1] + N[1][1] * P[1][1] + N[1][2] * P[2][1];
    C_TETE_J2000[1][2] = N[1][0] * P[0][2] + N[1][1] * P[1][2] + N[1][2] * P[2][2];
    C_TETE_J2000[2][0] = N[2][0] * P[0][0] + N[2][1] * P[1][0] + N[2][2] * P[2][0];
    C_TETE_J2000[2][1] = N[2][0] * P[0][1] + N[2][1] * P[1][1] + N[2][2] * P[2][1];
    C_TETE_J2000[2][2] = N[2][0] * P[0][2] + N[2][1] * P[1][2] + N[2][2] * P[2][2];
}

/**********************************************************************/
/* GPS Epoch is 6 Jan 1980 00:00:00.0 which is JD = 2444244.5         */
/* GPS Time is expressed in weeks and seconds                         */
/* GPS Time rolls over every 1024 weeks                               */
/* *******************************************************************/
double GpsTime_TO_JD(long GpsRollover, long GpsWeek, double GpsSecond)
{
    double DaysSinceWeek = 0.0, DaysSinceRollover = 0.0, DaysSinceEpoch = 0.0, JD = 0.0;

    DaysSinceWeek     = GpsSecond / 86400.0;
    DaysSinceRollover = DaysSinceWeek + 7.0 * GpsWeek;
    DaysSinceEpoch    = DaysSinceRollover + 7168.0 * GpsRollover;
    JD                = DaysSinceEpoch + 2444244.5;

    return (JD);
}
/**********************************************************************/
/*  Find Greenwich Mean Sidereal Time (GMST)                          */
/*  Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991.  */
/*  GMST is output in units of days.                                  */
/* ********************************************************************/
double JD_TO_GMST(double JD)
{
    double T = 0.0, JD0 = 0.0, GMST0 = 0.0, GMST = 0.0;

    JD0 = floor(JD) + 0.5;

    T = (JD0 - 2451545.0) / 36525.0;

    /* .. GMST at UT=0h, in deg */
    GMST0 = 100.46061837 + T * (36000.770053608 + T * (3.87933E-4 - T / 3.871E7));

    /* .. Convert to days */
    GMST0 /= 360.0;

    GMST = GMST0 + 1.00273790935 * (JD - JD0);

    GMST -= (int)(GMST);

    return (GMST);
}
/**********************************************************************/
/* GPS Epoch is 6 Jan 1980 00:00:00.0 UTC                             */
/* which is 6 Jan 1980 00:00:19.0 TAI                                 */
/* J2000 is 1 Jan 2000 12:00:00.0 TT                                  */
/* which is 1 Jan 2000 11:59:27.816 TAI                               */
/* so J2000-GPS epoch is 7300.5 days minus (19+32.184) sec            */
double GpsDateToGpsTime(long GpsRollover, long GpsWeek, double GpsSecond)
{
    return (((GpsRollover * 1024.0 + GpsWeek) * 7.0 - 7300.5) * 86400.0 + GpsSecond);
}
/**********************************************************************/
/*   Convert Time to Year, Month, Day, Hour, Minute, and Second       */
/*   Time is seconds since J2000 epoch (01 Jan 2000 12:00:00.0)       */
/*   Outputs are rounded to LSB to avoid loss of precision            */
/*   Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991. */
/*   This function is agnostic to the TT-to-UTC offset.  You get out  */
/*   what you put in.                                                 */
void TimeToDate(double Time, long *Year, long *Month, long *Day, long *Hour, long *Minute, double *Second, double LSB)
{
    double Z, F, A, B, C, D, E, alpha;
    double FD, JD;

    JD = Time / 86400.0 + 2451545.0;

    Z = floor(JD + 0.5);
    F = (JD + 0.5) - Z;

    if (Z < 2299161.0)
    {
        A = Z;
    }
    else
    {
        alpha = floor((Z - 1867216.25) / 36524.25);
        A     = Z + 1.0 + alpha - floor(alpha / 4.0);
    }

    B = A + 1524.0;
    C = floor((B - 122.1) / 365.25);
    D = floor(365.25 * C);
    E = floor((B - D) / 30.6001);

    FD   = B - D - floor(30.6001 * E) + F;
    *Day = (long)FD;

    if (E < 14.0)
    {
        *Month = (long)(E - 1.0);
        *Year  = (long)(C - 4716.0);
    }
    else
    {
        *Month = (long)(E - 13.0);
        *Year  = (long)(C - 4715.0);
    }

    FD = Time - 43200.0 + 0.5 * LSB;
    FD = FD - ((long)(FD / 86400.0)) * 86400.0;
    if (FD < 0.0)
        FD += 86400.0;

    *Hour = (long)(FD / 3600.0);

    FD -= 3600.0 * (*Hour);

    *Minute = (long)(FD / 60.0);

    *Second = FD - 60.0 * (*Minute);

    /* Clean up roundoff */
    *Second = ((long)(*Second / LSB)) * LSB;
}
