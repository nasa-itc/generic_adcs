/*******************************************************************************
** Purpose:
**   This file implements utility functions used by ADCS.
**
*******************************************************************************/

#include <math.h>
#include "generic_adcs_utilities.h"

// Magnitude of v vector
double MAGV(double v[3])
{
    return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}

/**********************************************************************/
/* Find components of V in A, given components of V in B, and qab     */
void QxV(double QAB[4],double Vb[3],double Va[3])
{
      double qq[4][4];
      long i,j;

      for(i=0;i<4;i++) {
         for(j=i;j<4;j++) qq[i][j] = QAB[i]*QAB[j];
      }

      Va[0] = ( qq[0][0]-qq[1][1]-qq[2][2]+qq[3][3])*Vb[0]
                          + 2.0*((qq[0][1]+qq[2][3])*Vb[1]
                                +(qq[0][2]-qq[1][3])*Vb[2]);
      Va[1] = (-qq[0][0]+qq[1][1]-qq[2][2]+qq[3][3])*Vb[1]
                          + 2.0*((qq[1][2]+qq[0][3])*Vb[2]
                                +(qq[0][1]-qq[2][3])*Vb[0]);
      Va[2] = (-qq[0][0]-qq[1][1]+qq[2][2]+qq[3][3])*Vb[2]
                          + 2.0*((qq[0][2]+qq[1][3])*Vb[0]
                                +(qq[1][2]-qq[0][3])*Vb[1]);
}
