/*******************************************************************************
** Purpose:
**   This file contains the source code for the attitude determination and
**   attitude control routines of the Generic ADCS application.
**
*******************************************************************************/

#include <math.h>
#include "generic_adcs_app.h"

static void AD_mag(Generic_ADCS_DI_Mag_Tlm_Payload_t DI_Mag, Generic_ADCS_AD_Mag_Tlm_Payload_t *AD_Mag);
static void AD_to_GNC(Generic_ADCS_AD_Tlm_Payload_t AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC);
static void AC_bdot(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Bdot_Tlm_t *AC_bdot);
static double MAGV(double v[3]);

void Generic_ADCS_execute_attitude_determination_and_attitude_control(void)
{
    AD_mag(Generic_ADCS_AppData.DIPacket.Payload.Mag, &Generic_ADCS_AppData.ADPacket.Payload.Mag);

    AD_to_GNC(Generic_ADCS_AppData.ADPacket.Payload, &Generic_ADCS_AppData.GNCPacket.Payload);

    // if mode == bdot
    AC_bdot(&Generic_ADCS_AppData.GNCPacket.Payload, &Generic_ADCS_AppData.ACSPacket.Payload.Bdot);

}

static void AD_mag(Generic_ADCS_DI_Mag_Tlm_Payload_t DI_Mag, Generic_ADCS_AD_Mag_Tlm_Payload_t *AD_Mag)
{
    /* AD very simple for magnetometer... there is only one mag and no fusion with anything else */
    for (int i = 0; i < 3; i++) {
        AD_Mag->bvb[i] = DI_Mag.bvb[i];
    }
}

static void AD_to_GNC(Generic_ADCS_AD_Tlm_Payload_t AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC)
{
    for (int i = 0; i < 3; i++) {
        GNC->bvb[i] = AD.Mag.bvb[i];
    }
}

static void AC_bdot(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Bdot_Tlm_t *ACS)
{
    static long First = 1;
    if (First) {
        First = 0;
        GNC->DT = 0.1; // seconds
        ACS->b_range = 4.096E-6; // Teslas
        ACS->Kb = 200.0;
        for (int i = 0; i < 3; i++) {
            ACS->bold[i] = GNC->bvb[i];
        }
        GNC->MaxMcmd = 1.42; // A-m^2
    }
    /* apply control only if b-field is in range */
    if (MAGV(GNC->bvb) > ACS->b_range) {
        for(int i = 0; i < 3; i++) {
            /* backward difference b-field derivative */
            ACS->bdot[i] = (GNC->bvb[i] - ACS->bold[i]) / GNC->DT;
            /* store old b-field */
            ACS->bold[i] = GNC->bvb[i];
            /* traditional b-dot algorithm */
            GNC->Mcmd[i] = -ACS->Kb * ACS->bdot[i] / MAGV(GNC->bvb);
            /* ensure wheels disabled */
            GNC->Tcmd[i] = 0.0;
        }
    } else {
        for (int i = 0; i < 3; i++) {
            GNC->Mcmd[i] = 0.0;
            GNC->Tcmd[i] = 0.0;
        }
    }
}

static double MAGV(double v[3])
{
    return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}
