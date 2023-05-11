/*******************************************************************************
** Purpose:
**   This file contains the source code for the attitude determination and
**   attitude control routines of the Generic ADCS application.
**
*******************************************************************************/

#include <stdio.h>
#include "generic_adcs_app.h"
#include "generic_adcs_utilities.h"
#include "generic_adcs_adac.h"

static void AD_imu(const Generic_ADCS_DI_Imu_Tlm_Payload_t *DI_IMU, Generic_ADCS_AD_Imu_Tlm_Payload_t *AD_IMU);
static void AD_mag(const Generic_ADCS_DI_Mag_Tlm_Payload_t *DI_Mag, Generic_ADCS_AD_Mag_Tlm_Payload_t *AD_Mag);
static void AD_sol(const Generic_ADCS_DI_Fss_Tlm_Payload_t *DI_FSS, const Generic_ADCS_DI_Css_Tlm_Payload_t *DI_CSS, Generic_ADCS_AD_Sol_Tlm_Payload_t *AD_Sol);
static void AD_to_GNC(const Generic_ADCS_AD_Tlm_Payload_t *AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC);
static void AC_bdot(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Bdot_Tlm_t *AC_bdot);

void Generic_ADCS_init_attitude_determination_and_attitude_control(FILE *in, Generic_ADCS_AD_Tlm_Payload_t *AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Tlm_Payload_t *ACS)
{
    char junk[120], newline;
    // AD
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf%[^\n]%[\n]", &AD->Imu.alpha, junk, &newline);
    AD->Imu.init = 0;
    // GNC
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf%[^\n]%[\n]", &GNC->DT, junk, &newline);
    fscanf(in, "%lf%[^\n]%[\n]", &GNC->MaxMcmd, junk, &newline);
    // AC
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf%[^\n]%[\n]", &ACS->Bdot.b_range, &ACS->Bdot.Kb, junk, &newline);
}

void Generic_ADCS_execute_attitude_determination_and_attitude_control(const Generic_ADCS_DI_Tlm_Payload_t *DI, Generic_ADCS_AD_Tlm_Payload_t *AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Tlm_Payload_t *ACS)
{
    AD_imu(&DI->Imu, &AD->Imu);
    AD_mag(&DI->Mag, &AD->Mag);
    AD_sol(&DI->Fss, &DI->Css, &AD->Sol);

    AD_to_GNC(AD, GNC);

    switch(GNC->Mode) {
    case BDOT_MODE:
        AC_bdot(GNC, &ACS->Bdot);
        break;
    
    case PASSIVE_MODE:
    default:
        for (int i = 0; i < 3; i++) {
            GNC->Mcmd[i] = 0.0;
            GNC->Tcmd[i] = 0.0;
        }
        break;
    }
}

static void AD_imu(const Generic_ADCS_DI_Imu_Tlm_Payload_t *DI_IMU, Generic_ADCS_AD_Imu_Tlm_Payload_t *AD_IMU)
{
    if (DI_IMU->valid) {
        AD_IMU->valid = 1;
        for (int i = 0; i < 3; i++) {
            AD_IMU->acc[i] = DI_IMU->acc[i];
        }

        if (AD_IMU->init == 0) {
            for (int i = 0; i < 3; i++) {
                AD_IMU->wbn[i] = DI_IMU->wbn[i];
            }
            AD_IMU->init = 1;
        } else {
            for (int i = 0; i < 3; i++) {
                AD_IMU->wbn[i] = AD_IMU->alpha * AD_IMU->wbn_prev[i] + (1 - AD_IMU->alpha) * DI_IMU->wbn[i];
            }
        }
        for (int i = 0; i < 3; i++) {
            AD_IMU->wbn_prev[i] = AD_IMU->wbn[i];
        }
    } else {
        AD_IMU->valid = 0;
    }
}

static void AD_mag(const Generic_ADCS_DI_Mag_Tlm_Payload_t *DI_Mag, Generic_ADCS_AD_Mag_Tlm_Payload_t *AD_Mag)
{
    /* AD very simple for magnetometer... there is only one mag and no fusion with anything else */
    for (int i = 0; i < 3; i++) {
        AD_Mag->bvb[i] = DI_Mag->bvb[i];
    }
}

static void AD_sol(const Generic_ADCS_DI_Fss_Tlm_Payload_t *DI_Fss, const Generic_ADCS_DI_Css_Tlm_Payload_t *DI_Css, Generic_ADCS_AD_Sol_Tlm_Payload_t *AD_Sol)
{
    if (DI_Fss->valid == 1) {
        AD_Sol->SunValid = 1;
        AD_Sol->FssValid = 1;
        AD_Sol->svb[0] = DI_Fss->svb[0];
        AD_Sol->svb[1] = DI_Fss->svb[1];
        AD_Sol->svb[2] = DI_Fss->svb[2];
    } else if (DI_Css->valid == 1) {
        AD_Sol->SunValid = 1;
        AD_Sol->FssValid = 0;
        AD_Sol->svb[0] = DI_Css->svb[0];
        AD_Sol->svb[1] = DI_Css->svb[1];
        AD_Sol->svb[2] = DI_Css->svb[2];
    } else {
        AD_Sol->SunValid = 0;
        AD_Sol->FssValid = 0;
        AD_Sol->svb[0] = 0.0;
        AD_Sol->svb[1] = 0.0;
        AD_Sol->svb[2] = 0.0;
    }
}

static void AD_to_GNC(const Generic_ADCS_AD_Tlm_Payload_t *AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC)
{
    for (int i = 0; i < 3; i++) {
        GNC->bvb[i] = AD->Mag.bvb[i];
        GNC->svb[i] = AD->Sol.svb[i];
    }
    GNC->SunValid = AD->Sol.SunValid;
}

static void AC_bdot(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Bdot_Tlm_t *ACS)
{
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
