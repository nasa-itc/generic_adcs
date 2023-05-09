/*******************************************************************************
** Purpose:
**   This file implements the functions to ingest messages from the sensor applications.
**
*******************************************************************************/

#include <stdio.h>
#include <math.h>
#include "generic_mag_msg.h"
#include "generic_fss_msg.h"
#include "generic_css_msg.h"
#include "generic_adcs_utilities.h"
#include "generic_adcs_ingest.h"

static const double NANO = 1.0e-9;

void Generic_ADCS_ingest_init(FILE *in, Generic_ADCS_DI_Tlm_Payload_t *DI)
{
    char junk[120], newline;
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Mag.qbs[0], &DI->Mag.qbs[1], &DI->Mag.qbs[2], &DI->Mag.qbs[3], junk, &newline);
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Fss.qbs[0], &DI->Fss.qbs[1], &DI->Fss.qbs[2], &DI->Fss.qbs[3], junk, &newline);
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[0].axis[0], &DI->Css.Sensor[0].axis[1], &DI->Css.Sensor[0].axis[2], &DI->Css.Sensor[0].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[1].axis[0], &DI->Css.Sensor[1].axis[1], &DI->Css.Sensor[1].axis[2], &DI->Css.Sensor[1].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[2].axis[0], &DI->Css.Sensor[2].axis[1], &DI->Css.Sensor[2].axis[2], &DI->Css.Sensor[2].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[3].axis[0], &DI->Css.Sensor[3].axis[1], &DI->Css.Sensor[3].axis[2], &DI->Css.Sensor[3].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[4].axis[0], &DI->Css.Sensor[4].axis[1], &DI->Css.Sensor[4].axis[2], &DI->Css.Sensor[4].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[5].axis[0], &DI->Css.Sensor[5].axis[1], &DI->Css.Sensor[5].axis[2], &DI->Css.Sensor[5].scale, junk, &newline);
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Imu.qbs[0], &DI->Imu.qbs[1], &DI->Imu.qbs[2], &DI->Imu.qbs[3], junk, &newline);
    fscanf(in, "%lf %lf %lf%[^\n]%[\n]", &DI->Imu.pos[0], &DI->Imu.pos[1], &DI->Imu.pos[2], junk, &newline);
}

void Generic_ADCS_ingest_generic_mag(CFE_SB_MsgPtr_t Msg, Generic_ADCS_DI_Mag_Tlm_Payload_t *Mag)
{
    GENERIC_MAG_Device_tlm_t *mag = (GENERIC_MAG_Device_tlm_t *)Msg;
    double msg_bvs[3] = {mag->Generic_mag.MagneticIntensityX, mag->Generic_mag.MagneticIntensityY, mag->Generic_mag.MagneticIntensityZ};
    QxV(Mag->qbs, msg_bvs, Mag->bvb); // convert from sensor frame to body frame
    /* convert from raw data to engineering units of Teslas */
    Mag->bvb[0] *= NANO;
    Mag->bvb[1] *= NANO;
    Mag->bvb[2] *= NANO;
    /* OS_printf("Generic_ADCS_ingest_generic_mag: %f %f %f\n", Mag->bvb[0], Mag->bvb[1], Mag->bvb[2]); */
}

void Generic_ADCS_ingest_generic_fss(CFE_SB_MsgPtr_t Msg, Generic_ADCS_DI_Fss_Tlm_Payload_t *Fss)
{
    GENERIC_FSS_Device_tlm_t *fss = (GENERIC_FSS_Device_tlm_t *)Msg;

    Fss->valid = 0;
    if (fss->Generic_fss.ErrorCode == 0) Fss->valid = 1;
    if (Fss->valid == 1) {
        double svs[3];
        double ta = tan(fss->Generic_fss.Alpha);
        double tb = tan(fss->Generic_fss.Beta);
        svs[2] = 1.0 / sqrt(1 + ta*ta + tb*tb);
        svs[0] = svs[2] * ta;
        svs[1] = svs[2] * tb;
        QxV(Fss->qbs, svs, Fss->svb); // convert from sensor frame to body frame
    } else {
        Fss->svb[0] = 0.0;
        Fss->svb[1] = 0.0;
        Fss->svb[2] = 0.0;
    }
}

void Generic_ADCS_ingest_generic_css(CFE_SB_MsgPtr_t Msg, Generic_ADCS_DI_Css_Tlm_Payload_t *Css)
{
    GENERIC_CSS_Device_tlm_t *css = (GENERIC_CSS_Device_tlm_t *)Msg;
    Css->Sensor[0].percenton = css->Generic_css.Voltage[0] * Css->Sensor[0].scale;
    Css->Sensor[1].percenton = css->Generic_css.Voltage[1] * Css->Sensor[1].scale;
    Css->Sensor[2].percenton = css->Generic_css.Voltage[2] * Css->Sensor[2].scale;
    Css->Sensor[3].percenton = css->Generic_css.Voltage[3] * Css->Sensor[3].scale;
    Css->Sensor[4].percenton = css->Generic_css.Voltage[4] * Css->Sensor[4].scale;
    Css->Sensor[5].percenton = css->Generic_css.Voltage[5] * Css->Sensor[5].scale;

    double svb[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 6; i++) {
        svb[0] += Css->Sensor[i].axis[0] * Css->Sensor[i].percenton;
        svb[1] += Css->Sensor[i].axis[1] * Css->Sensor[i].percenton;
        svb[2] += Css->Sensor[i].axis[2] * Css->Sensor[i].percenton;
    }
    UNITV(svb);

    Css->svb[0] = svb[0];
    Css->svb[1] = svb[1];
    Css->svb[2] = svb[2];
    if (MAGV(svb) > 0.0) {
        Css->valid = 1;
    } else {
        Css->valid = 0;
    }
}
