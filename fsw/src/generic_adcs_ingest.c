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
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css[0].axis[0], &DI->Css[0].axis[1], &DI->Css[0].axis[2], &DI->Css[0].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css[1].axis[0], &DI->Css[1].axis[1], &DI->Css[1].axis[2], &DI->Css[1].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css[2].axis[0], &DI->Css[2].axis[1], &DI->Css[2].axis[2], &DI->Css[2].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css[3].axis[0], &DI->Css[3].axis[1], &DI->Css[3].axis[2], &DI->Css[3].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css[4].axis[0], &DI->Css[4].axis[1], &DI->Css[4].axis[2], &DI->Css[4].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css[5].axis[0], &DI->Css[5].axis[1], &DI->Css[5].axis[2], &DI->Css[5].scale, junk, &newline);
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

    Fss->valid = fss->Generic_fss.ErrorCode;
    if (Fss->valid == 0) {
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
}
