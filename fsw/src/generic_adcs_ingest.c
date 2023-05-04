/*******************************************************************************
** Purpose:
**   This file implements the functions to ingest messages from the sensor applications.
**
*******************************************************************************/

#include <stdio.h>
#include "generic_mag_msg.h"
#include "generic_adcs_utilities.h"
#include "generic_adcs_ingest.h"

static const double NANO = 1.0e-9;

void Generic_ADCS_ingest_init(FILE *in, Generic_ADCS_DI_Tlm_Payload_t *DI)
{
    char junk[120], newline;
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Mag.qbs[0], &DI->Mag.qbs[1], &DI->Mag.qbs[2], &DI->Mag.qbs[3], junk, &newline);
}

void Generic_ADCS_ingest_generic_mag(CFE_SB_MsgPtr_t Msg, Generic_ADCS_DI_Mag_Tlm_Payload_t *Mag)
{
    GENERIC_MAG_Device_tlm_t *mag = (GENERIC_MAG_Device_tlm_t *)Msg;
    double msg_bvb[3] = {mag->Generic_mag.MagneticIntensityX, mag->Generic_mag.MagneticIntensityY, mag->Generic_mag.MagneticIntensityZ};
    QxV(Mag->qbs, msg_bvb, Mag->bvb); // convert from sensor frame to body frame
    /* convert from raw data to engineering units of Teslas */
    Mag->bvb[0] *= NANO;
    Mag->bvb[1] *= NANO;
    Mag->bvb[2] *= NANO;
    /* OS_printf("Generic_ADCS_ingest_generic_mag: %f %f %f\n", Mag->bvb[0], Mag->bvb[1], Mag->bvb[2]); */
}
