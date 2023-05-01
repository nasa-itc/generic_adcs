/*******************************************************************************
** Purpose:
**   This file implements the functions to ingest messages from the sensor applications.
**
*******************************************************************************/

#include "generic_adcs_ingest.h"
#include "generic_mag_msg.h"

static const double NANO = 1.0e-9;

void Generic_ADCS_ingest_generic_mag(CFE_SB_MsgPtr_t Msg, Generic_ADCS_DI_Mag_Tlm_Payload_t *Mag)
{
    GENERIC_MAG_Device_tlm_t *mag = (GENERIC_MAG_Device_tlm_t *)Msg;
    /* for now just assume always valid */
    /* for now just assume sensor and body frames align */
    Mag->bvb[0] = mag->Generic_mag.MagneticIntensityX;
    Mag->bvb[1] = mag->Generic_mag.MagneticIntensityY;
    Mag->bvb[2] = mag->Generic_mag.MagneticIntensityZ;
    /* convert from raw data to engineering units of Teslas */
    Mag->bvb[0] *= NANO;
    Mag->bvb[1] *= NANO;
    Mag->bvb[2] *= NANO;
    /* OS_printf("Generic_ADCS_ingest_generic_mag: %f %f %f\n", Mag->bvb[0], Mag->bvb[1], Mag->bvb[2]); */
}
