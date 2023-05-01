/*******************************************************************************
** Purpose:
**   This file contains the source code for the attitude determination and
**   attitude control routines of the Generic ADCS application.
**
*******************************************************************************/

#include "generic_adcs_app.h"

static void AD_mag(Generic_ADCS_DI_Mag_Tlm_Payload_t DI_Mag, Generic_ADCS_AD_Mag_Tlm_Payload_t *AD_Mag);

void Generic_ADCS_execute_attitude_determination_and_attitude_control(void)
{
    AD_mag(Generic_ADCS_AppData.DIPacket.Payload.Mag, &Generic_ADCS_AppData.ADPacket.Payload.Mag);
}

static void AD_mag(Generic_ADCS_DI_Mag_Tlm_Payload_t DI_Mag, Generic_ADCS_AD_Mag_Tlm_Payload_t *AD_Mag)
{
    /* AD very simple for magnetometer... there is only one mag and no fusion with anything else */
    for (int i = 0; i < 3; i++) {
        AD_Mag->bvb[i] = DI_Mag.bvb[i];
    }
}

