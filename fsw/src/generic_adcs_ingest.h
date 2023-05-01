/*******************************************************************************
** Purpose:
**   This file has the functions to ingest messages from the sensor applications.
**
*******************************************************************************/
#ifndef _GENERIC_ADCS_INGEST_H_
#define _GENERIC_ADCS_INGEST_H_

#include "cfe.h"
#include "generic_adcs_msg.h"

void Generic_ADCS_ingest_generic_mag(CFE_SB_MsgPtr_t Msg, Generic_ADCS_DI_Mag_Tlm_Payload_t *Mag);

#endif