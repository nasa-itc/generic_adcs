/*******************************************************************************
** Purpose:
**  Define GENERIC_ADCS application commands and telemetry messages
**
*******************************************************************************/
#ifndef _GENERIC_ADCS_MSG_H_
#define _GENERIC_ADCS_MSG_H_

#include "cfe.h"

/*
** Ground Command Codes
*/
#define GENERIC_ADCS_NOOP_CC                 0
#define GENERIC_ADCS_RESET_COUNTERS_CC       1
#define GENERIC_ADCS_SEND_DI_CMD_CC          2

/* 
** Telemetry Request Command Codes
*/
#define GENERIC_ADCS_REQ_HK_TLM              0

/*
** Generic "no arguments" command type definition
*/
typedef struct
{
    /* Every command requires a header used to identify it */
    uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];

} Generic_ADCS_NoArgs_cmd_t;

/*
** Generic_ADCS housekeeping type definition
*/
typedef struct 
{
    uint8   TlmHeader[CFE_SB_TLM_HDR_SIZE];
    uint8   CommandErrorCount;
    uint8   CommandCount;
} OS_PACK Generic_ADCS_Hk_tlm_t;
#define GENERIC_ADCS_HK_TLM_LNGTH sizeof ( Generic_ADCS_Hk_tlm_t )

typedef struct
{
    double bvb[3];
} OS_PACK Generic_ADCS_DI_Mag_Tlm_Payload_t;

typedef struct
{
    Generic_ADCS_DI_Mag_Tlm_Payload_t Mag;
} OS_PACK Generic_ADCS_DI_Tlm_Payload_t;

typedef struct
{
    uint8                         TlmHeader[CFE_SB_TLM_HDR_SIZE];
    Generic_ADCS_DI_Tlm_Payload_t Payload;
} OS_PACK Generic_ADCS_DI_Tlm_t;
#define GENERIC_ADCS_DI_LNGTH sizeof ( Generic_ADCS_DI_Tlm_t )

#endif
