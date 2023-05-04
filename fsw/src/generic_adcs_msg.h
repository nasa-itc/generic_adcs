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
#define GENERIC_ADCS_SEND_AD_CMD_CC          3
#define GENERIC_ADCS_SEND_GNC_CMD_CC         4
#define GENERIC_ADCS_SEND_AC_CMD_CC          5
#define GENERIC_ADCS_SET_MODE_CC             6

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

typedef struct
{
    /* Every command requires a header used to identify it */
    uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8    Mode;
} Generic_ADCS_Mode_cmd_t;

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

/*
** Generic_ADCS DI type definition
*/
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

/*
** Generic_ADCS AD type definition
*/
typedef struct 
{
    double bvb[3];
} OS_PACK Generic_ADCS_AD_Mag_Tlm_Payload_t;

typedef struct
{
    Generic_ADCS_AD_Mag_Tlm_Payload_t Mag;
} OS_PACK Generic_ADCS_AD_Tlm_Payload_t;

typedef struct
{
    uint8                         TlmHeader[CFE_SB_TLM_HDR_SIZE];
    Generic_ADCS_AD_Tlm_Payload_t Payload;
} OS_PACK Generic_ADCS_AD_Tlm_t;
#define GENERIC_ADCS_AD_LNGTH sizeof ( Generic_ADCS_AD_Tlm_t )

/*
** Generic_ADCS GNC type definition
*/
typedef struct {
    double DT;
    double MaxMcmd;
    uint8  Mode;
    double bvb[3];
    double Mcmd[3];
    double Tcmd[3];
} OS_PACK Generic_ADCS_GNC_Tlm_Payload_t;

typedef struct
{
    uint8                          TlmHeader[CFE_SB_TLM_HDR_SIZE];
    Generic_ADCS_GNC_Tlm_Payload_t Payload;
} OS_PACK Generic_ADCS_GNC_Tlm_t;
#define GENERIC_ADCS_GNC_LNGTH sizeof ( Generic_ADCS_GNC_Tlm_t )

/*
** Generic_ADCS AC type definition
*/
typedef struct {
    double b_range;
    double Kb;
    double bold[3];
    double bdot[3];
} OS_PACK Generic_ADCS_AC_Bdot_Tlm_t;

typedef struct {
    Generic_ADCS_AC_Bdot_Tlm_t Bdot;
} OS_PACK Generic_ADCS_AC_Tlm_Payload_t;

typedef struct {
    uint8                         TlmHeader[CFE_SB_TLM_HDR_SIZE];
    Generic_ADCS_AC_Tlm_Payload_t Payload;
} OS_PACK Generic_ADCS_AC_Tlm_t;
#define GENERIC_ADCS_AC_LNGTH sizeof ( Generic_ADCS_AC_Tlm_t )

#endif
