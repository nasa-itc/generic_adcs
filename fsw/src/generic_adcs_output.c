/*******************************************************************************
** Purpose:
**   This file implements the functions to output messages to the actuator applications.
**
*******************************************************************************/

#include "generic_adcs_app.h"

static void send_mtb_commands(void);
static void mcmd_to_percent_direction(double Mcmd, uint8 *percent, uint8 *direction);

void Generic_ADCS_output_to_actuators(void)
{
    send_mtb_commands();
}

static void send_mtb_commands(void)
{
    CFE_SB_SetCmdCode((CFE_SB_Msg_t *)&Generic_ADCS_AppData.MtbPctOnCmd, 5);
    mcmd_to_percent_direction(Generic_ADCS_AppData.GNCPacket.Payload.Mcmd[0], &Generic_ADCS_AppData.MtbPctOnCmd.PercentOn_0, &Generic_ADCS_AppData.MtbPctOnCmd.Direction_0);
    mcmd_to_percent_direction(Generic_ADCS_AppData.GNCPacket.Payload.Mcmd[1], &Generic_ADCS_AppData.MtbPctOnCmd.PercentOn_1, &Generic_ADCS_AppData.MtbPctOnCmd.Direction_1);
    mcmd_to_percent_direction(Generic_ADCS_AppData.GNCPacket.Payload.Mcmd[2], &Generic_ADCS_AppData.MtbPctOnCmd.PercentOn_2, &Generic_ADCS_AppData.MtbPctOnCmd.Direction_2);
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t *)&Generic_ADCS_AppData.MtbPctOnCmd);
    CFE_SB_SendMsg((CFE_SB_Msg_t *)&Generic_ADCS_AppData.MtbPctOnCmd);
}

static void mcmd_to_percent_direction(double Mcmd, uint8 *percent, uint8 *direction)
{
    double pct = Mcmd / Generic_ADCS_AppData.GNCPacket.Payload.MaxMcmd;
    *direction = 0;
    if (pct < 0) {
        pct *= -1.0;
        *direction = 1;
    }
    if (pct > 100) pct = 100;
    *percent = pct;
}