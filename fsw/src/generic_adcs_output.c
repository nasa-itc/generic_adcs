/*******************************************************************************
** Purpose:
**   This file implements the functions to output messages to the actuator applications.
**
*******************************************************************************/

#include "generic_adcs_app.h"

static void send_mtb_commands(double Mcmd[3]);
static void mcmd_to_percent_direction(double Mcmd, uint8 *percent, uint8 *direction);

void Generic_ADCS_output_to_actuators(const Generic_ADCS_GNC_Tlm_Payload_t *GNC)
{
    send_mtb_commands(GNC->Mcmd);
}

static struct{uint8 Direction; uint8 PercentOn;} CurrentMtb[] = {{1,0}, {1,0}, {1,0}}; // Keep current state so commands are only sent on change
static void send_mtb_commands(double Mcmd[3])
{
    CFE_SB_SetCmdCode((CFE_SB_Msg_t *)&Generic_ADCS_AppData.MtbPctOnCmd, 5);
    mcmd_to_percent_direction(Mcmd[0], &Generic_ADCS_AppData.MtbPctOnCmd.PercentOn_0, &Generic_ADCS_AppData.MtbPctOnCmd.Direction_0);
    mcmd_to_percent_direction(Mcmd[1], &Generic_ADCS_AppData.MtbPctOnCmd.PercentOn_1, &Generic_ADCS_AppData.MtbPctOnCmd.Direction_1);
    mcmd_to_percent_direction(Mcmd[2], &Generic_ADCS_AppData.MtbPctOnCmd.PercentOn_2, &Generic_ADCS_AppData.MtbPctOnCmd.Direction_2);
    if ((Generic_ADCS_AppData.MtbPctOnCmd.Direction_0 != CurrentMtb[0].Direction) || (Generic_ADCS_AppData.MtbPctOnCmd.PercentOn_0 != CurrentMtb[0].PercentOn) || 
        (Generic_ADCS_AppData.MtbPctOnCmd.Direction_1 != CurrentMtb[1].Direction) || (Generic_ADCS_AppData.MtbPctOnCmd.PercentOn_1 != CurrentMtb[1].PercentOn) || 
        (Generic_ADCS_AppData.MtbPctOnCmd.Direction_2 != CurrentMtb[2].Direction) || (Generic_ADCS_AppData.MtbPctOnCmd.PercentOn_2 != CurrentMtb[2].PercentOn)) {
        CFE_SB_TimeStampMsg((CFE_SB_Msg_t *)&Generic_ADCS_AppData.MtbPctOnCmd);
        CFE_SB_SendMsg((CFE_SB_Msg_t *)&Generic_ADCS_AppData.MtbPctOnCmd);
    }
}

static void mcmd_to_percent_direction(double Mcmd, uint8 *percent, uint8 *direction)
{
    double pct = 100.0 * Mcmd / Generic_ADCS_AppData.GNCPacket.Payload.MaxMcmd;
    *direction = 1;
    if (pct < 0) {
        pct *= -1.0;
        *direction = 0;
    }
    if (pct > 100) pct = 100;
    *percent = pct;
}