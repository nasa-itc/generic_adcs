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
#define GENERIC_ADCS_NOOP_CC                    0
#define GENERIC_ADCS_RESET_COUNTERS_CC          1
#define GENERIC_ADCS_SET_MODE_CC                2
#define GENERIC_ADCS_SEND_DI_CMD_CC             3
#define GENERIC_ADCS_SEND_AD_CMD_CC             4
#define GENERIC_ADCS_SEND_GNC_CMD_CC            5
#define GENERIC_ADCS_SEND_AC_CMD_CC             6
#define GENERIC_ADCS_SEND_DO_CMD_CC             7
#define GENERIC_ADCS_SET_MOMENTUM_MANAGEMENT_CC 8
#define GENERIC_ADCS_INERTIAL_QUATERNION_CC     9
#define GENERIC_ADCS_TWO_AXIS_PARAMS_CC         10

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
    CFE_MSG_CommandHeader_t CmdHeader;

} Generic_ADCS_NoArgs_cmd_t;

typedef struct
{
    /* Every command requires a header used to identify it */
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8    Mode;
} Generic_ADCS_Mode_cmd_t;

typedef struct
{
    /* Every command requires a header used to identify it */
    CFE_MSG_CommandHeader_t CmdHeader;
    double                  qbn[4];
} Generic_ADCS_Quat_Cmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8    MomentumManagement;
} Generic_ADCS_MomentumManagement_cmd_t;

typedef struct
{
    /* Every command requires a header used to identify it */
    CFE_MSG_CommandHeader_t CmdHeader;
    double                 primaryBody[3];
    double                 secondaryBody[3];
    uint8                  targetPrimary;
    uint8                  targetSecondary;
} Generic_ADCS_Two_Axis_Params_Cmd_t;

/*
** Generic_ADCS housekeeping type definition
*/
typedef struct 
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint8   CommandErrorCount;
    uint8   CommandCount;
} __attribute__((packed)) Generic_ADCS_Hk_tlm_t;
#define GENERIC_ADCS_HK_TLM_LNGTH sizeof ( Generic_ADCS_Hk_tlm_t )

/*
** Generic_ADCS DI type definition
*/
typedef struct
{
    double qbs[4]; // quaternion from sensor to body
    double bvb[3]; // magnetic field measurement by sensor in body frame
} __attribute__((packed)) Generic_ADCS_DI_Mag_Tlm_Payload_t;

typedef struct
{
    double qbs[4]; // quaternion from sensor to body
    uint8  valid;
    double svb[3]; // sun vector from sensor in body frame
} __attribute__((packed)) Generic_ADCS_DI_Fss_Tlm_Payload_t;

typedef struct
{
    double axis[3]; // CSS axis in body frame
    double scale;   // scale factor
    double percenton;
} __attribute__((packed)) Generic_ADCS_DI_Css_Sensor_Payload_t;

typedef struct
{
    Generic_ADCS_DI_Css_Sensor_Payload_t Sensor[6];
    uint8 valid;
    double svb[3]; // sun vector from sensors in body frame
} __attribute__((packed)) Generic_ADCS_DI_Css_Tlm_Payload_t;

typedef struct
{
    double qbs[4]; // quaternion from sensor to body
    double pos[3]; // position of sensor in body
    uint8  valid;
    double wbn[3]; // angular rate
    double acc[3]; // acceleration
} __attribute__((packed)) Generic_ADCS_DI_Imu_Tlm_Payload_t;

typedef struct
{
    double whl_axis[3][3];
    double H_maxB[3];
    double HwhlB[3];
} __attribute__((packed)) Generic_ADCS_DI_Rw_Tlm_Payload_t;

typedef struct
{
    double qbs[4]; // quaternion from sensor to body
    double q[4];
    uint8   valid;
} __attribute__((packed)) Generic_ADCS_DI_St_Tlm_Payload_t;

typedef struct
{
    double PosN[3];
    double VelN[3];
} __attribute__((packed)) Generic_ADCS_DI_Gps_Tlm_Payload_t;

typedef struct
{
    Generic_ADCS_DI_Mag_Tlm_Payload_t Mag;
    Generic_ADCS_DI_Fss_Tlm_Payload_t Fss;
    Generic_ADCS_DI_Css_Tlm_Payload_t Css;
    Generic_ADCS_DI_Imu_Tlm_Payload_t Imu;
    Generic_ADCS_DI_Rw_Tlm_Payload_t  Rw;
    Generic_ADCS_DI_St_Tlm_Payload_t  St;
    Generic_ADCS_DI_Gps_Tlm_Payload_t Gps;
} __attribute__((packed)) Generic_ADCS_DI_Tlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    Generic_ADCS_DI_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_DI_Tlm_t;
#define GENERIC_ADCS_DI_LNGTH sizeof ( Generic_ADCS_DI_Tlm_t )

/*
** Generic_ADCS AD type definition
*/
typedef struct 
{
    double bvb[3];
} __attribute__((packed)) Generic_ADCS_AD_Mag_Tlm_Payload_t;

typedef struct
{
    uint8 SunValid;
    uint8 FssValid;
    double svb[3];
    double svn[3];
} __attribute__((packed)) Generic_ADCS_AD_Sol_Tlm_Payload_t;

typedef struct
{
    uint8  init;
    double alpha;
    uint8  valid;
    double wbn_prev[3];
    double wbn[3];
    double acc[3];
} __attribute__((packed)) Generic_ADCS_AD_Imu_Tlm_Payload_t;

typedef struct
{
    double qbn[4];
} __attribute__((packed)) Generic_ADCS_AD_ST_Tlm_Payload_t;

typedef struct
{
    double PosN[3];
    double VelN[3];
} __attribute__((packed)) Generic_ADCS_AD_Gps_Tlm_Payload_t;

typedef struct
{
    Generic_ADCS_AD_Mag_Tlm_Payload_t Mag;
    Generic_ADCS_AD_Sol_Tlm_Payload_t Sol;
    Generic_ADCS_AD_Imu_Tlm_Payload_t Imu;
    Generic_ADCS_AD_ST_Tlm_Payload_t  St;
    Generic_ADCS_AD_Gps_Tlm_Payload_t Gps;
} __attribute__((packed)) Generic_ADCS_AD_Tlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    Generic_ADCS_AD_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_AD_Tlm_t;
#define GENERIC_ADCS_AD_LNGTH sizeof ( Generic_ADCS_AD_Tlm_t )

/*
** Generic_ADCS GNC type definition
*/
typedef struct {
    double Kb;
    double b_range;
    double loFrac;
    double hiFrac;
    uint8  mm_active[3];
    double Mcmd[3];
} __attribute__((packed)) Generic_ADCS_GNC_Hmgmt_t;

enum TargetVec
{
   POS,
   VEL,
   H,
   SUN_LOS,
   FLOATING
};

struct GNCCmdType {
   /*~ Parameters ~*/

   long init;

   double qbn[4];  /*Inertial to body quaternion command*/
   double qbl[4];  /*LVLH to body quaternon command*/
   double wbn[3];  /*Angular veloicty command*/

   double Kp[3]; /* Proportional gain */
   double Kr[3]; /* Derivative gain */
   double Ki[3]; /* Integral gain */
   double Kb;    /* Magnetic gain */
   double Kprec;
   double Knute;
   double Kspin;
   double kwhl;

   double spinRate;
   double Hlmt;

   double Hmax[3];
   double Tmax[3];

   double vmax;
   double b_range;
   double phiErr_max;
   double threshold_angle;
   long whl_preserve_direction;
   long mtb_preserve_direction;
   long h_mgmt_enable;

   enum TargetVec targetPrimary;
   enum TargetVec targetSecondary;
   double bodyPrimaryVec[3];
   double bodySecondaryVec[3];

   double primary_axis_body[3];   /*Primary Body axis for Primary Target per mode*/
   double secondary_axis_body[3]; /*Secondary Body axis for Secondary Target per mode */
   double primary_axis_target[3];   /*Primary Body axis for Primary Target per mode*/
   double secondary_axis_target[3]; /*Secondary Body axis for Secondary Target per mode */
   int primary_ID;                  /*Primary Target ID*/
   int secondary_ID;               /*Secondary Target ID */

   double dt;
   double delta;
   double tg;

} __attribute__((packed));

typedef struct {
    double DT;
    double MaxMcmd;
    uint8  Mode;
    uint8  HmgmtOn;
    Generic_ADCS_GNC_Hmgmt_t Hmgmt;
    double bvb[3];
    double svn[3];
    double svb[3];
    uint8  SunValid;
    double PosN[3];
    double VelN[3];
    double wbn[3];
    double HwhlMaxB[3];
    double HwhlB[3];
    double Mcmd[3];
    double Tcmd[3];
    double qbn[4];
    double qErr[4];
    double qbn_init[4];
    struct GNCCmdType Cmd[5];
} __attribute__((packed)) Generic_ADCS_GNC_Tlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    Generic_ADCS_GNC_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_GNC_Tlm_t;
#define GENERIC_ADCS_GNC_LNGTH sizeof ( Generic_ADCS_GNC_Tlm_t )

/*
** Generic_ADCS AC type definition
*/
typedef struct {
    double b_range;
    double Kb;
    double bold[3];
    double bdot[3];
} __attribute__((packed)) Generic_ADCS_AC_Bdot_Tlm_t;

typedef struct {
   /* Inputs*/
   double Kp[3];
   double Kr[3];
   double sside[3];
   double vmax;
   double cmd_wbn[3];
   uint8 h_mgmt;

   /* Internal Variables */
   double therr[3];
   double werr[3];
   double Tcmd[3];
   double err_t;
} __attribute__((packed)) Generic_ADCS_AC_Sunsafe_Tlm_t;

typedef struct {
   /* Inputs*/
   double Kp[3];
   double Kr[3];
   double Ki[3];
   double phiErr_max;
   double qbn_cmd[4];
   uint8 h_mgmt;

   /* Internal Variables */
   double therr[3];
   double sumtherr[3];
   double qErr[4];
   double werr[3];
   double Tcmd[3];
} __attribute__((packed)) Generic_ADCS_AC_inertialType;

typedef struct {
   /* Inputs*/
   double Kp[3];
   double Kr[3];
   double Ki[3];
   double threshold_angle;
   double phiErr_max;
   long whl_preserve_direction;
   long h_mgmt;

   enum TargetVec targetPrimary;
   enum TargetVec targetSecondary;

   /* Internal Variables */
   double qbn_cmd[4];
   double primaryTarget[3]; /*Primary Target Vector*/
   double secondaryTarget[3]; /*Secondary Target Vector*/
   double primaryBody[3]; /*Primary Target Vector*/
   double secondaryBody[3]; /*Secondary Target Vector*/
   double wbn_cmd[3];
   double qErr[4];
   double werr[3];
   double Tcmd[3];
   double therr[3];
   double sumtherr[3];
} __attribute__((packed)) Generic_ADCS_AC_TwoAxisType;

typedef struct {
    Generic_ADCS_AC_Bdot_Tlm_t    Bdot;
    Generic_ADCS_AC_Sunsafe_Tlm_t Sunsafe;
    Generic_ADCS_AC_inertialType  Inertial;
    Generic_ADCS_AC_TwoAxisType   TwoAxis;
} __attribute__((packed)) Generic_ADCS_AC_Tlm_Payload_t;

typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;
    Generic_ADCS_AC_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_AC_Tlm_t;
#define GENERIC_ADCS_AC_LNGTH sizeof ( Generic_ADCS_AC_Tlm_t )

/*
** Generic_ADCS DO type definition
*/
typedef struct
{
    double qba[4]; // quaternion from actuator to body
    double Mcmd[3];
} __attribute__((packed)) Generic_ADCS_DO_Trq_TlmPayload_t;

typedef struct
{
    double axis[3][3];
    double Tcmd[3];
} __attribute__((packed)) Generic_ADCS_DO_Rw_TlmPayload_t;

typedef struct
{
    Generic_ADCS_DO_Trq_TlmPayload_t Trq;
    Generic_ADCS_DO_Rw_TlmPayload_t Rw;
} __attribute__((packed)) Generic_ADCS_DO_Tlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    Generic_ADCS_DO_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_DO_Tlm_t;
#define GENERIC_ADCS_DO_LNGTH sizeof ( Generic_ADCS_DO_Tlm_t )


#endif
