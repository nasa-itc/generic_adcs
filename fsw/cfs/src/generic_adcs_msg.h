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

/*
** Telemetry Request Command Codes
*/
#define GENERIC_ADCS_REQ_HK_TLM 0

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
    uint8                   Mode;
} Generic_ADCS_Mode_cmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8                   MomentumManagement;
} Generic_ADCS_MomentumManagement_cmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    double                  qbn[4];
} __attribute__((packed)) Generic_ADCS_Quat_cmd_t;

/*
** Generic_ADCS housekeeping type definition
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint8                     CommandErrorCount;
    uint8                     CommandCount;
} __attribute__((packed)) Generic_ADCS_Hk_tlm_t;
#define GENERIC_ADCS_HK_TLM_LNGTH sizeof(Generic_ADCS_Hk_tlm_t)

/*
** Generic_ADCS EPH type definition
*/
typedef struct
{
   double date_epoch;
   double coeff_G1;
   double coeff_G2;
   double coeff_L1;
   double coeff_l2;
   double coeff_long1;
   double coeff_long2;
   double cos_obliq_eclp;
   double sin_obliq_eclp;
} __attribute__((packed)) Generic_ADCS_EPH_Sol_Tlm_Payload_t;

typedef struct 
{
   int nmax;
} __attribute__((packed)) Generic_ADCS_EPH_Mag_Tlm_Payload_t;

typedef struct
{
    Generic_ADCS_EPH_Sol_Tlm_Payload_t Sol;
    Generic_ADCS_EPH_Mag_Tlm_Payload_t bfld;
} __attribute__((packed)) Generic_ADCS_EPH_Tlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t     TlmHeader;
    Generic_ADCS_EPH_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_EPH_Tlm_t;
#define GENERIC_ADCS_EPH_LNGTH sizeof(Generic_ADCS_EPH_Tlm_t)

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
    uint8                                valid;
    double                               svb[3]; // sun vector from sensors in body frame
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
    uint8  valid;
} __attribute__((packed)) Generic_ADCS_DI_St_Tlm_Payload_t;

typedef struct
{
    uint16_t Weeks;
    uint32_t SecondsIntoWeek;
    double   Fractions;
    double   ECEFX;
    double   ECEFY;
    double   ECEFZ;
    double   VelX;
    double   VelY;
    double   VelZ;
    float    lat;
    float    lon;
    float    alt;
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
    CFE_MSG_TelemetryHeader_t     TlmHeader;
    Generic_ADCS_DI_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_DI_Tlm_t;
#define GENERIC_ADCS_DI_LNGTH sizeof(Generic_ADCS_DI_Tlm_t)

/*
** Generic_ADCS AD type definition
*/
typedef struct
{
    double bvb[3];
    uint8  MagValid;
} __attribute__((packed)) Generic_ADCS_AD_Mag_Tlm_Payload_t;

typedef struct
{
    uint8  SunValid;
    uint8  FssValid;
    double svb[3];
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
    uint8  Valid;  /* [-] data validity flag */
    double qbn[4]; /* [-] quaternion expressed in body frame */
} __attribute__((packed)) Generic_ADCS_AD_ST_Tlm_Payload_t;

typedef struct
{
    uint16_t Weeks;
    uint32_t SecondsIntoWeek;
    double   Fractions;
    double   ECEFX;
    double   ECEFY;
    double   ECEFZ;
    double   VelX;
    double   VelY;
    double   VelZ;
    float    lat;
    float    lon;
    float    alt;
} __attribute__((packed)) Generic_ADCS_AD_Gps_Tlm_Payload_t;

typedef struct 
{
   uint8 Valid;
   uint8 enable_filter;    /*Flag to enable/disable Moving Average filter*/
   uint8 SolInit;
   uint8 MagInit;
   double wbn[3];
   double ws[3];   /*(rad/s) Estimated angular rate from Sun Vector*/
   double wm[3];    /*(rad/s) Estimated angular rate from Mag Vector*/
   double svb_prev[3];  /*sol.svb at prevous time*/
   double bvb_prev[3];  /*mag.bvb unit at last time step*/
   int32  sample_size;    /*Number of samples used in moving average filter*/
} __attribute__((packed)) Generic_AD_rateEst_Tlm_Payload_t;

typedef struct
{
    Generic_ADCS_AD_Mag_Tlm_Payload_t Mag;
    Generic_ADCS_AD_Sol_Tlm_Payload_t Sol;
    Generic_ADCS_AD_Imu_Tlm_Payload_t Imu;
    Generic_ADCS_AD_ST_Tlm_Payload_t  ST;
    Generic_ADCS_AD_Gps_Tlm_Payload_t Gps;
    Generic_AD_rateEst_Tlm_Payload_t  RateEst;
} __attribute__((packed)) Generic_ADCS_AD_Tlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t     TlmHeader;
    Generic_ADCS_AD_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_AD_Tlm_t;
#define GENERIC_ADCS_AD_LNGTH sizeof(Generic_ADCS_AD_Tlm_t)

/*
** Generic_ADCS GNC type definition
*/
typedef struct
{
    double Kb;
    double b_range;
    double loFrac;
    double hiFrac;
    uint8  mm_active[3];
    double Mcmd[3];
} __attribute__((packed)) Generic_ADCS_GNC_Hmgmt_t;

typedef struct
{
    double                   DT;
    double                   MaxMcmd;
    uint8                    Mode;
    uint8                    HmgmtOn;
    Generic_ADCS_GNC_Hmgmt_t Hmgmt;
    double                   bvb[3];
    double                   svb[3];
    uint8                    SunValid;
    double                   wbn[3];
    double                   HwhlMaxB[3];
    double                   HwhlB[3];
    double                   Mcmd[3];
    double                   Tcmd[3];
    uint8                    qValid;
    double                   qbn[4];
    double                   qErr[4];
    double                   Bfield_ECIF[3];
    double                   Bfield_ECEF[3];
    double                   Bfield_NED[3];
    double                   svn[3];
    double                   beta;
} __attribute__((packed)) Generic_ADCS_GNC_Tlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t      TlmHeader;
    Generic_ADCS_GNC_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_GNC_Tlm_t;
#define GENERIC_ADCS_GNC_LNGTH sizeof(Generic_ADCS_GNC_Tlm_t)

/*
** Generic_ADCS AC type definition
*/
typedef struct
{
    double b_range;
    double Kb;
    double bold[3];
    double bdot[3];
} __attribute__((packed)) Generic_ADCS_AC_Bdot_Tlm_t;

typedef struct
{
    /* Inputs*/
    double Kp[3];
    double Kr[3];
    double sside[3];
    double vmax;
    double cmd_wbn[3];
    uint8  h_mgmt;

    /* Internal Variables */
    double therr[3];
    double werr[3];
    double Tcmd[3];
    double err_t;
} __attribute__((packed)) Generic_ADCS_AC_Sunsafe_Tlm_t;

typedef struct
{
    /* Inputs*/
    double Kp[3];
    double Kr[3];
    double Ki[3];
    double phiErr_max;
    double qbn_cmd[4];
    long   h_mgmt;

    /* Internal Variables */
    double therr[3];
    double sumtherr[3];
    double qErr[4];
    double werr[3];
    double Tcmd[3];
} __attribute__((packed)) Generic_ADCS_AC_Inertial_Tlm_t;

typedef struct
{
    Generic_ADCS_AC_Bdot_Tlm_t     Bdot;
    Generic_ADCS_AC_Sunsafe_Tlm_t  Sunsafe;
    Generic_ADCS_AC_Inertial_Tlm_t Inertial;
} __attribute__((packed)) Generic_ADCS_AC_Tlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t     TlmHeader;
    Generic_ADCS_AC_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_AC_Tlm_t;
#define GENERIC_ADCS_AC_LNGTH sizeof(Generic_ADCS_AC_Tlm_t)

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
    Generic_ADCS_DO_Rw_TlmPayload_t  Rw;
} __attribute__((packed)) Generic_ADCS_DO_Tlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t     TlmHeader;
    Generic_ADCS_DO_Tlm_Payload_t Payload;
} __attribute__((packed)) Generic_ADCS_DO_Tlm_t;
#define GENERIC_ADCS_DO_LNGTH sizeof(Generic_ADCS_DO_Tlm_t)

#endif
