/*******************************************************************************
** Purpose:
**   This file contains the source code for the attitude determination and
**   attitude control routines of the Generic ADCS application.
**
*******************************************************************************/

#include <stdio.h>
#include <math.h>
#include "generic_adcs_app.h"
#include "generic_adcs_utilities.h"
#include "generic_adcs_adac.h"

static void AD_imu(const Generic_ADCS_DI_Imu_Tlm_Payload_t *DI_IMU, Generic_ADCS_AD_Imu_Tlm_Payload_t *AD_IMU);
static void AD_mag(const Generic_ADCS_DI_Mag_Tlm_Payload_t *DI_Mag, Generic_ADCS_AD_Mag_Tlm_Payload_t *AD_Mag);
static void AD_st(const Generic_ADCS_DI_St_Tlm_Payload_t *DI_ST, Generic_ADCS_AD_ST_Tlm_Payload_t *AD_ST);
static void AD_sol(const Generic_ADCS_DI_Fss_Tlm_Payload_t *DI_FSS, const Generic_ADCS_DI_Css_Tlm_Payload_t *DI_CSS, 
    Generic_ADCS_AD_Sol_Tlm_Payload_t *AD_Sol);
static void AD_gps(const Generic_ADCS_DI_Gps_Tlm_Payload_t *DI_Gps, Generic_ADCS_AD_Gps_Tlm_Payload_t *AD_Gps);
static void AD_to_GNC(const Generic_ADCS_AD_Tlm_Payload_t *AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC);
static void AC_bdot(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Bdot_Tlm_t *AC_bdot);
static void AC_sunsafe(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Sunsafe_Tlm_t *ACS);
static void AC_h_mgmt(Generic_ADCS_GNC_Tlm_Payload_t *GNC);
static void AC_inertial(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_inertialType *ACS);
static void AC_twoaxis(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_TwoAxisType *ACS);

void Generic_ADCS_init_attitude_determination_and_attitude_control(FILE *in, Generic_ADCS_AD_Tlm_Payload_t *AD, 
    Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Tlm_Payload_t *ACS)
{
    char junk[512], newline;
    // AD
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf%[^\n]%[\n]", &AD->Imu.alpha, junk, &newline);
    AD->Imu.init = 0;
    // GNC
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf%[^\n]%[\n]", &GNC->DT, junk, &newline);
    fscanf(in, "%lf%[^\n]%[\n]", &GNC->MaxMcmd, junk, &newline);
    // AC Bdot
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf%[^\n]%[\n]", &ACS->Bdot.b_range, &ACS->Bdot.Kb, junk, &newline);
    // AC Sunsafe
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf %lf %lf%[^\n]%[\n]", &ACS->Sunsafe.Kp[0], &ACS->Sunsafe.Kp[1], &ACS->Sunsafe.Kp[2], 
        &ACS->Sunsafe.Kr[0], &ACS->Sunsafe.Kr[1], &ACS->Sunsafe.Kr[2], junk, &newline);
    fscanf(in, "%lf %lf %lf %lf %lf %lf %lf%[^\n]%[\n]", &ACS->Sunsafe.sside[0], &ACS->Sunsafe.sside[1], &ACS->Sunsafe.sside[2], &ACS->Sunsafe.vmax, 
        &ACS->Sunsafe.cmd_wbn[0], &ACS->Sunsafe.cmd_wbn[1], &ACS->Sunsafe.cmd_wbn[2], junk, &newline);
    for (int i = 0; i < 3; i++) {
        ACS->Sunsafe.therr[i] = ACS->Sunsafe.werr[i] = ACS->Sunsafe.Tcmd[i] = 0;
    }
    // AC Inertial
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf %lf %c%[^\n]%[\n]", 
        &Generic_ADCS_AppData.inertial_qbn[0], &Generic_ADCS_AppData.inertial_qbn[1], &Generic_ADCS_AppData.inertial_qbn[2], &Generic_ADCS_AppData.inertial_qbn[3], 
        &ACS->Inertial.phiErr_max, &ACS->Inertial.h_mgmt, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf %lf %lf %lf %lf %lf%[^\n]%[\n]", &ACS->Inertial.Kp[0], &ACS->Inertial.Kp[1], &ACS->Inertial.Kp[2], &ACS->Inertial.Kr[0], &ACS->Inertial.Kr[1], &ACS->Inertial.Kr[2], 
        &ACS->Inertial.Ki[0], &ACS->Inertial.Ki[1], &ACS->Inertial.Ki[2], junk, &newline );
    // AC Two Axis
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %ld %lf %lf %lf %lf %lf %lf %lf %lf %lf%[^\n]%[\n]",
        &ACS->TwoAxis.threshold_angle, &ACS->TwoAxis.phiErr_max, &ACS->TwoAxis.whl_preserve_direction, 
        &ACS->TwoAxis.Kp[0], &ACS->TwoAxis.Kp[1], &ACS->TwoAxis.Kp[2], 
        &ACS->TwoAxis.Kr[0], &ACS->TwoAxis.Kr[1], &ACS->TwoAxis.Kr[2], 
        &ACS->TwoAxis.Ki[0], &ACS->TwoAxis.Ki[1], &ACS->TwoAxis.Ki[2], junk, &newline);
    ACS->TwoAxis.targetPrimary = POS;
    ACS->TwoAxis.targetSecondary = SUN_LOS;
    ACS->TwoAxis.primaryBody[0] = 1.0;
    ACS->TwoAxis.primaryBody[1] = 0.0;
    ACS->TwoAxis.primaryBody[2] = 0.0;
    ACS->TwoAxis.secondaryBody[0] = 0.0;
    ACS->TwoAxis.secondaryBody[1] = 1.0;
    ACS->TwoAxis.secondaryBody[2] = 0.0;
    // AC Momentum management
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &GNC->Hmgmt.Kb, &GNC->Hmgmt.b_range, &GNC->Hmgmt.loFrac, &GNC->Hmgmt.hiFrac, junk, &newline);
}

void Generic_ADCS_execute_attitude_determination_and_attitude_control(const Generic_ADCS_DI_Tlm_Payload_t *DI, Generic_ADCS_AD_Tlm_Payload_t *AD, 
    Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Tlm_Payload_t *ACS)
{
    AD_imu(&DI->Imu, &AD->Imu);
    AD_mag(&DI->Mag, &AD->Mag);
    AD_st(&DI->St, &AD->St);
    AD_sol(&DI->Fss, &DI->Css, &AD->Sol);
    AD_gps(&DI->Gps, &AD->Gps);

    AD_to_GNC(AD, GNC);
    for (int i = 0; i < 3; i++) GNC->HwhlB[i] = DI->Rw.HwhlB[i];
    for (int i = 0; i < 3; i++) GNC->HwhlMaxB[i] = DI->Rw.H_maxB[i];

    switch(GNC->Mode) {
    case BDOT_MODE:
        AC_bdot(GNC, &ACS->Bdot);
        break;
    
    case SUNSAFE_MODE:
        AC_sunsafe(GNC, &ACS->Sunsafe);
        break;

    case INERTIAL_MODE:
        for (int i = 0; i < 4; i++) ACS->Inertial.qbn_cmd[i] = Generic_ADCS_AppData.inertial_qbn[i];
        AC_inertial(GNC, &ACS->Inertial);
        break;

    case TWO_AXIS_MODE:
        AC_twoaxis(GNC, &ACS->TwoAxis);
        break;

    case PASSIVE_MODE:
    default:
        for (int i = 0; i < 3; i++) {
            GNC->Mcmd[i] = 0.0;
            GNC->Tcmd[i] = 0.0;
        }
        break;
    }
}

static void AD_imu(const Generic_ADCS_DI_Imu_Tlm_Payload_t *DI_IMU, Generic_ADCS_AD_Imu_Tlm_Payload_t *AD_IMU)
{
    if (DI_IMU->valid) {
        AD_IMU->valid = 1;
        for (int i = 0; i < 3; i++) {
            AD_IMU->acc[i] = DI_IMU->acc[i];
        }

        if (AD_IMU->init == 0) {
            for (int i = 0; i < 3; i++) {
                AD_IMU->wbn[i] = DI_IMU->wbn[i];
            }
            AD_IMU->init = 1;
        } else {
            for (int i = 0; i < 3; i++) {
                AD_IMU->wbn[i] = AD_IMU->alpha * AD_IMU->wbn_prev[i] + (1 - AD_IMU->alpha) * DI_IMU->wbn[i];
            }
        }
        for (int i = 0; i < 3; i++) {
            AD_IMU->wbn_prev[i] = AD_IMU->wbn[i];
        }
    } else {
        AD_IMU->valid = 0;
    }
}

static void AD_mag(const Generic_ADCS_DI_Mag_Tlm_Payload_t *DI_Mag, Generic_ADCS_AD_Mag_Tlm_Payload_t *AD_Mag)
{
    /* AD very simple for magnetometer... there is only one mag and no fusion with anything else */
    for (int i = 0; i < 3; i++) {
        AD_Mag->bvb[i] = DI_Mag->bvb[i];
    }
}

static void AD_st(const Generic_ADCS_DI_St_Tlm_Payload_t *DI_ST, Generic_ADCS_AD_ST_Tlm_Payload_t *AD_ST)
{
    for (int i = 0; i < 4; i++) AD_ST->qbn[i] = DI_ST->q[i];
}

static void AD_sol(const Generic_ADCS_DI_Fss_Tlm_Payload_t *DI_Fss, const Generic_ADCS_DI_Css_Tlm_Payload_t *DI_Css, Generic_ADCS_AD_Sol_Tlm_Payload_t *AD_Sol)
{
    if (DI_Fss->valid == 1) {
        AD_Sol->SunValid = 1;
        AD_Sol->FssValid = 1;
        AD_Sol->svb[0] = DI_Fss->svb[0];
        AD_Sol->svb[1] = DI_Fss->svb[1];
        AD_Sol->svb[2] = DI_Fss->svb[2];
    } else if (DI_Css->valid == 1) {
        AD_Sol->SunValid = 1;
        AD_Sol->FssValid = 0;
        AD_Sol->svb[0] = DI_Css->svb[0];
        AD_Sol->svb[1] = DI_Css->svb[1];
        AD_Sol->svb[2] = DI_Css->svb[2];
    } else {
        AD_Sol->SunValid = 0;
        AD_Sol->FssValid = 0;
        AD_Sol->svb[0] = 0.0;
        AD_Sol->svb[1] = 0.0;
        AD_Sol->svb[2] = 0.0;
    }
}

static void AD_gps(const Generic_ADCS_DI_Gps_Tlm_Payload_t *DI_Gps, Generic_ADCS_AD_Gps_Tlm_Payload_t *AD_Gps)
{
    for (int i = 0; i < 3; i++) {
        AD_Gps->PosN[i] = DI_Gps->PosN[i];
        AD_Gps->VelN[i] = DI_Gps->VelN[i];
    }
}

static void AD_to_GNC(const Generic_ADCS_AD_Tlm_Payload_t *AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC)
{
    for (int i = 0; i < 3; i++) {
        GNC->bvb[i] = AD->Mag.bvb[i];
        GNC->svb[i] = AD->Sol.svb[i];
        GNC->wbn[i] = AD->Imu.wbn[i];
        GNC->qbn[i] = AD->St.qbn[i];
        GNC->PosN[i] = AD->Gps.PosN[i];
        GNC->VelN[i] = AD->Gps.VelN[i];
    }
    GNC->qbn[3] = AD->St.qbn[3];
    QxV(GNC->qbn, GNC->svb, GNC->svn);
    GNC->SunValid = AD->Sol.SunValid;
}

static void AC_bdot(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Bdot_Tlm_t *ACS)
{
    /* apply control only if b-field is in range */
    if (MAGV(GNC->bvb) > ACS->b_range) {
        for(int i = 0; i < 3; i++) {
            /* backward difference b-field derivative */
            ACS->bdot[i] = (GNC->bvb[i] - ACS->bold[i]) / GNC->DT;
            /* store old b-field */
            ACS->bold[i] = GNC->bvb[i];
            /* traditional b-dot algorithm */
            GNC->Mcmd[i] = -ACS->Kb * ACS->bdot[i] / MAGV(GNC->bvb);
            /* ensure wheels disabled */
            GNC->Tcmd[i] = 0.0;
        }
    } else {
        for (int i = 0; i < 3; i++) {
            GNC->Mcmd[i] = 0.0;
            GNC->Tcmd[i] = 0.0;
        }
    }
}

#define EPS 1.0E-6
static void AC_sunsafe(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Sunsafe_Tlm_t *ACS)
{
   int i;
   double u1[3] = {0.0, 0.0, 0.0}, err_b[3] = {0.0, 0.0, 0.0};      /* angle error calculation parameteres */
   double temp_sside[3] = {0.0, 0.0, 0.0};
   double SoS = 0.0;

/* .. Check that SS Vector is valid */
   if (GNC->SunValid) {

/* .. Form attitude error signals */
      SoS = VoV(GNC->svb, ACS->sside);
      if ((SoS > (EPS - 1.0)) && (SoS < (1.0 - EPS))) {
         VxV(GNC->svb, ACS->sside, ACS->therr);
      }
      else if (SoS >= (1.0 - EPS)) {
         ACS->therr[0] = 0.0;
         ACS->therr[1] = 0.0;
         ACS->therr[2] = 0.0;
         }
      else {
         err_b[0] = ACS->sside[1];
         err_b[1] = ACS->sside[2];
         err_b[2] = ACS->sside[0];
         if (fabs(err_b[0] - err_b[1]) < EPS && fabs(err_b[0] - err_b[2]) < EPS) {
            err_b[0] = -err_b[0];
         }
         VxV(ACS->sside, err_b, temp_sside);
         VxV(GNC->svb, temp_sside, ACS->therr);
      }
      
/* .. Closed-loop attitude control - PD Method */
      for(i = 0; i < 3; i++) {
         /* Clip attitude slew rates */
         u1[i] = Limit(ACS->Kp[i] / ACS->Kr[i] * ACS->therr[i], -ACS->vmax,ACS->vmax);
         ACS->werr[i] = GNC->wbn[i] - ACS->cmd_wbn[i];
         ACS->Tcmd[i] = -ACS->Kr[i] * (u1[i] + ACS->werr[i]);
      }

/* .. Apply Torque Command */
      for(i = 0; i < 3; i++) {
         GNC->Tcmd[i] = -ACS->Tcmd[i];
      }
   }

   else { /* during eclipse, reduce attitude rates only */

      for(i = 0; i < 3; i++) {
         ACS->werr[i] = GNC->wbn[i];
         ACS->Tcmd[i] = -ACS->Kr[i]* ACS->werr[i];
      }
      /* .. Apply Torque Command  */
      for (i = 0; i < 3; i++) {
         GNC->Tcmd[i] = -ACS->Tcmd[i];
      }
   }

   if (GNC->HmgmtOn) {
      AC_h_mgmt(GNC);
      for(i = 0; i < 3; i++) {
        GNC->Mcmd[i] = GNC->Hmgmt.Mcmd[i];
      }
   }
   else {
      for(i = 0; i < 3; i++) {
         GNC->Mcmd[i] = 0.0;
      }
   }

}

static void AC_h_mgmt(Generic_ADCS_GNC_Tlm_Payload_t *GNC)
{

   double Herr[3] = {0.0, 0.0, 0.0};
   double bvb[3] = {0.0, 0.0, 0.0};
   double HxB[3] = {0.0, 0.0, 0.0};
   int i;

   if ( MAGV(GNC->bvb) > GNC->Hmgmt.b_range ) {
      /*Test if any axis needs to be momentum managed*/
      for(i=0;i<3;i++) {
         if (fabs(GNC->HwhlB[i]) > GNC->Hmgmt.hiFrac*fabs(GNC->HwhlMaxB[i])) {
            GNC->Hmgmt.mm_active[i] = 1;
         }
         if (fabs(GNC->HwhlB[i]) < GNC->Hmgmt.loFrac*fabs(GNC->HwhlMaxB[i])) {
            GNC->Hmgmt.mm_active[i] = 0;
         }
      }
      for(i = 0; i < 3; i++) {
         Herr[i] = 0.0;
         if (GNC->Hmgmt.mm_active[i] == 1) {
            Herr[i] = GNC->HwhlB[i];
         }
      }
      CopyUnitV(GNC->bvb, bvb);
      VxV(Herr,bvb,HxB);
      for(i = 0; i < 3; i++) {
         GNC->Hmgmt.Mcmd[i] = GNC->Hmgmt.Kb * HxB[i] / MAGV(GNC->bvb);
      }
   }
   else {
      for (i = 0; i < 3; i++) {
         GNC->Hmgmt.Mcmd[i] = 0.0;
      }
   }

}

static void AC_inertial(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_inertialType *ACS)
{
    int i;
    double qErrLimited[4] = {0.0, 0.0, 0.0, 0.0}; /* Initialize Error quaterion for internal use */
    double e_axis[3] = {0.0, 0.0, 0.0};           /* Initialze Eigen axis of the Body to Body quaternion*/
    double phiErr = 0.0;                          /* Intialize angular error of Body to Body quaternion */

    /*..Form attitude error signals */
    QxQT(ACS->qbn_cmd, GNC->qbn, ACS->qErr);

    /*..Unitize Quaternion Error */
    UNITQ(ACS->qErr);

    /*..Adopt shortest path */
    RECTIFYQ(ACS->qErr);

    for (i = 0; i < 4; i++)
    {
        GNC->qErr[i] = ACS->qErr[i];
    }

    /*..Limit B<-B quaterion Error */
    phiErr = 2.0 * arccos(ACS->qErr[3]);
    if (phiErr > ACS->phiErr_max)
    {
        phiErr = ACS->phiErr_max;
        e_axis[0] = ACS->qErr[0];
        e_axis[1] = ACS->qErr[1];
        e_axis[2] = ACS->qErr[2];
        UNITV(e_axis);
        for (i = 0; i < 3; i++)
        {
            qErrLimited[i] = e_axis[i] * sin(phiErr / 2.0);
        }
        qErrLimited[3] = cos(phiErr / 2.0);
    }
    else
    {
        for (i = 0; i < 4; i++)
        {
            qErrLimited[i] = ACS->qErr[i];
        }
    }

    /*..Compute attittude/rate errors, Apply PD Control Law and compute minimum Torque margin */
    for (i = 0; i < 3; i++)
    {
        ACS->therr[i] = 2.0 * qErrLimited[i];
        ACS->sumtherr[i] = ACS->sumtherr[i] + ACS->therr[i];
        ACS->werr[i] = -GNC->wbn[i];
        ACS->Tcmd[i] = ACS->Kp[i] * ACS->therr[i] + ACS->Kr[i] * ACS->werr[i] + ACS->Ki[i] * ACS->sumtherr[i];
    }

    for (i = 0; i < 3; i++)
    {
        GNC->Tcmd[i] = -ACS->Tcmd[i];
    }

    if (ACS->h_mgmt) {
      AC_h_mgmt(GNC);
      for(i = 0; i < 3; i++) {
          GNC->Mcmd[i] = GNC->Hmgmt.Mcmd[i];
      }
    }
    else {
      for(i = 0; i < 3; i++) {
          GNC->Mcmd[i] = 0.0;
      }
    }

}

static void check_collinearity(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_TwoAxisType *ACS)
{
    double tgt_angle = 0.0, bdy_angle = 0.0, primAngle = 0.0;
    double primTgt_body[3] = {0.0, 0.0, 0.0}, primTgt_body_init[3] = {0.0, 0.0, 0.0};

    bdy_angle = arccos(fabs(VoV(ACS->primaryBody, ACS->secondaryTarget)));
    tgt_angle = arccos(fabs(VoV(ACS->primaryTarget, ACS->secondaryTarget)));

    if (bdy_angle < EPS || MAGV(ACS->secondaryBody) < EPS)
    {
        /*If the primary and secondary body are collinear or secondarybody is CROSS_PRIMARY check the curren tangle
        between primary body and primary target*/
        QxV(GNC->qbn, ACS->primaryTarget, primTgt_body);
        UNITV(primTgt_body);
        primAngle = arccos(fabs(VoV(ACS->primaryBody, primTgt_body)));
        if (primAngle > EPS)
        {
            QxV(GNC->qbn_init, ACS->primaryTarget, primTgt_body_init);
            VxV(ACS->primaryBody, primTgt_body_init, ACS->secondaryBody);
            UNITV(ACS->secondaryBody);
        }
        else
        {
            if (arccos(fabs(ACS->primaryBody[1])) > EPS)
            {
                ACS->secondaryBody[0] = 0.0;
                ACS->secondaryBody[1] = 1.0;
                ACS->secondaryBody[2] = 0.0;
            }
            else
            {
                ACS->secondaryBody[0] = 1.0;
                ACS->secondaryBody[1] = 0.0;
                ACS->secondaryBody[2] = 0.0;
            }
        }

        QTxV(GNC->qbn, ACS->secondaryBody, ACS->secondaryTarget);
        UNITV(ACS->secondaryTarget);
    }
    else if (tgt_angle < ACS->threshold_angle)
    {
        QTxV(GNC->qbn, ACS->secondaryBody, ACS->secondaryTarget);
        UNITV(ACS->secondaryTarget);
    }
}
static void calc_qbn(Generic_ADCS_AC_TwoAxisType *ACS)
{
    int i;
    double tgtX_b[3] = {0.0, 0.0, 0.0}, tgtY_b[3] = {0.0, 0.0, 0.0}, tgtZ_b[3] = {0.0, 0.0, 0.0};
    double tgtX_n[3] = {0.0, 0.0, 0.0}, tgtY_n[3] = {0.0, 0.0, 0.0}, tgtZ_n[3] = {0.0, 0.0, 0.0};
    double C_tb[3][3] = {{0,0,0},{0,0,0},{0,0,0}}, C_tn[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double q_tb[4] = {0,0,0,1}, q_tn[4] = {0,0,0,1};

    for (i = 0; i < 3; i++)
    {
        tgtX_b[i] = ACS->primaryBody[i];
        tgtX_n[i] = ACS->primaryTarget[i];
    }
    VxV(ACS->primaryBody, ACS->secondaryBody, tgtZ_b);
    VxV(ACS->primaryTarget, ACS->secondaryTarget, tgtZ_n);
    VxV(tgtZ_b, tgtX_b, tgtY_b);
    VxV(tgtZ_n, tgtX_n, tgtY_n);

    UNITV(tgtX_b);
    UNITV(tgtY_b);
    UNITV(tgtZ_b);
    UNITV(tgtX_n);
    UNITV(tgtY_n);
    UNITV(tgtZ_n);

    /*construct body to target DCM and Inertial to Target DCMS*/
    for (i = 0; i < 3; i++)
    {
        C_tb[0][i] = tgtX_b[i];
        C_tb[1][i] = tgtY_b[i];
        C_tb[2][i] = tgtZ_b[i];
        C_tn[0][i] = tgtX_n[i];
        C_tn[1][i] = tgtY_n[i];
        C_tn[2][i] = tgtZ_n[i];
    }
    C2Q(C_tb, q_tb);
    C2Q(C_tn, q_tn);

    /* Calculate Inertial to Body Quaternion */
    QTxQ(q_tb, q_tn, ACS->qbn_cmd);
    UNITQ(ACS->qbn_cmd);
}

#define MU (3.986004418E14)        /* [m^2/s^2] Geocentric gravity constant */
static void setTargetVec(enum TargetVec targetVec_enum, Generic_ADCS_GNC_Tlm_Payload_t *GNC, double bodyVec[3], double targetVector[3], double rate[3])
{
    double h[3] = {0.0, 0.0, 0.0}, g[3] = {0.0, 0.0, 0.0};
    double rmag_sq = 0.0, rmag = 0.0, vmag_sq = 0.0;

    switch (targetVec_enum)
    {
    case POS:
        targetVector[0] = GNC->PosN[0];
        targetVector[1] = GNC->PosN[1];
        targetVector[2] = GNC->PosN[2];
        UNITV(targetVector);
        VxV(GNC->PosN, GNC->VelN, h);
        rmag_sq = VoV(GNC->PosN, GNC->PosN);
        rate[0] = h[0] / rmag_sq;
        rate[1] = h[1] / rmag_sq;
        rate[2] = h[2] / rmag_sq;
        break;
    case VEL:
        targetVector[0] = GNC->VelN[0];
        targetVector[1] = GNC->VelN[1];
        targetVector[2] = GNC->VelN[2];
        UNITV(targetVector);
        rmag = MAGV(GNC->PosN);
        g[0] = -MU * GNC->PosN[0] / (rmag * rmag * rmag);
        g[1] = -MU * GNC->PosN[1] / (rmag * rmag * rmag);
        g[2] = -MU * GNC->PosN[2] / (rmag * rmag * rmag);
        VxV(GNC->VelN, g, h);
        vmag_sq = VoV(GNC->VelN, GNC->VelN);
        rate[0] = h[0] / vmag_sq;
        rate[1] = h[1] / vmag_sq;
        rate[2] = h[2] / vmag_sq;
        break;
    case H: /*Orbit Angular Momentum Vector direction*/
        VxV(GNC->PosN, GNC->VelN, targetVector);
        UNITV(targetVector);
        rate[0] = 0.0;
        rate[1] = 0.0;
        rate[2] = 0.0;
        break;
    case SUN_LOS:
        targetVector[0] = GNC->svn[0];
        targetVector[1] = GNC->svn[1];
        targetVector[2] = GNC->svn[2];
        UNITV(targetVector);
        rate[0] = 0.0;
        rate[1] = 0.0;
        rate[2] = 0.0;
        break;
    case FLOATING:
        QTxV(GNC->qbn, bodyVec, targetVector);
        rate[0] = 0.0;
        rate[1] = 0.0;
        rate[2] = 0.0;
        break;
    default:
        fprintf(stderr, "%d is not a Valid Option for Target Vector Choice!\n", targetVec_enum);
        QTxV(GNC->qbn, bodyVec, targetVector);
        rate[0] = 0.0;
        rate[1] = 0.0;
        rate[2] = 0.0;
        break;
    }
}
static void AC_twoaxis(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_TwoAxisType *ACS)
{
    int i;
    double qErrLimited[4] = {0.0, 0.0, 0.0, 0.0}; /* Initialize Error quaterion for internal use */
    double e_axis[3] = {0.0, 0.0, 0.0};           /* Initialze Eigen axis of the Body to Body quaternion*/
    double phiErr = 0.0;                          /* Intialize angular error of Body to Body quaternion */
    double primaryRate[3] = {0.0, 0.0, 0.0};
    double secondaryRate[3] = {0.0, 0.0, 0.0};
    double wn_cmd[3] = {0.0, 0.0, 0.0}; /* Commanded interial angular velocity expressed in inertial frame*/
    double rateDotAxis = 0;

    /*Set Primary/Seconary Vectors*/
    setTargetVec(ACS->targetPrimary, GNC, ACS->primaryBody, ACS->primaryTarget, primaryRate);
    setTargetVec(ACS->targetSecondary, GNC, ACS->secondaryTarget, ACS->secondaryTarget, secondaryRate);


    /*check collinearity*/
    check_collinearity(GNC, ACS);

    /*calcualte commanded quaternon*/
    calc_qbn(ACS);

    /*calculate rate commands*/
    rateDotAxis = VoV(secondaryRate, ACS->primaryTarget);
    for (i = 0; i < 3; i++)
    {
        wn_cmd[i] = primaryRate[i] + rateDotAxis * ACS->primaryTarget[i];
    }
    QTxV(GNC->qbn, wn_cmd, ACS->wbn_cmd);

    /*..Form attitude error signals */
    QxQT(ACS->qbn_cmd, GNC->qbn, ACS->qErr);
    UNITQ(ACS->qErr);
    RECTIFYQ(ACS->qErr);

    for (i = 0; i < 4; i++)
    {
        GNC->qErr[i] = ACS->qErr[i];
    }

    /*..Limit B<-B quaterion Error */
    phiErr = 2.0 * arccos(ACS->qErr[3]);
    if (phiErr > ACS->phiErr_max)
    {
        phiErr = ACS->phiErr_max;
        e_axis[0] = ACS->qErr[0];
        e_axis[1] = ACS->qErr[1];
        e_axis[2] = ACS->qErr[2];
        UNITV(e_axis);
        for (i = 0; i < 3; i++)
        {
            qErrLimited[i] = e_axis[i] * sin(phiErr / 2.0);
        }
        qErrLimited[3] = cos(phiErr / 2.0);
    }
    else
    {
        for (i = 0; i < 4; i++)
        {
            qErrLimited[i] = ACS->qErr[i];
        }
    }

    /*..Compute attittude/rate errors, Apply PD Control Law and compute minimum Torque margin */
    for (i = 0; i < 3; i++)
    {
        ACS->therr[i] = 2.0 * qErrLimited[i];
        ACS->sumtherr[i] = ACS->sumtherr[i] + ACS->therr[i];
        ACS->werr[i] = ACS->wbn_cmd[i] - GNC->wbn[i];
        ACS->Tcmd[i] = ACS->Kp[i] * ACS->therr[i] + ACS->Kr[i] * ACS->werr[i] + ACS->Ki[i] * ACS->sumtherr[i];
    }
    /*..Apply limited torque to wheels*/
    for (i = 0; i < 3; i++)
    {
        GNC->Tcmd[i] = -ACS->Tcmd[i];
    }

    if (ACS->h_mgmt) {
      AC_h_mgmt(GNC);
      for(i = 0; i < 3; i++) {
          GNC->Mcmd[i] = GNC->Hmgmt.Mcmd[i];
      }
    }
    else {
      for(i = 0; i < 3; i++) {
          GNC->Mcmd[i] = 0.0;
      }
    }

}
