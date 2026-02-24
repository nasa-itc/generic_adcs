/*******************************************************************************
** Purpose:
**   This file contains the source code for the attitude determination and
**   attitude control routines of the Generic ADCS application.
**
*******************************************************************************/

#include <stdio.h>
#include <math.h>
#include "generic_adcs_msg.h"
#include "generic_adcs_utilities.h"
#include "generic_adcs_adac.h"
#include <stdbool.h>

static int  igrf(Generic_ADCS_EPH_Mag_Tlm_Payload_t *bfld, Generic_ADCS_DI_Gps_Tlm_Payload_t *DI_GPS, Generic_ADCS_GNC_Tlm_Payload_t *GNC);
static int32_t solar_ephemeris(Generic_ADCS_EPH_Sol_Tlm_Payload_t *sol, Generic_ADCS_DI_Gps_Tlm_Payload_t *DI_GPS, Generic_ADCS_GNC_Tlm_Payload_t *GNC);
static void AD_imu(const Generic_ADCS_DI_Imu_Tlm_Payload_t *DI_IMU, Generic_ADCS_AD_Imu_Tlm_Payload_t *AD_IMU);
static void AD_mag(const Generic_ADCS_DI_Mag_Tlm_Payload_t *DI_Mag, Generic_ADCS_AD_Mag_Tlm_Payload_t *AD_Mag);
static void AD_sol(const Generic_ADCS_DI_Fss_Tlm_Payload_t *DI_FSS, const Generic_ADCS_DI_Css_Tlm_Payload_t *DI_CSS,
                   Generic_ADCS_AD_Sol_Tlm_Payload_t *AD_Sol);
static void AD_st(const Generic_ADCS_DI_St_Tlm_Payload_t *DI_ST, Generic_ADCS_AD_ST_Tlm_Payload_t *AD_Mag);
static void AD_gps(const Generic_ADCS_DI_Gps_Tlm_Payload_t *DI_GPS, Generic_ADCS_AD_Gps_Tlm_Payload_t *gps);
static void AD_rateEst(Generic_ADCS_GNC_Tlm_Payload_t  GNC, Generic_ADCS_AD_Mag_Tlm_Payload_t mag, 
                       Generic_ADCS_AD_Sol_Tlm_Payload_t sol, Generic_AD_rateEst_Tlm_Payload_t *AD);
static void calc_wmag(double dt, Generic_ADCS_AD_Mag_Tlm_Payload_t mag, Generic_AD_rateEst_Tlm_Payload_t *AD );
static void calc_wsol(double dt, Generic_ADCS_AD_Sol_Tlm_Payload_t sol, Generic_AD_rateEst_Tlm_Payload_t *AD );
static void AD_to_GNC(const Generic_ADCS_AD_Tlm_Payload_t *AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC);
static void AC_bdot(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Bdot_Tlm_t *AC_bdot);
static void AC_sunsafe(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Sunsafe_Tlm_t *ACS);
static void AC_inertial(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Inertial_Tlm_t *ACS);
static void AC_h_mgmt(Generic_ADCS_GNC_Tlm_Payload_t *GNC);
static void AC_rw_momentum_dump(Generic_ADCS_GNC_Tlm_Payload_t *GNC);

void Generic_ADCS_init_attitude_determination_and_attitude_control(FILE *in, 
                                                                   Generic_ADCS_EPH_Tlm_Payload_t     *EPH,
                                                                   Generic_ADCS_AD_Tlm_Payload_t      *AD,
                                                                   Generic_ADCS_GNC_Tlm_Payload_t     *GNC,
                                                                   Generic_ADCS_AC_Tlm_Payload_t      *ACS)
{
    char junk[512], newline;
    // EPH
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf%[^\n]%[\n]", &EPH->Sol.date_epoch, junk, &newline);
    fscanf(in, "%lf %lf%[^\n]%[\n]", &EPH->Sol.coeff_G1, &EPH->Sol.coeff_G2, junk, &newline);
    fscanf(in, "%lf %lf%[^\n]%[\n]", &EPH->Sol.coeff_L1, &EPH->Sol.coeff_l2, junk, &newline);
    fscanf(in, "%lf %lf%[^\n]%[\n]", &EPH->Sol.coeff_long1, &EPH->Sol.coeff_long2, junk, &newline);
    fscanf(in, "%lf%[^\n]%[\n]", &EPH->Sol.cos_obliq_eclp, junk, &newline);
    fscanf(in, "%lf%[^\n]%[\n]", &EPH->Sol.sin_obliq_eclp, junk, &newline);
    fscanf(in, "%d%[^\n]%[\n]", &EPH->bfld.nmax, junk, &newline);
    // AD
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf%[^\n]%[\n]", &AD->Imu.alpha, junk, &newline);
    AD->Imu.init = 0;
    fscanf(in, "%hhu%[^\n]%[\n]", &AD->RateEst.enable_filter, junk, &newline);
    fscanf(in, "%d%[^\n]%[\n]", &AD->RateEst.sample_size, junk, &newline);
    AD->RateEst.Valid = false;
    AD->RateEst.MagInit = false;
    AD->RateEst.SolInit = false;
    for (int i = 0; i < 3; i++) {
        AD->RateEst.wbn[i] = 0.0;
        AD->RateEst.ws[i] = 0.0;
        AD->RateEst.wm[i] = 0.0;
        AD->RateEst.svb_prev[i] = 0.0;
        AD->RateEst.bvb_prev[i] = 0.0;
    }
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
    fscanf(in, "%lf %lf %lf %lf %lf %lf %lf%[^\n]%[\n]", &ACS->Sunsafe.sside[0], &ACS->Sunsafe.sside[1],
           &ACS->Sunsafe.sside[2], &ACS->Sunsafe.vmax, &ACS->Sunsafe.cmd_wbn[0], &ACS->Sunsafe.cmd_wbn[1],
           &ACS->Sunsafe.cmd_wbn[2], junk, &newline);
    for (int i = 0; i < 3; i++)
    {
        ACS->Sunsafe.therr[i] = ACS->Sunsafe.werr[i] = ACS->Sunsafe.Tcmd[i] = 0;
    }
    // AC inertial
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf %lf%[^\n]%[\n]", &ACS->Inertial.qbn_cmd[0], &ACS->Inertial.qbn_cmd[1],
           &ACS->Inertial.qbn_cmd[2], &ACS->Inertial.qbn_cmd[3], &ACS->Inertial.phiErr_max, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf %lf %lf%[^\n]%[\n]", &ACS->Inertial.Kp[0], &ACS->Inertial.Kp[1], &ACS->Inertial.Kp[2],
           &ACS->Inertial.Kr[0], &ACS->Inertial.Kr[1], &ACS->Inertial.Kr[2], junk, &newline);
    fscanf(in, "%lf %lf %lf%[^\n]%[\n]", &ACS->Inertial.Ki[0], &ACS->Inertial.Ki[1], &ACS->Inertial.Ki[2], junk,
           &newline);
    // AC Momentum management
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &GNC->Hmgmt.Kb, &GNC->Hmgmt.b_range, &GNC->Hmgmt.loFrac,
           &GNC->Hmgmt.hiFrac, junk, &newline);
}

void Generic_ADCS_execute_attitude_determination_and_attitude_control(const Generic_ADCS_DI_Tlm_Payload_t *DI,
                                                                      Generic_ADCS_EPH_Tlm_Payload_t      *EPH,
                                                                      Generic_ADCS_AD_Tlm_Payload_t       *AD,
                                                                      Generic_ADCS_GNC_Tlm_Payload_t      *GNC,
                                                                      Generic_ADCS_AC_Tlm_Payload_t       *ACS)
{
    igrf(&EPH->bfld, &DI->Gps, GNC);
    solar_ephemeris(&EPH->Sol, &DI->Gps, GNC);
    AD_imu(&DI->Imu, &AD->Imu);
    AD_mag(&DI->Mag, &AD->Mag);
    AD_st(&DI->St, &AD->ST);
    AD_gps(&DI->Gps, &AD->Gps);
    AD_sol(&DI->Fss, &DI->Css, &AD->Sol);
    AD_rateEst(*GNC, AD->Mag, AD->Sol, &AD->RateEst);

    AD_to_GNC(AD, GNC);
    for (int i = 0; i < 3; i++)
        GNC->HwhlB[i] = DI->Rw.HwhlB[i];
    for (int i = 0; i < 3; i++)
        GNC->HwhlMaxB[i] = DI->Rw.H_maxB[i];

    switch (GNC->Mode)
    {
        case BDOT_MODE:
            AC_bdot(GNC, &ACS->Bdot);
            AC_rw_momentum_dump(GNC);
            break;

        case SUNSAFE_MODE:
            AC_sunsafe(GNC, &ACS->Sunsafe);
            break;

        case INERTIAL_MODE:
            AC_inertial(GNC, &ACS->Inertial);
            break;

        case PASSIVE_MODE:
        default:
            for (int i = 0; i < 3; i++)
            {
                GNC->Mcmd[i] = 0.0;
                GNC->Tcmd[i] = 0.0;
            }
            break;
    }
}

#define MAXDEG 13
#define MAXCOEFF (MAXDEG*(MAXDEG+2))
#define RAD2DEG (180.0/M_PI)

const int IGRF_DATE = 2020;
const int IGRF_ORD = 13;
const int SV_ORD = 8;
const float igrf_coeffs[195] = {-29404.8,-1450.9,4652.5,-2499.6,2982,-2991.6,1677,-734.6,1363.2,-2381.2,-82.1,1236.2,241.9,525.7,-543.4,903,809.5,281.9,86.3,-158.4,-309.4,199.7,48,-349.7,-234.3,363.2,47.7,187.8,208.3,-140.7,-121.2,-151.2,32.3,13.5,98.9,66,65.5,-19.1,72.9,25.1,-121.5,52.8,-36.2,-64.5,13.5,8.9,-64.7,68.1,80.6,-76.7,-51.5,-8.2,-16.9,56.5,2.2,15.8,23.5,6.4,-2.2,-7.2,-27.2,9.8,-1.8,23.7,9.7,8.4,-17.6,-15.3,-0.5,12.8,-21.1,-11.7,15.3,14.9,13.7,3.6,-16.5,-6.9,-0.3,2.8,5,8.4,-23.4,2.9, 11,-1.5,9.8,-1.1,-5.1,-13.2,-6.3,1.1,7.8,8.8,0.4,-9.3,-1.4,-11.9,9.6,-1.9,-6.2,3.4,-0.1,-0.2,1.7,3.6,-0.9,4.8,0.7,-8.6,-0.9,-0.1,1.9,-4.3,1.4,-3.4,-2.4,-0.1,-3.8,-8.8,3,-1.4,0,-2.5,2.5,2.3,-0.6,-0.9,-0.4,0.3,0.6,-0.7,-0.2,-0.1,-1.7,1.4,-1.6,-0.6,-3,0.2,-2,3.1,-2.6,-2,-0.1,-1.2,0.5,0.5,1.3,1.4,-1.2,-1.8,0.7,0.1,0.3,0.8,0.5,-0.2,-0.3,0.6,-0.5,0.2,0.1,-0.9,-1.1,0,-0.3,0.5,0.1,-0.9,-0.9,0.5,0.6,0.7,1.4,-0.3,-0.4,0.8,-1.3,0,-0.1,0.8,0.3,0,-0.1,0.4,0.5,0.1,0.5,0.5,-0.4,-0.5,-0.4,-0.4,-0.6};
const float igrf_sv[80] = {5.7,7.4,-25.9,-11,-7,-30.2,-2.1,-22.4,2.2,-5.9,6,3.1,-1.1,-12,0.5,-1.2,-1.6,-0.1,-5.9,6.5,5.2,3.6,-5.1,-5,-0.3,0.5,0,-0.6,2.5,0.2,-0.6,1.3,3,0.9,0.3,-0.5,-0.3,0,0.4,-1.6,1.3,-1.3,-1.4,0.8,0,0,0.9,1,-0.1,-0.2,0.6,0,0.6,0.7,-0.8,0.1,-0.2,-0.5,-1.1,-0.8,0.1,0.8,0.3,0,0.1,-0.2,-0.1,0.6,0.4,-0.2,-0.1,0.5,0.4,-0.3,0.3,-0.4,-0.1,0.5,0.4};

float mag_coeff[MAXCOEFF];                   /*Computed coefficients*/

static int extrapsh(int date)
{
   int nmax;
   int k, l;
   int i;
   int igo = IGRF_ORD,svo = SV_ORD;
   float factor;
   /*# of years to extrapolate */
   factor = date - IGRF_DATE;
/*make shure that degree is smaller then MAXDEG */
   if(igo > MAXDEG) {
      igo = MAXDEG;
   }
   if(svo > MAXDEG) {
      svo = MAXDEG;
   }
   /*check for equal degree*/
   if (igo == svo) {
      k =  igo * (igo + 2);
      nmax = igo;
   }else{
/* check if reference is bigger */
      if (igo > svo) {
         k = svo * (svo + 2);
         l = igo * (igo + 2);
         /* copy extra elements unchanged */
         for ( i = k; i < l; ++i) {
            mag_coeff[i] = igrf_coeffs[i];
         }
         /*maximum degree of model */
         nmax = igo;
      }else{
         k = igo * (igo + 2);
         l = svo * (svo + 2);
         /*put in change for extra elements? */
         for(i = k; i < l; ++i) {
            mag_coeff[i] = factor * igrf_sv[i];
         }
         nmax = svo;
      }
   }
/*apply secular variations to model */
   for ( i = 0; i < k; ++i) {
      mag_coeff[i] = igrf_coeffs[i] + factor * igrf_sv[i];
   }
   /* return maximum degree of model and secular variations */
   return nmax;
}

#define PQ_BUFFSIZE         32

static int igrf(Generic_ADCS_EPH_Mag_Tlm_Payload_t *bfld, Generic_ADCS_DI_Gps_Tlm_Payload_t *DI_GPS, Generic_ADCS_GNC_Tlm_Payload_t *GNC)
{
   float slat;
   float clat;
   float ratio;
   float aa, bb, cc;
   float rr;
   float fm,fn;
   float sl[MAXDEG];
   float cl[MAXDEG];
   float p[PQ_BUFFSIZE];
   float q[PQ_BUFFSIZE];
   int i,j,k,l,m,n;
   int kw;
   int npq;
   float x,y,z;
   Matrix3x3f Dcm_NEDtoECEF, Dcm_ECEFtoECIF;
   Vector3f dest, Bfield_ECIF, Bfield_ECEF ;
   float PriMerAng;
   double GMST;
   float elev;

   double GpsTime = GpsDateToGpsTime(2, DI_GPS->Weeks, DI_GPS->SecondsIntoWeek); // rollover=2, valid April 7, 2019 to November 20, 2038
   double j2000 = GpsTime - 7300.5*86400 + (19+32.184);
   long year, month, day, hour, minute;
   double second;
   TimeToDate(j2000, &year, &month, &day, &hour, &minute, &second, 0.01);

   extrapsh(year);

   /* Prime Meridian Calculation */
   GMST = JD_TO_GMST(GpsTime_TO_JD(2, DI_GPS->Weeks, DI_GPS->SecondsIntoWeek)); // rollover=2, valid April 7, 2019 to November 20, 2038
   PriMerAng = TWOPI*GMST;

/*calculate sin and cos of latitude */
   slat = sin(DI_GPS->lat);
   clat = cos(DI_GPS->lat);
   /*prevent divide by zero */
   if(clat < EPS16) {
      clat = EPS16;
   }

   /*calculate sin and cos of longitude */
   sl[0] = sin(DI_GPS->lon);
   cl[0] = cos(DI_GPS->lon);

   /*initialize coordinates */
   x = 0;
   y = 0;
   z = 0;

   Dcm_NEDtoECEF.Comp[0][0] = -slat*cl[0];
   Dcm_NEDtoECEF.Comp[0][1] = -sl[0];
   Dcm_NEDtoECEF.Comp[0][2] = -clat*cl[0];
   Dcm_NEDtoECEF.Comp[1][0] = -slat*sl[0];
   Dcm_NEDtoECEF.Comp[1][1] = cl[0];
   Dcm_NEDtoECEF.Comp[1][2] = -clat*sl[0];
   Dcm_NEDtoECEF.Comp[2][0] = clat;
   Dcm_NEDtoECEF.Comp[2][1] = 0;
   Dcm_NEDtoECEF.Comp[2][2] = -slat;

   /*calculate loop iterations */
   npq = (bfld->nmax * (bfld->nmax + 3)) / 2;

/*calculate ratio of earths radius to elevation */
   elev = (DI_GPS->alt+RE)*M2KM;
   ratio = RE*M2KM / elev;

   aa = sqrt(3.0);

/*set initial values of p */
   p[0] = 2.0 * slat;
   p[1] = 2.0 * clat;
   p[2] = 4.5 * slat * slat - 1.5;
   p[3] = 3.0 * aa * clat * slat;

/*Set initial values of q */
   q[0] = -clat;
   q[1] = slat;
   q[2] = -3.0 * clat * slat;
   q[3] = aa * (slat * slat - clat * clat);

   for(k = 0,l = 1,n = 0,m = 0,rr = ratio*ratio; k < npq; k++,m++) {
      /*testing get wrapped idx */
      kw = k%PQ_BUFFSIZE;
      if (n <= m) {
         m = -1;
         n += 1;
         /*rr = pow(ratio,n+2); */
         rr *= ratio;
         fn = n;
      }
      fm = m+1;
      if (k >= 4) {
         j = k - n;
         /*wrap j for smaller array */
         j = j%PQ_BUFFSIZE;
         if (m+1 == n) {
            aa = sqrt(1.0 - 0.5/fm);
            p[kw] = (1.0 + 1.0/fm) * aa * clat * p[j-1];
            q[kw] = aa * (clat * q[j-1] + slat/fm * p[j-1]);
            sl[m] = sl[m-1] * cl[0] + cl[m-1] * sl[0];
            cl[m] = cl[m-1] * cl[0] - sl[m-1] * sl[0];
         }else{
            aa = sqrt(fn*fn - fm*fm);
            bb = sqrt(((fn - 1.0)*(fn-1.0)) - (fm * fm))/aa;
            cc = (2.0 * fn - 1.0)/aa;
            i = k - 2 * n + 1;
            /*wrap i for smaller array */
            i = i%PQ_BUFFSIZE;
            p[kw] = (fn + 1.0) * (cc * slat/fn * p[j] - bb/(fn - 1.0) * p[i]);
            q[kw] = cc * (slat * q[j] - clat/fn * p[j]) - bb * q[i];
         }
      }
      aa = rr * mag_coeff[l-1];

      if (m == -1) {
         x = x + aa * q[kw];
         z = z - aa * p[kw];
         l += 1;
      }else{
         bb = rr * mag_coeff[l];
         cc = aa * cl[m] + bb * sl[m];
         x = x + cc * q[kw];
         z = z - cc * p[kw];
         if (clat > 0) {
            y = y + (aa * sl[m] - bb * cl[m]) *fm * p[kw]/((fn + 1.0) * clat);
         }else{
            y = y + (aa * sl[m] - bb * cl[m]) * q[kw] * slat;
         }
         l += 2;
      }
   }

   /*set destination values */
   dest.Comp[0] = x;
   dest.Comp[1] = y;
   dest.Comp[2] = z;
   Matrix3x3f_MultVec ( &Bfield_ECEF, &Dcm_NEDtoECEF, &dest );

   Dcm_ECEFtoECIF.Comp[0][0] =  cos(PriMerAng);
   Dcm_ECEFtoECIF.Comp[0][1] = -sin(PriMerAng);
   Dcm_ECEFtoECIF.Comp[0][2] = 0;
   Dcm_ECEFtoECIF.Comp[1][0] =  sin(PriMerAng);
   Dcm_ECEFtoECIF.Comp[1][1] =  cos(PriMerAng);
   Dcm_ECEFtoECIF.Comp[1][2] = 0;
   Dcm_ECEFtoECIF.Comp[2][0] = 0;
   Dcm_ECEFtoECIF.Comp[2][1] = 0;
   Dcm_ECEFtoECIF.Comp[2][2] = 1;

   Matrix3x3f_MultVec( &Bfield_ECIF, &Dcm_ECEFtoECIF, &Bfield_ECEF);

   GNC->Bfield_ECIF[0] = Bfield_ECIF.Comp[0]*NANO2TSLA;
   GNC->Bfield_ECIF[1] = Bfield_ECIF.Comp[1]*NANO2TSLA;
   GNC->Bfield_ECIF[2] = Bfield_ECIF.Comp[2]*NANO2TSLA;

   GNC->Bfield_ECEF[0] = Bfield_ECEF.Comp[0]*NANO2TSLA;
   GNC->Bfield_ECEF[1] = Bfield_ECEF.Comp[1]*NANO2TSLA;
   GNC->Bfield_ECEF[2] = Bfield_ECEF.Comp[2]*NANO2TSLA;

   GNC->Bfield_NED[0] = x*NANO2TSLA;
   GNC->Bfield_NED[1] = y*NANO2TSLA;
   GNC->Bfield_NED[2] = z*NANO2TSLA;


   return 0;
}

static int32_t solar_ephemeris(Generic_ADCS_EPH_Sol_Tlm_Payload_t *sol, Generic_ADCS_DI_Gps_Tlm_Payload_t *DI_GPS, Generic_ADCS_GNC_Tlm_Payload_t *GNC)
{
   /*float Date_Julian, Date_Solar; */
   double Date_Solar;
   double g, g_rad;
   double Long_Ecliptic;
   double Sin_Long_Ecliptic;
   double Unit_Sun_GciF[3];
   double h[3];
   double b;
   double PosN[3];
   double VelN[3];

   Date_Solar = GpsTime_TO_JD(2, DI_GPS->Weeks, DI_GPS->SecondsIntoWeek) - sol->date_epoch; // rollover=2, valid April 7, 2019 to November 20, 2038

   g = sol->coeff_G1 + sol->coeff_G2 * (double)Date_Solar;
   g_rad = g*D2R;

   Long_Ecliptic = sol->coeff_L1 + sol->coeff_l2 * Date_Solar +
                   sol->coeff_long1*sin(g_rad) + sol->coeff_long2*sin(2*g_rad);

   Sin_Long_Ecliptic = sin(Long_Ecliptic*D2R);
   Unit_Sun_GciF[0] = cos(Long_Ecliptic*D2R);
   Unit_Sun_GciF[1] = sol->cos_obliq_eclp * Sin_Long_Ecliptic;
   Unit_Sun_GciF[2] = sol->sin_obliq_eclp * Sin_Long_Ecliptic;

   GNC->svn[0] = Unit_Sun_GciF[0];
   GNC->svn[1] = Unit_Sun_GciF[1];
   GNC->svn[2] = Unit_Sun_GciF[2];

    /*Calcualte number of elapsed days since Epoch of J2000 and corresponding angle of rotation*/
    double GMST = JD_TO_GMST(Date_Solar);
    double PriMerAng = TWOPI * GMST;
    double ZAxis[3] = {0.0, 0.0, 1.0};
    double C_W_TETE[3][3],C_TETE_J2000[3][3],C_ECEF_ECI[3][3];
    HiFiEarthPrecNute(Date_Solar,C_TETE_J2000);
    SimpRot(ZAxis,PriMerAng,C_W_TETE);
    MxM(C_W_TETE,C_TETE_J2000,C_ECEF_ECI);
    double PosW[3] = {DI_GPS->ECEFX, DI_GPS->ECEFY, DI_GPS->ECEFZ};
    double VelW[3] = {DI_GPS->VelX, DI_GPS->VelY, DI_GPS->VelZ};

   /* Solar beta angle */
   MxV(C_ECEF_ECI, PosW, PosN);
   MxV(C_ECEF_ECI, VelW, VelN);

   VxV(PosN,VelN,h);
   UNITV(h);
   b = Limit(VoV(GNC->svn,h), -1.0, 1.0);
   GNC->beta = asin(b);


   return 0;
}

static void AD_imu(const Generic_ADCS_DI_Imu_Tlm_Payload_t *DI_IMU, Generic_ADCS_AD_Imu_Tlm_Payload_t *AD_IMU)
{
    if (DI_IMU->valid)
    {
        AD_IMU->valid = 1;
        for (int i = 0; i < 3; i++)
        {
            AD_IMU->acc[i] = DI_IMU->acc[i];
        }

        if (AD_IMU->init == 0)
        {
            for (int i = 0; i < 3; i++)
            {
                AD_IMU->wbn[i] = DI_IMU->wbn[i];
            }
            AD_IMU->init = 1;
        }
        else
        {
            for (int i = 0; i < 3; i++)
            {
                AD_IMU->wbn[i] = AD_IMU->alpha * AD_IMU->wbn_prev[i] + (1 - AD_IMU->alpha) * DI_IMU->wbn[i];
            }
        }
        for (int i = 0; i < 3; i++)
        {
            AD_IMU->wbn_prev[i] = AD_IMU->wbn[i];
        }
    }
    else
    {
        AD_IMU->valid = 0;
    }
}

static void AD_mag(const Generic_ADCS_DI_Mag_Tlm_Payload_t *DI_Mag, Generic_ADCS_AD_Mag_Tlm_Payload_t *AD_Mag)
{
    /* AD very simple for magnetometer... there is only one mag and no fusion with anything else */
    for (int i = 0; i < 3; i++)
    {
        AD_Mag->bvb[i] = DI_Mag->bvb[i];
    }
}

static void AD_sol(const Generic_ADCS_DI_Fss_Tlm_Payload_t *DI_Fss, const Generic_ADCS_DI_Css_Tlm_Payload_t *DI_Css,
                   Generic_ADCS_AD_Sol_Tlm_Payload_t *AD_Sol)
{
    if (DI_Fss->valid == 1)
    {
        AD_Sol->SunValid = 1;
        AD_Sol->FssValid = 1;
        AD_Sol->svb[0]   = DI_Fss->svb[0];
        AD_Sol->svb[1]   = DI_Fss->svb[1];
        AD_Sol->svb[2]   = DI_Fss->svb[2];
    }
    else if (DI_Css->valid == 1)
    {
        AD_Sol->SunValid = 1;
        AD_Sol->FssValid = 0;
        AD_Sol->svb[0]   = DI_Css->svb[0];
        AD_Sol->svb[1]   = DI_Css->svb[1];
        AD_Sol->svb[2]   = DI_Css->svb[2];
    }
    else
    {
        AD_Sol->SunValid = 0;
        AD_Sol->FssValid = 0;
        AD_Sol->svb[0]   = 0.0;
        AD_Sol->svb[1]   = 0.0;
        AD_Sol->svb[2]   = 0.0;
    }
}

static void AD_st(const Generic_ADCS_DI_St_Tlm_Payload_t *DI_ST, Generic_ADCS_AD_ST_Tlm_Payload_t *st)
{
    int    i;
    double qST[4]       = {0.0, 0.0, 0.0, 1.0};
    int    valid_st_cnt = 0;

    if (DI_ST->valid)
    {
        valid_st_cnt = valid_st_cnt + 1;
        QxQ(DI_ST->q, DI_ST->qbs, qST);
    }

    if (valid_st_cnt > 0.0)
    {
        for (i = 0; i < 4; i++)
            st->qbn[i] = qST[i];
        st->Valid = true;
    }
    else
    {
        st->Valid = false;
    }
}

static void AD_gps(const Generic_ADCS_DI_Gps_Tlm_Payload_t *DI_GPS, Generic_ADCS_AD_Gps_Tlm_Payload_t *gps)
{
    gps->Weeks = DI_GPS->Weeks;
    gps->SecondsIntoWeek = DI_GPS->SecondsIntoWeek;
    gps->Fractions = DI_GPS->Fractions;
    gps->ECEFX = DI_GPS->ECEFX;
    gps->ECEFY = DI_GPS->ECEFY;
    gps->ECEFZ = DI_GPS->ECEFZ;
    gps->VelX = DI_GPS->VelX;
    gps->VelY = DI_GPS->VelY;
    gps->VelZ = DI_GPS->VelZ;
    gps->lat = DI_GPS->lat;
    gps->lon = DI_GPS->lon;
    gps->alt = DI_GPS->alt;
}

static void AD_rateEst(Generic_ADCS_GNC_Tlm_Payload_t  GNC, Generic_ADCS_AD_Mag_Tlm_Payload_t mag, Generic_ADCS_AD_Sol_Tlm_Payload_t sol, Generic_AD_rateEst_Tlm_Payload_t *AD)
{
   int i;
   static long valid_counter = 0;
   static double w_array[100][3] = {{0}};
   double sum_wx = 0.0;
   double sum_wy = 0.0;
   double sum_wz = 0.0;

   AD->Valid = false;

   for (i = 0; i < 3; i++)
   {
      AD->wbn[i] = 0.0;
   }

   if (sol.FssValid || mag.MagValid)
   {
      if (sol.FssValid && mag.MagValid)
      {
         if (AD->MagInit || AD->SolInit)
         {
            if (AD->MagInit && AD->SolInit)
            {
               calc_wmag(GNC.DT, mag, AD);
               calc_wsol(GNC.DT, sol, AD);
               for(i = 0; i < 3; i++) AD->wbn[i] = AD->ws[i] + VoV(AD->wm,sol.svb)*sol.svb[i];
               AD->Valid = true;
            }
            else
            {
               if (AD->MagInit)
               {
                  AD->SolInit = true;
                  calc_wmag(GNC.DT, mag, AD);
                  for(i = 0; i < 3; i++) AD->wbn[i] = AD->wm[i];
                  AD->Valid = true;

               }
               else      /*sol is already intitialzied*/
               {
                  AD->MagInit = true;
                  calc_wsol(GNC.DT, sol, AD);
                  for(i = 0; i < 3; i++) AD->wbn[i] = AD->ws[i];
                  AD->Valid = true;

               }
            }
         }
         else
         {
            AD->SolInit = true;
            AD->MagInit = true;
         }
      }
      else
      {
         if (sol.FssValid)
         {
            AD->MagInit = false;
            if (AD->SolInit)
            {
               calc_wsol(GNC.DT, sol, AD);
               for(i = 0; i < 3; i++) AD->wbn[i] = AD->ws[i];
               AD->Valid = true;
            }
            else
            {
               AD->SolInit = true;
            }
         }
         else    /*Mag has to be valid*/
         {
            AD->SolInit = false;
            if (AD->MagInit)
            {
               calc_wmag(GNC.DT, mag, AD);
               for(i = 0; i < 3; i++) AD->wbn[i] = AD->wm[i];
               AD->Valid = true;
            }
            else
            {
               AD->MagInit = true;
            }
         }
      }
   }
   else
   {
      AD->MagInit = false;
      AD->SolInit = false;
   }

   /*Moving average filter applied*/
   if (AD->enable_filter && AD->Valid)
   {

      if (valid_counter < AD->sample_size)
      {
         w_array[valid_counter][0] = AD->wbn[0];
         w_array[valid_counter][1] = AD->wbn[1];
         w_array[valid_counter][2] = AD->wbn[2];
      }
      else
      {
         for(i = 0; i < AD->sample_size-1; i++)
         {
            w_array[i][0] = w_array[i+1][0];
            w_array[i][1] = w_array[i+1][1];
            w_array[i][2] = w_array[i+1][2];
         }
         w_array[AD->sample_size-1][0] = AD->wbn[0];
         w_array[AD->sample_size-1][1] = AD->wbn[1];
         w_array[AD->sample_size-1][2] = AD->wbn[2];

         for(i = 0; i < AD->sample_size; i++)
         {
            sum_wx = sum_wx + w_array[i][0];
            sum_wy = sum_wy + w_array[i][1];
            sum_wz = sum_wz + w_array[i][2];
         }
         AD->wbn[0] = sum_wx/AD->sample_size;
         AD->wbn[1] = sum_wy/AD->sample_size;
         AD->wbn[2] = sum_wz/AD->sample_size;
      }

      valid_counter = valid_counter + 1;
   }
   else
   {
      valid_counter = 0.0;
   }


   /*Update previous states*/
   for (i = 0; i < 3; i++)
   {
      AD->svb_prev[i] = sol.svb[i];
      AD->bvb_prev[i] = mag.bvb[i];
   }
}

static void calc_wmag(double dt, Generic_ADCS_AD_Mag_Tlm_Payload_t mag, Generic_AD_rateEst_Tlm_Payload_t *AD )
{

   double bvb[3] = {0}, bvb_prev[3] = {0}, ang = 0.0, axis[3] = {0}, rate = 0.0;

   int i;
   for (i = 0; i < 3; i++)
   {
      bvb[i] = mag.bvb[i];
      bvb_prev[i] = AD->bvb_prev[i];
   }
   UNITV(bvb);
   UNITV(bvb_prev);
   ang = arccos(VoV(bvb, bvb_prev));
   VxV(bvb, bvb_prev, axis);
   if (MAGV(axis) > 1.0e-10)
   {
      UNITV(axis);
   }
   rate = ang / dt;
   for (i = 0; i < 3; i++)
   {
      AD->wm[i] = rate * axis[i];
   }

}

static void calc_wsol(double dt, Generic_ADCS_AD_Sol_Tlm_Payload_t sol, Generic_AD_rateEst_Tlm_Payload_t *AD )
{

   int i;
   double ang = 0.0, axis[3] = {0}, rate = 0.0;
   ang = arccos(VoV(sol.svb, AD->svb_prev));
   VxV(sol.svb, AD->svb_prev, axis);
   if (MAGV(axis) > 1.0e-10)
   {
      UNITV(axis);
   }
   rate = ang / dt;
   for (i = 0; i < 3; i++)
   {
      AD->ws[i] = rate * axis[i];
   }

}

static void AD_to_GNC(const Generic_ADCS_AD_Tlm_Payload_t *AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC)
{
    for (int i = 0; i < 3; i++)
    {
        GNC->bvb[i] = AD->Mag.bvb[i];
        GNC->svb[i] = AD->Sol.svb[i];
        GNC->wbn[i] = AD->Imu.wbn[i];
        GNC->qbn[i] = AD->ST.qbn[i];
    }
    GNC->qbn[3]   = AD->ST.qbn[3];
    GNC->qValid   = AD->ST.Valid;
    GNC->SunValid = AD->Sol.SunValid;
}

static void AC_bdot(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Bdot_Tlm_t *ACS)
{
    /* apply control only if b-field is in range */
    if (MAGV(GNC->bvb) > ACS->b_range)
    {
        for (int i = 0; i < 3; i++)
        {
            /* backward difference b-field derivative */
            ACS->bdot[i] = (GNC->bvb[i] - ACS->bold[i]) / GNC->DT;
            /* store old b-field */
            ACS->bold[i] = GNC->bvb[i];
            /* traditional b-dot algorithm */
            GNC->Mcmd[i] = -ACS->Kb * ACS->bdot[i] / MAGV(GNC->bvb);
            /* ensure wheels disabled */
            GNC->Tcmd[i] = 0.0;
        }
    }
    else
    {
        for (int i = 0; i < 3; i++)
        {
            GNC->Mcmd[i] = 0.0;
            GNC->Tcmd[i] = 0.0;
        }
    }
}

#define EPS 1.0E-6
static void AC_sunsafe(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Sunsafe_Tlm_t *ACS)
{
    int    i;
    double u1[3] = {0.0, 0.0, 0.0}, err_b[3] = {0.0, 0.0, 0.0}; /* angle error calculation parameteres */
    double temp_sside[3] = {0.0, 0.0, 0.0};
    double SoS           = 0.0;

    /* .. Check that SS Vector is valid */
    if (GNC->SunValid)
    {

        /* .. Form attitude error signals */
        SoS = VoV(GNC->svb, ACS->sside);
        if ((SoS > (EPS - 1.0)) && (SoS < (1.0 - EPS)))
        {
            VxV(GNC->svb, ACS->sside, ACS->therr);
        }
        else if (SoS >= (1.0 - EPS))
        {
            ACS->therr[0] = 0.0;
            ACS->therr[1] = 0.0;
            ACS->therr[2] = 0.0;
        }
        else
        {
            err_b[0] = ACS->sside[1];
            err_b[1] = ACS->sside[2];
            err_b[2] = ACS->sside[0];
            if (fabs(err_b[0] - err_b[1]) < EPS && fabs(err_b[0] - err_b[2]) < EPS)
            {
                err_b[0] = -err_b[0];
            }
            VxV(ACS->sside, err_b, temp_sside);
            VxV(GNC->svb, temp_sside, ACS->therr);
        }

        /* .. Closed-loop attitude control - PD Method */
        for (i = 0; i < 3; i++)
        {
            /* Clip attitude slew rates */
            u1[i]        = Limit(ACS->Kp[i] / ACS->Kr[i] * ACS->therr[i], -ACS->vmax, ACS->vmax);
            ACS->werr[i] = GNC->wbn[i] - ACS->cmd_wbn[i];
            ACS->Tcmd[i] = -ACS->Kr[i] * (u1[i] + ACS->werr[i]);
        }

        /* .. Apply Torque Command */
        for (i = 0; i < 3; i++)
        {
            GNC->Tcmd[i] = -ACS->Tcmd[i];
        }
    }

    else
    { /* during eclipse, reduce attitude rates only */

        for (i = 0; i < 3; i++)
        {
            ACS->werr[i] = GNC->wbn[i];
            ACS->Tcmd[i] = -ACS->Kr[i] * ACS->werr[i];
        }
        /* .. Apply Torque Command  */
        for (i = 0; i < 3; i++)
        {
            GNC->Tcmd[i] = -ACS->Tcmd[i];
        }
    }

    if (GNC->HmgmtOn)
    {
        AC_h_mgmt(GNC);
        for (i = 0; i < 3; i++)
        {
            GNC->Mcmd[i] = GNC->Hmgmt.Mcmd[i];
        }
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            GNC->Mcmd[i] = 0.0;
        }
    }
}

static void AC_inertial(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Inertial_Tlm_t *ACS)
{
    int    i;
    double qErrLimited[4] = {0.0, 0.0, 0.0, 0.0}; /* Initialize Error quaterion for internal use */
    double e_axis[3]      = {0.0, 0.0, 0.0};      /* Initialze Eigen axis of the Body to Body quaternion*/
    double phiErr         = 0.0;                  /* Intialize angular error of Body to Body quaternion */

    if (GNC->qValid)
    {
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
            phiErr    = ACS->phiErr_max;
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
            ACS->therr[i]    = 2.0 * qErrLimited[i];
            ACS->sumtherr[i] = ACS->sumtherr[i] + ACS->therr[i];
            ACS->werr[i]     = -GNC->wbn[i];
            ACS->Tcmd[i]     = ACS->Kp[i] * ACS->therr[i] + ACS->Kr[i] * ACS->werr[i] + ACS->Ki[i] * ACS->sumtherr[i];
        }

        for (i = 0; i < 3; i++)
        {
            GNC->Tcmd[i] = -ACS->Tcmd[i];
        }

        if (ACS->h_mgmt)
        {
            AC_h_mgmt(GNC);
            for (i = 0; i < 3; i++)
            {
                GNC->Mcmd[i] = GNC->Hmgmt.Mcmd[i];
            }
        }
        else
        {
            for (i = 0; i < 3; i++)
            {
                GNC->Mcmd[i] = 0.0;
            }
        }
    }
}

static void AC_h_mgmt(Generic_ADCS_GNC_Tlm_Payload_t *GNC)
{

    double Herr[3] = {0.0, 0.0, 0.0};
    double bvb[3]  = {0.0, 0.0, 0.0};
    double HxB[3]  = {0.0, 0.0, 0.0};
    int    i;

    if (MAGV(GNC->bvb) > GNC->Hmgmt.b_range)
    {
        /*Test if any axis needs to be momentum managed*/
        for (i = 0; i < 3; i++)
        {
            if (fabs(GNC->HwhlB[i]) > GNC->Hmgmt.hiFrac * fabs(GNC->HwhlMaxB[i]))
            {
                GNC->Hmgmt.mm_active[i] = 1;
            }
            if (fabs(GNC->HwhlB[i]) < GNC->Hmgmt.loFrac * fabs(GNC->HwhlMaxB[i]))
            {
                GNC->Hmgmt.mm_active[i] = 0;
            }
        }
        for (i = 0; i < 3; i++)
        {
            Herr[i] = 0.0;
            if (GNC->Hmgmt.mm_active[i] == 1)
            {
                Herr[i] = GNC->HwhlB[i];
            }
        }
        CopyUnitV(GNC->bvb, bvb);
        VxV(Herr, bvb, HxB);
        for (i = 0; i < 3; i++)
        {
            GNC->Hmgmt.Mcmd[i] = GNC->Hmgmt.Kb * HxB[i] / MAGV(GNC->bvb);
        }
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            GNC->Hmgmt.Mcmd[i] = 0.0;
        }
    }
}

static void AC_rw_momentum_dump(Generic_ADCS_GNC_Tlm_Payload_t *GNC)
{
    double h_mag     = MAGV(GNC->HwhlB);
    double h_max     = MAGV(GNC->HwhlMaxB);
    double Kr        = 1.0;
    double threshold = 1E-6;

    if ((h_mag / h_max) > threshold)
    {
        for (int i = 0; i < 3; i++)
        {
            // Proportional Control, Momentum --> 0
            double Tcmd_dump = -Kr * GNC->HwhlB[i];

            GNC->Tcmd[i] += Tcmd_dump;
        }
    }
}
