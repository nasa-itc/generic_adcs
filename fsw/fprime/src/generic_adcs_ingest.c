/*******************************************************************************
** Purpose:
**   This file implements the functions to ingest messages from the sensor applications.
**
*******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "generic_adcs_utilities.h"
#include "generic_adcs_ingest.h"

static const double NANO = 1.0e-9;

void Generic_ADCS_ingest_init(FILE *in, Generic_ADCS_DI_Tlm_Payload_t *DI)
{
    char junk[120], newline;
    // Magnetometer
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Mag.qbs[0], &DI->Mag.qbs[1], &DI->Mag.qbs[2], &DI->Mag.qbs[3], junk,
           &newline);
    // Fine Sun Sensor
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Fss.qbs[0], &DI->Fss.qbs[1], &DI->Fss.qbs[2], &DI->Fss.qbs[3], junk,
           &newline);
    // Coarse Sun Sensors
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[0].axis[0], &DI->Css.Sensor[0].axis[1],
           &DI->Css.Sensor[0].axis[2], &DI->Css.Sensor[0].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[1].axis[0], &DI->Css.Sensor[1].axis[1],
           &DI->Css.Sensor[1].axis[2], &DI->Css.Sensor[1].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[2].axis[0], &DI->Css.Sensor[2].axis[1],
           &DI->Css.Sensor[2].axis[2], &DI->Css.Sensor[2].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[3].axis[0], &DI->Css.Sensor[3].axis[1],
           &DI->Css.Sensor[3].axis[2], &DI->Css.Sensor[3].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[4].axis[0], &DI->Css.Sensor[4].axis[1],
           &DI->Css.Sensor[4].axis[2], &DI->Css.Sensor[4].scale, junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Css.Sensor[5].axis[0], &DI->Css.Sensor[5].axis[1],
           &DI->Css.Sensor[5].axis[2], &DI->Css.Sensor[5].scale, junk, &newline);
    // Inertial Measurement Unit
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Imu.qbs[0], &DI->Imu.qbs[1], &DI->Imu.qbs[2], &DI->Imu.qbs[3], junk,
           &newline);
    fscanf(in, "%lf %lf %lf%[^\n]%[\n]", &DI->Imu.pos[0], &DI->Imu.pos[1], &DI->Imu.pos[2], junk, &newline);
    // Reaction Wheels
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    double h_max[3] = {0.0, 0.0, 0.0};
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Rw.whl_axis[0][0], &DI->Rw.whl_axis[0][1], &DI->Rw.whl_axis[0][2],
           &h_max[0], junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Rw.whl_axis[1][0], &DI->Rw.whl_axis[1][1], &DI->Rw.whl_axis[1][2],
           &h_max[1], junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->Rw.whl_axis[2][0], &DI->Rw.whl_axis[2][1], &DI->Rw.whl_axis[2][2],
           &h_max[2], junk, &newline);
    double H_in_body[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; i++)
    {
        DI->Rw.H_maxB[i] = 0.0;
    }
    for (int whl = 0; whl < 3; whl++)
    {
        SxV(h_max[whl], DI->Rw.whl_axis[whl], H_in_body);
        for (int i = 0; i < 3; i++)
        {
            DI->Rw.H_maxB[i] += H_in_body[i];
        }
    }
    // Star Tracker
    fscanf(in, "%[^\n]%[\n]", junk, &newline);
    fscanf(in, "%lf %lf %lf %lf%[^\n]%[\n]", &DI->St.qbs[0], &DI->St.qbs[1], &DI->St.qbs[2], &DI->St.qbs[3], junk,
           &newline);
}

void Generic_ADCS_ingest_generic_mag(__int32_t MagIntX, __int32_t MagIntY, __int32_t MagIntZ,
                                     Generic_ADCS_DI_Mag_Tlm_Payload_t *Mag)
{
    double bvs[3] = {(double)MagIntX, (double)MagIntY, (double)MagIntZ};

    QxV(Mag->qbs, bvs, Mag->bvb);

    Mag->bvb[0] *= NANO;
    Mag->bvb[1] *= NANO;
    Mag->bvb[2] *= NANO;
}

void Generic_ADCS_ingest_generic_fss(float Alpha, float Beta, __uint8_t Error, Generic_ADCS_DI_Fss_Tlm_Payload_t *Fss)
{
    Fss->valid = 0;
    if (Error == 0)
        Fss->valid = 1;

    if (Fss->valid == 1)
    {
        double svs[3];
        double ta = tan(Alpha);
        double tb = tan(Beta);
        svs[2]    = 1.0 / sqrt(1 + ta * tb + tb * tb);
        svs[0]    = svs[2] * ta;
        svs[1]    = svs[2] * tb;
        QxV(Fss->qbs, svs, Fss->svb);
    }
    else
    {
        Fss->svb[0] = 0.0;
        Fss->svb[1] = 0.0;
        Fss->svb[2] = 0.0;
    }
}

void Generic_ADCS_ingest_generic_css(__uint16_t ADCV0, __uint16_t ADCV1, __uint16_t ADCV2, __uint16_t ADCV3,
                                     __uint16_t ADCV4, __uint16_t ADCV5, Generic_ADCS_DI_Css_Tlm_Payload_t *Css)
{
    Css->Sensor[0].percenton = ADCV0 * Css->Sensor[0].scale;
    Css->Sensor[1].percenton = ADCV1 * Css->Sensor[1].scale;
    Css->Sensor[2].percenton = ADCV2 * Css->Sensor[2].scale;
    Css->Sensor[3].percenton = ADCV3 * Css->Sensor[3].scale;
    Css->Sensor[4].percenton = ADCV4 * Css->Sensor[4].scale;
    Css->Sensor[5].percenton = ADCV5 * Css->Sensor[5].scale;

    double svb[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 6; i++)
    {
        svb[0] += Css->Sensor[i].axis[0] * Css->Sensor[i].percenton;
        svb[1] += Css->Sensor[i].axis[1] * Css->Sensor[i].percenton;
        svb[2] += Css->Sensor[i].axis[2] * Css->Sensor[i].percenton;
    }
    UNITV(svb);

    Css->svb[0] = svb[0];
    Css->svb[1] = svb[1];
    Css->svb[2] = svb[2];
    if (MAGV(svb) > 0.0)
    {
        Css->valid = 1;
    }
    else
    {
        Css->valid = 0;
    }
}

void Generic_ADCS_ingest_generic_imu(float LinX, float LinY, float LinZ, float AngX, float AngY, float AngZ,
                                     Generic_ADCS_DI_Imu_Tlm_Payload_t *Imu)
{
    double wsn[3] = {AngX, AngY, AngZ};
    QxV(Imu->qbs, wsn, Imu->wbn);

    double acc[3] = {LinX, LinY, LinZ};
    QxV(Imu->qbs, acc, Imu->acc);
    Imu->valid = 1;
}

void Generic_ADCS_ingest_generic_rw(double RW0, double RW1, double RW2, Generic_ADCS_DI_Rw_Tlm_Payload_t *Rw)
{
    double H_in_body[3]   = {0.0, 0.0, 0.0};
    double rwMomentums[3] = {RW0, RW1, RW2};

    for (int i = 0; i < 3; i++)
    {
        Rw->HwhlB[i] = 0.0;
    }

    for (int whl = 0; whl < 3; whl++)
    {
        SxV(rwMomentums[whl], Rw->whl_axis[whl], H_in_body);

        for (int i = 0; i < 3; i++)
        {
            Rw->HwhlB[i] += H_in_body[i];
        }
    }
}

void Generic_ADCS_ingest_generic_st(double Q0, double Q1, double Q2, double Q3, __uint8_t IsValid,
                                    Generic_ADCS_DI_St_Tlm_Payload_t *St)
{
    St->valid   = IsValid;
    double q[4] = {Q0, Q1, Q2, Q3};
    QxQ(q, St->qbs, St->q);
}

void Generic_ADCS_ingest_novatel_gps(uint16_t Weeks, uint32_t SecondsIntoWeek, double Fractions, double ECEFX, double ECEFY, double ECEFZ, double VelX, double VelY, double VelZ, double lat, double lon, double alt, Generic_ADCS_DI_Gps_Tlm_Payload_t *Gps)
{
    Gps->Weeks = Weeks;
    Gps->SecondsIntoWeek = SecondsIntoWeek;
    Gps->Fractions = Fractions;
    Gps->ECEFX = ECEFX;
    Gps->ECEFY = ECEFY;
    Gps->ECEFZ = ECEFZ;
    Gps->VelX = VelX;
    Gps->VelY = VelY;
    Gps->VelZ = VelZ;
    Gps->lat = lat;
    Gps->lon = lon;
    Gps->alt = alt;
}