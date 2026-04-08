/*******************************************************************************
** Purpose:
**   This file has the functions to ingest messages from the sensor applications.
**
*******************************************************************************/
#ifndef _GENERIC_ADCS_INGEST_H_
#define _GENERIC_ADCS_INGEST_H_

#include "generic_adcs_msg.h"

void Generic_ADCS_ingest_init(FILE *in, Generic_ADCS_DI_Tlm_Payload_t *DI);
void Generic_ADCS_ingest_generic_mag(__int32_t MagIntX, __int32_t MagIntY, __int32_t MagIntZ,
                                     Generic_ADCS_DI_Mag_Tlm_Payload_t *Mag);
void Generic_ADCS_ingest_generic_fss(float Alpha, float Beta, __uint8_t Error, Generic_ADCS_DI_Fss_Tlm_Payload_t *Fss);
void Generic_ADCS_ingest_generic_css(__uint16_t ADCV0, __uint16_t ADCV1, __uint16_t ADCV2, __uint16_t ADCV3,
                                     __uint16_t ADCV4, __uint16_t ADCV5, Generic_ADCS_DI_Css_Tlm_Payload_t *Css);
void Generic_ADCS_ingest_generic_imu(float LinX, float LinY, float LinZ, float AngX, float AngY, float AngZ,
                                     Generic_ADCS_DI_Imu_Tlm_Payload_t *Imu);
void Generic_ADCS_ingest_generic_rw(double RW0, double RW1, double RW2, Generic_ADCS_DI_Rw_Tlm_Payload_t *Rw);
void Generic_ADCS_ingest_generic_st(double Q0, double Q1, double Q2, double Q3, __uint8_t IsValid,
                                    Generic_ADCS_DI_St_Tlm_Payload_t *St);
void Generic_ADCS_ingest_novatel_gps(uint16_t Weeks, uint32_t SecondsIntoWeek, double Fractions, double ECEFX, double ECEFY, 
                                    double ECEFZ, double VelX, double VelY, double VelZ, double lat, double lon, double alt, 
                                    Generic_ADCS_DI_Gps_Tlm_Payload_t *Gps);

#endif