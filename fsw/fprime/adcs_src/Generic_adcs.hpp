// ======================================================================
// \title  Generic_adcs.hpp
// \author jstar
// \brief  hpp file for Generic_adcs component implementation class
// ======================================================================

#ifndef Components_Generic_adcs_HPP
#define Components_Generic_adcs_HPP

#include "adcs_src/Generic_adcsComponentAc.hpp"
#include "adcs_src/Generic_adcs_adcs_modeEnumAc.hpp"
#include "adcs_src/Generic_adcs_adcs_mgmt_stateEnumAc.hpp"


extern "C"{
  #include "generic_adcs_utilities.h"
  #include "generic_adcs_msg.h"
  #include "generic_adcs_ingest.h"
  #include "generic_adcs_adac.h"
}

#define NANO 10e-9
#define EPS 1.0E-6

#define PASSIVE_MODE 0
#define BDOT_MODE    1
#define SUNSAFE_MODE 2
#define INERTIAL_MODE 3

namespace Components {

  class Generic_adcs :
    public Generic_adcsComponentBase
  {

    public:

    Generic_ADCS_Hk_tlm_t  HkTelemetryPkt; /* GENERIC_ADCS Housekeeping Telemetry Packet */
    Generic_ADCS_EPH_Tlm_t EPHPacket;
    Generic_ADCS_DI_Tlm_t  DIPacket;
    Generic_ADCS_AD_Tlm_t  ADPacket;
    Generic_ADCS_GNC_Tlm_t GNCPacket;
    Generic_ADCS_AC_Tlm_t  ACSPacket;
    Generic_ADCS_DO_Tlm_t  DOPacket;
    GENERIC_TORQUER_All_Percent_On_cmd_t MtbPctOnCmd;
    GENERIC_RW_Cmd_t                     RwCmd;

    U32 ingestIMUCount = 0;
    U32 ingestMagCount = 0;
    U32 ingestFSSCount = 0;
    U32 ingestCSSCount = 0;
    U32 ingestRWCount = 0;
    U32 ingestSTCount = 0;
    U32 ingestGPSCount = 0;
    U32 ingestPASSIVE = 0;
    U32 ingestSUNSAFE = 0;
    U32 ingestINERTIAL = 0;
    U32 ingestBDOT = 0;

    struct
    {
        uint8_t Direction;
        uint8_t PercentOn;
    } CurrentMtb[3];

    int16_t CurrentRw[3];

    
      // ----------------------------------------------------------------------
      // Component construction and destruction
      // ----------------------------------------------------------------------

      //! Construct Generic_adcs object
      Generic_adcs(
          const char* const compName //!< The component name
      );

      //! Destroy Generic_adcs object
      ~Generic_adcs();

    private:

      void IMUin_handler(
        FwIndexType portNum,
        F32 XLin,
        F32 YLin,
        F32 ZLin,
        F32 XAng,
        F32 YAng,
        F32 ZAng
      ) override;

      void MAGin_handler(
        FwIndexType portNum,
        I32 MagX,
        I32 MagY,
        I32 MagZ
      ) override;

      void FSSin_handler(
        FwIndexType portNum,
        F32 Alpha,
        F32 Beta,
        U8 Error
      ) override;

      void CSSin_handler(
        FwIndexType portNum,
        U16 ADCV0,
        U16 ADCV1,
        U16 ADCV2,
        U16 ADCV3,
        U16 ADCV4,
        U16 ADCV5
      ) override;

      void RWin_handler(
        FwIndexType portNum,
        F64 RW0,
        F64 RW1,
        F64 RW2
      ) override;

      void STin_handler(
        FwIndexType portNum,
        F64 Q0,
        F64 Q1,
        F64 Q2,
        F64 Q3,
        U8 IsValid
      ) override;

      void GPSin_handler(
        FwIndexType portNum,
        I16 Weeks,
        U32 SecondsIntoWeek,
        F64 Fractions,
        F64 ECEFX,
        F64 ECEFY,
        F64 ECEFZ,
        F64 VelX,
        F64 VelY,
        F64 VelZ,
        F64 lat,
        F64 lon,
        F64 alt
      ) override;

      void updateData_handler(
        const FwIndexType portNum,
        U32 context
      ) override;

      void SET_MODE_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        Generic_adcs_adcs_mode MODE
      ) override;

      void SET_INERTIAL_QUATERNION_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        F64 QX, 
        F64 QY, 
        F64 QZ, 
        F64 QW
      ) override;

      void NOOP_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq
      ) override;

      void RESET_COUNTERS_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq
      ) override;

      void REQUEST_HOUSEKEEPING_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq
      ) override;

      void SET_MOMENTUM_MANAGEMENT_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        Generic_adcs_adcs_mgmt_state STATE
      ) override;

      //refactor everything below eventually

      void ingest_init(Generic_ADCS_DI_Tlm_Payload_t *DI);
      void init_adac(Generic_ADCS_EPH_Tlm_Payload_t *EPH,
                     Generic_ADCS_AD_Tlm_Payload_t *AD,
                     Generic_ADCS_GNC_Tlm_Payload_t *GNC,
                     Generic_ADCS_AC_Tlm_Payload_t  *ACS);
      
      void init_output(Generic_ADCS_DO_Tlm_Payload_t *DO);

      void output_actuators(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_DO_Tlm_Payload_t *DO,
                            GENERIC_TORQUER_All_Percent_On_cmd_t *MtbPctOnCmd, GENERIC_RW_Cmd_t *RwCmd);
      
      void send_mtb_commands(double Mcmd[3], Generic_ADCS_DO_Trq_TlmPayload_t *DO,
                            GENERIC_TORQUER_All_Percent_On_cmd_t *MtbPctOnCmd);

      void mcmd_to_percent_direction(double Mcmd, uint8_t *percent, uint8_t *direction);

      void send_rw_commands(double Tcmd[3], Generic_ADCS_DO_Rw_TlmPayload_t *DO, GENERIC_RW_Cmd_t *RwCmd);

      inline Generic_adcs_adcs_mode get_adcs_mode(uint8_t mode);

      inline Generic_adcs_adcs_mgmt_state get_adcs_mgmt_state(uint8_t state);

  };

}

#endif
