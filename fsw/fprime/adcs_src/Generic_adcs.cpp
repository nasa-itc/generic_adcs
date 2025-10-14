// ======================================================================
// \title  Generic_adcs.cpp
// \author jstar
// \brief  cpp file for Generic_adcs component implementation class
// ======================================================================

#include "adcs_src/Generic_adcs.hpp"
// #include "FpConfig.hpp"
#include "Fw/FPrimeBasicTypes.hpp"
#include <Fw/Log/LogString.hpp>

#include <math.h>

namespace Components {

  // ----------------------------------------------------------------------
  // Component construction and destruction
  // ----------------------------------------------------------------------

  Generic_adcs ::
    Generic_adcs(const char* const compName) :
      Generic_adcsComponentBase(compName)
  {
    ingest_init(&DIPacket.Payload);
    init_adac(&ADPacket.Payload, &GNCPacket.Payload, &ACSPacket.Payload);
    init_output(&DOPacket.Payload);
    
    HkTelemetryPkt.CommandCount = 0;
    HkTelemetryPkt.CommandErrorCount = 0;
  }

  Generic_adcs ::
    ~Generic_adcs()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for commands
  // ----------------------------------------------------------------------

  void Generic_adcs :: IMUin_handler(NATIVE_INT_TYPE portNum, F32 XLin, F32 YLin, F32 ZLin, F32 XAng, F32 YAng, F32 ZAng)
  {
    Generic_ADCS_ingest_generic_imu(XLin, YLin, ZLin, XAng, YAng, ZAng, &DIPacket.Payload.Imu);
    // this->tlmWrite_ingestIMUCount(++ingestIMUCount);
  }

  void Generic_adcs :: MAGin_handler( NATIVE_INT_TYPE portNum, I32 MagX, I32 MagY, I32 MagZ)
  {
    Generic_ADCS_ingest_generic_mag(MagX, MagY, MagZ, &DIPacket.Payload.Mag);
    // this->tlmWrite_ingestMagCount(++ingestMagCount);
  }

  void Generic_adcs :: FSSin_handler( NATIVE_INT_TYPE portNum, F32 Alpha, F32 Beta, U8 Error)
  {
    Generic_ADCS_ingest_generic_fss(Alpha, Beta, Error, &DIPacket.Payload.Fss);
    // this->tlmWrite_ingestFSSCount(++ingestFSSCount);
  }

  void Generic_adcs :: CSSin_handler( NATIVE_INT_TYPE portNum, U16 ADCV0, U16 ADCV1, U16 ADCV2, U16 ADCV3, U16 ADCV4, U16 ADCV5)
  {
    Generic_ADCS_ingest_generic_css(ADCV0, ADCV1, ADCV2, ADCV3, ADCV4, ADCV5, &DIPacket.Payload.Css);
    // this->tlmWrite_ingestCSSCount(++ingestCSSCount);
  }

  void Generic_adcs :: RWin_handler( NATIVE_INT_TYPE portNum, F64 RW0, F64 RW1, F64 RW2)
  {
    Generic_ADCS_ingest_generic_rw(RW0, RW1, RW2, &DIPacket.Payload.Rw);
    // this->tlmWrite_ingestRWCount(++ingestRWCount);
  }

  void Generic_adcs :: STin_handler( NATIVE_INT_TYPE portNum, F64 Q0, F64 Q1, F64 Q2, F64 Q3, U8 IsValid)
  {
    Generic_ADCS_ingest_generic_st(Q0, Q1, Q2, Q3, IsValid, &DIPacket.Payload.St);
    // this->tlmWrite_ingestSTCount(++ingestSTCount);
  }

  void Generic_adcs :: updateData_handler(const NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context)
  {
    Generic_ADCS_execute_attitude_determination_and_attitude_control(&DIPacket.Payload, &ADPacket.Payload, &GNCPacket.Payload, &ACSPacket.Payload);
    output_actuators(&GNCPacket.Payload, &DOPacket.Payload, &MtbPctOnCmd, &RwCmd);
  }

  void Generic_adcs :: SET_MODE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Generic_adcs_adcs_mode MODE)
  {
    GNCPacket.Payload.Mode = MODE.e;

    this->tlmWrite_ADCSMode(MODE);

    switch (MODE.e)
    {
        case BDOT_MODE:{
            Fw::LogStringArg log_msg("Set to BDOT Mode!");
            this->log_ACTIVITY_HI_TELEM(log_msg);
            this->RWOUTout_out(0, 0, 0, 0); // turn off RW
            break;
        }
        case SUNSAFE_MODE:{
            Fw::LogStringArg log_msg("Set to SUNSAFE Mode!");
            this->log_ACTIVITY_HI_TELEM(log_msg);
            break;
        }
        case INERTIAL_MODE:{
            Fw::LogStringArg log_msg("Set to Inertial Mode!");
            this->log_ACTIVITY_HI_TELEM(log_msg);
            break;
        }
        case PASSIVE_MODE:{
        default:
            Fw::LogStringArg log_msg("Set to PASSIVE Mode!");
            this->log_ACTIVITY_HI_TELEM(log_msg);
            this->RWOUTout_out(0, 0, 0, 0); // turn off RW
            break;
        }
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void Generic_adcs :: SET_INERTIAL_QUATERNION_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F64 QX, F64 QY, F64 QZ, F64 QW)
  {
        ACSPacket.Payload.Inertial.qbn_cmd[0] = (double)QX;
        ACSPacket.Payload.Inertial.qbn_cmd[1] = (double)QY;
        ACSPacket.Payload.Inertial.qbn_cmd[2] = (double)QZ;
        ACSPacket.Payload.Inertial.qbn_cmd[3] = (double)QW;

        this->tlmWrite_INERTIALQUATERNIONX(QX);
        this->tlmWrite_INERTIALQUATERNIONY(QY);
        this->tlmWrite_INERTIALQUATERNIONZ(QZ);
        this->tlmWrite_INERTIALQUATERNIONW(QW);

        char quatMsg[50];
        sprintf(quatMsg, "Set inertial quat to (%.3lf,%.3lf,%.3lf,%.3lf)", QW, QX, QY, QZ);
        Fw::LogStringArg log_msg(quatMsg);
        this->log_ACTIVITY_HI_TELEM(log_msg);

        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);

  }

  void Generic_adcs :: NOOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq)
  {
    HkTelemetryPkt.CommandCount++;

    Fw::LogStringArg log_msg("NOOP SENT");
    this->log_ACTIVITY_HI_TELEM(log_msg);
    this->tlmWrite_CommandCount(HkTelemetryPkt.CommandCount);
    this->tlmWrite_CommandErrorCount(HkTelemetryPkt.CommandErrorCount);
    this->tlmWrite_ADCSMode(get_adcs_mode(GNCPacket.Payload.Mode));
    this->tlmWrite_ADCSMomentumManagement(get_adcs_mgmt_state(GNCPacket.Payload.HmgmtOn));

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void Generic_adcs :: RESET_COUNTERS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq)
  {
    HkTelemetryPkt.CommandCount = 0;
    HkTelemetryPkt.CommandErrorCount = 0;

    Fw::LogStringArg log_msg("Reset Counters command successful!");
    this->log_ACTIVITY_HI_TELEM(log_msg);
    this->tlmWrite_CommandCount(HkTelemetryPkt.CommandCount);
    this->tlmWrite_CommandErrorCount(HkTelemetryPkt.CommandErrorCount);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }


  void Generic_adcs :: REQUEST_HOUSEKEEPING_cmdHandler(FwOpcodeType opCode, U32 cmdSeq)
  {
    this->tlmWrite_CommandCount(HkTelemetryPkt.CommandCount);
    this->tlmWrite_CommandErrorCount(HkTelemetryPkt.CommandErrorCount);
    this->tlmWrite_ADCSMode(get_adcs_mode(GNCPacket.Payload.Mode));
    this->tlmWrite_ADCSMomentumManagement(get_adcs_mgmt_state(GNCPacket.Payload.HmgmtOn));

    Fw::LogStringArg log_msg("Requested Housekeeping successfully!");
    this->log_ACTIVITY_HI_TELEM(log_msg);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void Generic_adcs :: SET_MOMENTUM_MANAGEMENT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Generic_adcs_adcs_mgmt_state STATE)
  {
    GNCPacket.Payload.HmgmtOn = STATE.e;

    this->tlmWrite_ADCSMomentumManagement(STATE);

    if(STATE.e){
      Fw::LogStringArg log_msg("Momentum Management ON!");
      this->log_ACTIVITY_HI_TELEM(log_msg);
    }
    else
    {
      Fw::LogStringArg log_msg("Momentum Management OFF!");
      this->log_ACTIVITY_HI_TELEM(log_msg);
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void Generic_adcs :: ingest_init(Generic_ADCS_DI_Tlm_Payload_t *DI)
  {
    //hardcoded instead of from the cfg

    //mag
    DI->Mag.qbs[0] = 0.0;
    DI->Mag.qbs[1] = 0.0;
    DI->Mag.qbs[2] = 0.0;
    DI->Mag.qbs[3] = 1.0;

    //fss
    DI->Fss.qbs[0] = 0.0;
    DI->Fss.qbs[1] = -0.7071;
    DI->Fss.qbs[2] = 0.0;
    DI->Fss.qbs[3] = 0.7071;

    //css
    DI->Css.Sensor[0].axis[0] = 1.0;
    DI->Css.Sensor[0].axis[1] = 0.0;
    DI->Css.Sensor[0].axis[2] = 0.0;
    DI->Css.Sensor[0].scale   = 0.001;

    DI->Css.Sensor[1].axis[0] = -1.0;
    DI->Css.Sensor[1].axis[1] = 0.0;
    DI->Css.Sensor[1].axis[2] = 0.0;
    DI->Css.Sensor[1].scale   = 0.001;

    DI->Css.Sensor[2].axis[0] = 0.0;
    DI->Css.Sensor[2].axis[1] = 1.0;
    DI->Css.Sensor[2].axis[2] = 0.0;
    DI->Css.Sensor[2].scale   = 0.001;

    DI->Css.Sensor[3].axis[0] = 0.0;
    DI->Css.Sensor[3].axis[1] = -1.0;
    DI->Css.Sensor[3].axis[2] = 0.0;
    DI->Css.Sensor[3].scale   = 0.001;

    DI->Css.Sensor[4].axis[0] = 0.0;
    DI->Css.Sensor[4].axis[1] = 0.0;
    DI->Css.Sensor[4].axis[2] = 1.0;
    DI->Css.Sensor[4].scale   = 0.001;

    DI->Css.Sensor[5].axis[0] = 0.0;
    DI->Css.Sensor[5].axis[1] = 0.0;
    DI->Css.Sensor[5].axis[2] = -1.0;
    DI->Css.Sensor[5].scale   = 0.001;

    //imu
    DI->Imu.qbs[0] = 0.0;
    DI->Imu.qbs[1] = 0.0;
    DI->Imu.qbs[2] = 0.0;
    DI->Imu.qbs[3] = 1.0;

    DI->Imu.pos[0] = 0.5;
    DI->Imu.pos[1] = 1.0;
    DI->Imu.pos[2] = 1.5;


    //rw
    double h_max[3] = {0.0, 0.0, 0.0};
    DI->Rw.whl_axis[0][0] =1.0;
    DI->Rw.whl_axis[0][1] =0.0;
    DI->Rw.whl_axis[0][2] =0.0;
    h_max[0] =0.01082;

    DI->Rw.whl_axis[1][0] =0.0;
    DI->Rw.whl_axis[1][1] =1.0;
    DI->Rw.whl_axis[1][2] =0.0;
    h_max[1] =0.01082;

    DI->Rw.whl_axis[2][0] =0.0;
    DI->Rw.whl_axis[2][1] =0.0;
    DI->Rw.whl_axis[2][2] =1.0;
    h_max[2] =0.01082;

    double H_in_body[3] = {0.0, 0.0, 0.0};
    for(int i = 0; i < 3; i++)
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


    //st
    DI->St.qbs[0] = 0.0;
    DI->St.qbs[1] = 0.0;
    DI->St.qbs[2] = 0.0;
    DI->St.qbs[3] = 1.0;
  }

  void Generic_adcs :: init_adac(Generic_ADCS_AD_Tlm_Payload_t *AD, Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_AC_Tlm_Payload_t  *ACS)
  {
    //Hardcode instead of reading from the cfg

    AD->Imu.alpha = 0.0;

    GNC->DT = 0.1;
    GNC->MaxMcmd = 1.42;

    ACS->Bdot.b_range = 4.096E-6;
    ACS->Bdot.Kb = 200.0;

    ACS->Sunsafe.Kp[0] = 0.0047;
    ACS->Sunsafe.Kp[1] = 0.0047;
    ACS->Sunsafe.Kp[2] = 0.0047;
    ACS->Sunsafe.Kr[0] = 0.1329;
    ACS->Sunsafe.Kr[1] = 0.1329;
    ACS->Sunsafe.Kr[2] = 0.1329;

    ACS->Sunsafe.sside[0] = 1.0;
    ACS->Sunsafe.sside[1] = 0.0;
    ACS->Sunsafe.sside[2] = 0.0;
    ACS->Sunsafe.vmax = 0.1;
    ACS->Sunsafe.cmd_wbn[0] = 0.0;
    ACS->Sunsafe.cmd_wbn[1] = 0.0;
    ACS->Sunsafe.cmd_wbn[2] = 0.0;

    for(int i = 0; i < 3; i++)
    {
      ACS->Sunsafe.therr[i] = ACS->Sunsafe.werr[i] = ACS->Sunsafe.Tcmd[i] = 0;
    }

    ACS->Inertial.qbn_cmd[0] = 0.5;
    ACS->Inertial.qbn_cmd[1] = 0.5;
    ACS->Inertial.qbn_cmd[2] = 0.5;
    ACS->Inertial.qbn_cmd[3] = 0.5;

    ACS->Inertial.Kp[0] = 0.04;
    ACS->Inertial.Kp[1] = 0.04;
    ACS->Inertial.Kp[2] = 0.04;
    ACS->Inertial.Kr[0] = 0.28;
    ACS->Inertial.Kr[1] = 0.28;
    ACS->Inertial.Kr[2] = 0.28;

    ACS->Inertial.Ki[0] = 0.0;
    ACS->Inertial.Ki[1] = 0.0;
    ACS->Inertial.Ki[2] = 0.0;

    GNC->Hmgmt.Kb = 1.0;
    GNC->Hmgmt.b_range = 4.096E-6;
    GNC->Hmgmt.loFrac = 0.1;
    GNC->Hmgmt.hiFrac = 0.5;
  }

  void Generic_adcs :: init_output(Generic_ADCS_DO_Tlm_Payload_t *DO)
  {
    DO->Trq.qba[0] = 0.0;
    DO->Trq.qba[1] = 0.0;
    DO->Trq.qba[2] = 0.0;
    DO->Trq.qba[3] = 1.0;

    DO->Rw.axis[0][0] = 1.0;
    DO->Rw.axis[0][1] = 0.0;
    DO->Rw.axis[0][2] = 0.0;

    DO->Rw.axis[1][0] = 0.0;
    DO->Rw.axis[1][1] = 1.0;
    DO->Rw.axis[1][2] = 0.0;

    DO->Rw.axis[2][0] = 0.0;
    DO->Rw.axis[2][1] = 0.0;
    DO->Rw.axis[2][2] = 1.0;

    for(int i = 0; i < 3; i++) CurrentMtb[i] = {1, 0};
    for(int i = 0; i < 3; i++) CurrentRw[i] = 0;
  }

    void Generic_adcs :: output_actuators(Generic_ADCS_GNC_Tlm_Payload_t *GNC, Generic_ADCS_DO_Tlm_Payload_t *DO,
                                        GENERIC_TORQUER_All_Percent_On_cmd_t *MtbPctOnCmd, GENERIC_RW_Cmd_t *RwCmd)
    {
        send_mtb_commands(GNC->Mcmd, &DO->Trq, MtbPctOnCmd);
        send_rw_commands(GNC->Tcmd, &DO->Rw, RwCmd);
    }

    void Generic_adcs :: send_mtb_commands(double Mcmd[3], Generic_ADCS_DO_Trq_TlmPayload_t *DO,
                                           GENERIC_TORQUER_All_Percent_On_cmd_t *MtbPctOnCmd)
    {
        QTxV(DO->qba, Mcmd, DO->Mcmd);

        mcmd_to_percent_direction(DO->Mcmd[0], &MtbPctOnCmd->PercentOn_0, &MtbPctOnCmd->Direction_0);
        mcmd_to_percent_direction(DO->Mcmd[1], &MtbPctOnCmd->PercentOn_1, &MtbPctOnCmd->Direction_1);
        mcmd_to_percent_direction(DO->Mcmd[2], &MtbPctOnCmd->PercentOn_2, &MtbPctOnCmd->Direction_2);
        if ((MtbPctOnCmd->Direction_0 != CurrentMtb[0].Direction) ||
            (MtbPctOnCmd->PercentOn_0 != CurrentMtb[0].PercentOn) ||
            (MtbPctOnCmd->Direction_1 != CurrentMtb[1].Direction) ||
            (MtbPctOnCmd->PercentOn_1 != CurrentMtb[1].PercentOn) ||
            (MtbPctOnCmd->Direction_2 != CurrentMtb[2].Direction) || (MtbPctOnCmd->PercentOn_2 != CurrentMtb[2].PercentOn))
        {
            //send commands to torquer
            this->TORQout_out(0, MtbPctOnCmd->PercentOn_0, MtbPctOnCmd->Direction_0, MtbPctOnCmd->PercentOn_1, MtbPctOnCmd->Direction_1, MtbPctOnCmd->PercentOn_2, MtbPctOnCmd->Direction_2);


            CurrentMtb[0].Direction = MtbPctOnCmd->Direction_0;
            CurrentMtb[0].PercentOn = MtbPctOnCmd->PercentOn_0;
            CurrentMtb[1].Direction = MtbPctOnCmd->Direction_1;
            CurrentMtb[1].PercentOn = MtbPctOnCmd->PercentOn_1;
            CurrentMtb[2].Direction = MtbPctOnCmd->Direction_2;
            CurrentMtb[2].PercentOn = MtbPctOnCmd->PercentOn_2;
        }
    }

    void Generic_adcs :: mcmd_to_percent_direction(double Mcmd, uint8_t *percent, uint8_t *direction)
    {
        double pct = 100.0 * Mcmd / GNCPacket.Payload.MaxMcmd;
        *direction = 1;
        if (pct < 0)
        {
            pct *= -1.0;
            *direction = 0;
        }
        if (pct > 100)
            pct = 100;
        *percent = pct;
    }

    void Generic_adcs :: send_rw_commands(double Tcmd[3], Generic_ADCS_DO_Rw_TlmPayload_t *DO, GENERIC_RW_Cmd_t *RwCmd)
    {
        double torque[3];
        bool new_rw_cmds = false;
        for (uint8_t i = 0; i < 3; i++)
        {
            DO->Tcmd[i] = Tcmd[i];
            torque[i]      = VoV(Tcmd, DO->axis[i]); // cmd is in 10^-4 Nm
            if (torque[i] != CurrentRw[i])
            {
                RwCmd->data         = torque[i];
                RwCmd->wheel_number = i;
                //send commands to RW
                CurrentRw[i] = torque[i];
                new_rw_cmds = true;
            }
        }

        if(new_rw_cmds) this->RWOUTout_out(0, torque[0], torque[1], torque[2]);
    }

    inline Generic_adcs_adcs_mode Generic_adcs :: get_adcs_mode(uint8_t mode)
    {
      Generic_adcs_adcs_mode modeEnum;

      switch(mode)
      {
        case BDOT_MODE:
            modeEnum.e = Generic_adcs_adcs_mode::BDOT;
            break;

        case SUNSAFE_MODE:
            modeEnum.e = Generic_adcs_adcs_mode::SUNSAFE;
            break;

        case INERTIAL_MODE:
            modeEnum.e = Generic_adcs_adcs_mode::INERTIAL;
            break;

        case PASSIVE_MODE:
        default:
            modeEnum.e = Generic_adcs_adcs_mode::PASSIVE;
            break;
      }

      return modeEnum;
    }

    inline Generic_adcs_adcs_mgmt_state Generic_adcs :: get_adcs_mgmt_state(uint8_t state)
    {
      Generic_adcs_adcs_mgmt_state stateEnum;

      if(state)
      {
        stateEnum.e = Generic_adcs_adcs_mgmt_state::ON;
      }
      else
      {
        stateEnum.e = Generic_adcs_adcs_mgmt_state::OFF;
      }

      return stateEnum;
    }

}
