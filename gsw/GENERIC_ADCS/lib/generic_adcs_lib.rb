# Library for GENERIC_ADCS Target
require 'cosmos'
require 'cosmos/script'

#
# Definitions
#
GENERIC_ADCS_CMD_SLEEP = 0.25
GENERIC_ADCS_RESPONSE_TIMEOUT = 5
GENERIC_ADCS_MODE_CHECK_TIMEOUT = 60;
GENERIC_ADCS_TEST_LOOP_COUNT = 1
GENERIC_ADCS_DEVICE_LOOP_COUNT = 5

#
# Functions
#
def get_adcs_hk()
    cmd("GENERIC_ADCS GENERIC_ADCS_REQ_HK")
    wait_check_packet("GENERIC_ADCS", "GENERIC_ADCS_HK_TLM", 1, GENERIC_ADCS_RESPONSE_TIMEOUT)
    sleep(GENERIC_ADCS_CMD_SLEEP)
end

def get_adcs_data()
    cmd("GENERIC_ADCS GENERIC_ADCS_SEND_DI_CC")
    wait_check_packet("GENERIC_ADCS", "GENERIC_ADCS_DI", 1, GENERIC_ADCS_RESPONSE_TIMEOUT)
    cmd("GENERIC_ADCS GENERIC_ADCS_SEND_GNC_CC")
    wait_check_packet("GENERIC_ADCS", "GENERIC_ADCS_GNC", 1, GENERIC_ADCS_RESPONSE_TIMEOUT)
    cmd("GENERIC_ADCS GENERIC_ADCS_SEND_AD_CC")
    wait_check_packet("GENERIC_ADCS", "GENERIC_ADCS_AD", 1, GENERIC_ADCS_RESPONSE_TIMEOUT)
    cmd("GENERIC_ADCS GENERIC_ADCS_SEND_AC_CC")
    wait_check_packet("GENERIC_ADCS", "GENERIC_ADCS_AC", 1, GENERIC_ADCS_RESPONSE_TIMEOUT)
    cmd("GENERIC_ADCS GENERIC_ADCS_SEND_DO_CC")
    wait_check_packet("GENERIC_ADCS", "GENERIC_ADCS_DO", 1, GENERIC_ADCS_RESPONSE_TIMEOUT)
    sleep(GENERIC_ADCS_CMD_SLEEP)
end

def adcs_cmd(*command)
    count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_COUNT") + 1

    if (count == 256)
        count = 0
    end

    cmd(*command)
    get_adcs_hk()
    current = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_COUNT")
    if (current != count)
        # Try again
        cmd(*command)
        get_adcs_hk()
        current = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_COUNT")
        if (current != count)
            # Third times the charm
            cmd(*command)
            get_adcs_hk()
            current = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_COUNT")
        end
    end
    check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_COUNT >= #{count}")
end

def adcs_sunsafe()

    cmd("GENERIC_ADCS GENERIC_ADCS_SET_MODE_CC with GNC_MODE SUNSAFE_MODE")

end

def adcs_bdot()

    cmd("GENERIC_ADCS GENERIC_ADCS_SET_MODE_CC with GNC_MODE BDOT_MODE")

end

def adcs_inertial()

    cmd("GENERIC_ADCS GENERIC_ADCS_SET_MODE_CC with GNC_MODE INERTIAL_MODE")

end

def adcs_set_q()

    cmd("GENERIC_ADCS GENERIC_ADCS_INERTIAL_QUATERNION_CC with GNC_INER_QUAT1 0.0, GNC_INER_QUAT2 0.0, GNC_INER_QUAT3 0.0, GNC_INER_QUAT4 1.0")

end

def safe_adcs()

    cmd("GENERIC_FSS GENERIC_FSS_REQ_HK")
    fss_enabled = tlm("GENERIC_FSS GENERIC_FSS_HK_TLM DEVICE_ENABLED")
    if (fss_enabled != "ENABLED")
        cmd("GENERIC_FSS GENERIC_FSS_ENABLE_CC")
    end

    cmd("GENERIC_CSS GENERIC_CSS_REQ_HK")
    css_enabled = tlm("GENERIC_CSS GENERIC_CSS_HK_TLM DEVICE_ENABLED")
    if (css_enabled != "ENABLED")
        cmd("GENERIC_CSS GENERIC_CSS_ENABLE_CC")
    end

    cmd("GENERIC_IMU GENERIC_IMU_REQ_HK")
    imu_enabled = tlm("GENERIC_IMU GENERIC_IMU_HK_TLM DEVICE_ENABLED")
    if (imu_enabled != "ENABLED")
        cmd("GENERIC_IMU GENERIC_IMU_ENABLE_CC")
    end

    cmd("GENERIC_MAG GENERIC_MAG_REQ_HK")
    mag_enabled = tlm("GENERIC_MAG GENERIC_MAG_HK_TLM DEVICE_ENABLED")
    if (mag_enabled != "ENABLED")
        cmd("GENERIC_MAG GENERIC_MAG_ENABLE_CC")
    end

    sw1_state = tlm("GENERIC_EPS GENERIC_EPS_HK_TLM SWITCH_1_STATE")
    if(sw1_state == "OFF")
        eps_cmd("GENERIC_EPS GENERIC_EPS_SWITCH_CC with SWITCH_NUMBER SWITCH_1, STATE ON")
    end

    cmd("GENERIC_STAR_TRACKER GENERIC_STAR_TRACKER_REQ_HK")
    st_enabled = tlm("GENERIC_STAR_TRACKER GENERIC_STAR_TRACKER_HK_TLM DEVICE_ENABLED")
    if (st_enabled != "ENABLED")
        cmd("GENERIC_STAR_TRACKER GENERIC_STAR_TRACKER_ENABLE_CC")
    end

    cmd("NOVATEL_OEM615 NOVATEL_OEM615_REQ_HK")
    gps_enabled = tlm("NOVATEL_OEM615 NOVATEL_OEM615_HK_TLM DEVICE_ENABLED")
    if (gps_enabled != "ENABLED")
        cmd("NOVATEL_OEM615 NOVATEL_OEM615_ENABLE_CC")
    end

    cmd("GENERIC_TORQUER GENERIC_TORQUER_REQ_HK_CC")
    torquer_enabled = tlm("GENERIC_TORQUER GENERIC_TORQUER_HK_TLM_T DEVICE_ENABLED")
    if (torquer_enabled != "ENABLED")
        cmd("GENERIC_TORQUER GENERIC_TORQUER_ENABLE_CC")
    end

    get_adcs_data()
    mode = tlm("GENERIC_ADCS GENERIC_ADCS_GNC MODE")
    if (mode != "SUNSAFE")
        adcs_sunsafe()
    end
end

def confirm_adcs_data()
    
    adcs_sunsafe()

    diff = 0.05

    get_adcs_data()
    adcs_sun_valid = tlm("GENERIC_ADCS GENERIC_ADCS_GNC SUN_VALID")
    if adcs_sun_valid == 0
        wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC SVB_X", 1.0, diff, GENERIC_ADCS_MODE_CHECK_TIMEOUT)
        wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC SVB_Y", 0.0, diff, GENERIC_ADCS_MODE_CHECK_TIMEOUT)
        wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC SVB_Z", 0.0, diff, GENERIC_ADCS_MODE_CHECK_TIMEOUT)

        diff = 50
        wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC CSS_0", 1000, diff, GENERIC_ADCS_MODE_CHECK_TIMEOUT)
    end

    cmd("GENERIC_IMU GENERIC_IMU_REQ_DATA")

    x_ang_rate_before = tlm("GENERIC_IMU GENERIC_IMU_DATA_TLM X_ANGULAR_RATE").abs
    y_ang_rate_before = tlm("GENERIC_IMU GENERIC_IMU_DATA_TLM Y_ANGULAR_RATE").abs
    z_ang_rate_before = tlm("GENERIC_IMU GENERIC_IMU_DATA_TLM Z_ANGULAR_RATE").abs

    adcs_bdot()

    diff = 0.05
    
    wait_check("GENERIC_IMU GENERIC_IMU_DATA_TLM X_ANGULAR_RATE < #{x_ang_rate_before}", GENERIC_ADCS_MODE_CHECK_TIMEOUT)
    wait_check("GENERIC_IMU GENERIC_IMU_DATA_TLM Y_ANGULAR_RATE < #{y_ang_rate_before}", GENERIC_ADCS_MODE_CHECK_TIMEOUT)
    wait_check("GENERIC_IMU GENERIC_IMU_DATA_TLM Z_ANGULAR_RATE < #{z_ang_rate_before}", GENERIC_ADCS_MODE_CHECK_TIMEOUT)

    adcs_inertial()
    adcs_set_q();

    diff = 0.05

    sleep(GENERIC_ADCS_MODE_CHECK_TIMEOUT)

    qbn0 = tlm("GENERIC_ADCS GENERIC_ADCS_GNC QBN_0").abs
    qbn1 = tlm("GENERIC_ADCS GENERIC_ADCS_GNC QBN_1").abs
    qbn2 = tlm("GENERIC_ADCS GENERIC_ADCS_GNC QBN_2").abs
    qbn3 = tlm("GENERIC_ADCS GENERIC_ADCS_GNC QBN_3").abs

    wait_check_expression("#{qbn0} < 0.05 or #{qbn0} > -0.05", GENERIC_ADCS_MODE_CHECK_TIMEOUT)
    wait_check_expression("#{qbn1} < 0.05 or #{qbn1} > -0.05", GENERIC_ADCS_MODE_CHECK_TIMEOUT)
    wait_check_expression("#{qbn2} < 0.05 or #{qbn2} > -0.05", GENERIC_ADCS_MODE_CHECK_TIMEOUT)
    wait_check_expression("#{qbn3} < 1.05 or #{qbn3} > 0.95", GENERIC_ADCS_MODE_CHECK_TIMEOUT)

    get_adcs_hk()
end

def confirm_adcs_data_loop()
    GENERIC_ADCS_DEVICE_LOOP_COUNT.times do |n|
        confirm_adcs_data()
    end
end
