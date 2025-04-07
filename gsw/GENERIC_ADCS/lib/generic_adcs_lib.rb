# Library for GENERIC_ADCS Target
require 'cosmos'
require 'cosmos/script'

#
# Definitions
#
GENERIC_ADCS_CMD_SLEEP = 0.25
GENERIC_ADCS_RESPONSE_TIMEOUT = 5
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

def safe_adcs()
    get_adcs_data()
    mode = tlm("GENERIC_ADCS GENERIC_ADCS_GNC MODE")
    if (mode != "SUNSAFE")
        adcs_sunsafe()
    end
end

def confirm_adcs_data()
    
    get_adcs_data()
    adcs_sun_valid = tlm("GENERIC_ADCS GENERIC_ADCS_GNC SUN_VALID")

    diff = 0.03

    if adcs_sun_valid == 0
    wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC SVB_X", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA SVB_0"), diff, GENERIC_ADCS_RESPONSE_TIMEOUT)
    wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC SVB_Y", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA SVB_1"), diff, GENERIC_ADCS_RESPONSE_TIMEOUT)
    wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC SVB_Z", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA SVB_2"), diff, GENERIC_ADCS_RESPONSE_TIMEOUT)
    end

    diff = 0.05

    wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_DI RW_MOMENTUM_X", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA WN_0"), diff, GENERIC_ADCS_RESPONSE_TIMEOUT)
    wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_DI RW_MOMENTUM_Y", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA WN_1"), diff, GENERIC_ADCS_RESPONSE_TIMEOUT)
    wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_DI RW_MOMENTUM_Z", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA WN_2"), diff, GENERIC_ADCS_RESPONSE_TIMEOUT)

    diff = 0.05

    wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC QBN_0", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_0"), diff, GENERIC_ADCS_RESPONSE_TIMEOUT)
    wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC QBN_1", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_1"), diff, GENERIC_ADCS_RESPONSE_TIMEOUT)
    wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC QBN_2", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_2"), diff, GENERIC_ADCS_RESPONSE_TIMEOUT)
    wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC QBN_3", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_3"), diff, GENERIC_ADCS_RESPONSE_TIMEOUT)


    get_adcs_hk()
end

def confirm_adcs_data_loop()
    GENERIC_ADCS_DEVICE_LOOP_COUNT.times do |n|
        confirm_adcs_data()
    end
end
