require 'cosmos'
require 'cosmos/script'
require "cfs_lib.rb"
#require 'math'

##
## NOOP
##
initial_command_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_COUNT")
initial_error_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")
cmd("GENERIC_ADCS GENERIC_ADCS_NOOP_CC")
wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_COUNT > #{initial_command_count}", 30)
wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

sleep(5)

##
## Housekeeping
##
initial_error_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")
cmd("GENERIC_ADCS GENERIC_ADCS_REQ_HK")
wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

sleep(5)

##
## GNC SVB Data
##
initial_error_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")

cmd("GENERIC_ADCS GENERIC_ADCS_SEND_GNC_CC")

adcs_sun_valid = tlm("GENERIC_ADCS GENERIC_ADCS_GNC SUN_VALID")

wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

if adcs_sun_valid == 0
  wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC SVB_X", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA SVB_0"), diff, 30)
  wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC SVB_Y", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA SVB_1"), diff, 30)
  wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC SVB_Z", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA SVB_2"), diff, 30)
end

sleep(5)

##
## DI RW Data
##
initial_error_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")

cmd("GENERIC_ADCS GENERIC_ADCS_SEND_DI_CC")

diff = 0.03

wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_DI RW_MOMENTUM_X", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA WN_0"), diff, 30)
wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_DI RW_MOMENTUM_Y", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA WN_1"), diff, 30)
wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_DI RW_MOMENTUM_Z", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA WN_2"), diff, 30)

sleep(5)

##
## DI BVB Data
##
initial_error_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")

cmd("GENERIC_ADCS GENERIC_ADCS_SEND_DI_CC")

diff = 0.03

wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_DI BVB_X", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA BVB_0"), diff, 30)
wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_DI BVB_Y", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA BVB_1"), diff, 30)
wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_DI BVB_Z", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA BVB_2"), diff, 30)

sleep(5)

# ##
# ## GNC TCMD Data (TODO: Figure out Truth Source)
# ##
# initial_error_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")

# cmd("GENERIC_ADCS GENERIC_ADCS_SEND_DI_CC")

# bvb0 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA TCMD_0")
# bvb1 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA TCMD_1")
# bvb2 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA TCMD_2")

# adcs_tcmdx = tlm("GENERIC_ADCS GENERIC_ADCS_GNC TCMD_X")
# adcs_tcmdy = tlm("GENERIC_ADCS GENERIC_ADCS_GNC TCMD_Y")
# adcs_tcmdz = tlm("GENERIC_ADCS GENERIC_ADCS_GNC TCMD_Z")

# truth_42_tcmdx_diff = (adcs_tcmdx - bvb0).abs()
# truth_42_tcmdy_diff = (adcs_tcmdy - bvb1).abs()
# truth_42_tcmdz_diff = (adcs_tcmdz - bvb2).abs()
# diff_margin = 0.03

# wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

# wait_check_expression("truth_42_tcmdx_diff <= diff_margin # #{truth_42_tcmdx_diff} <= #{diff_margin}", 15)

# wait_check_expression("truth_42_tcmdy_diff <= diff_margin # #{truth_42_tcmdy_diff} <= #{diff_margin}", 15)

# wait_check_expression("truth_42_tcmdz_diff <= diff_margin # #{truth_42_tcmdz_diff} <= #{diff_margin}", 15)

# sleep(5)

##
## GNC QBN Data
##
initial_error_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")

cmd("GENERIC_ADCS GENERIC_ADCS_SEND_GNC_CC")

diff = 1

wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC QBN_0", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_0"), diff, 30)
wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC QBN_1", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_1"), diff, 30)
wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC QBN_2", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_2"), diff, 30)
wait_check_tolerance("GENERIC_ADCS GENERIC_ADCS_GNC QBN_3", tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_3"), diff, 30)

sleep(5)

##
## Reset Counters
##
cmd("GENERIC_ADCS GENERIC_ADCS_RST_COUNTERS_CC")
wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_COUNT == 0", 30)
wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == 0", 30)