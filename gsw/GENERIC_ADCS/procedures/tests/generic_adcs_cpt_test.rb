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
## GNC Data
##
initial_error_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")

cmd("GENERIC_ADCS GENERIC_ADCS_SEND_GNC_CC")

svb0 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA SVB_0")
svb1 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA SVB_1")
svb2 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA SVB_2")

adcs_svbx = tlm("GENERIC_ADCS GENERIC_ADCS_GNC SVB_X")
adcs_svby = tlm("GENERIC_ADCS GENERIC_ADCS_GNC SVB_Y")
adcs_svbz = tlm("GENERIC_ADCS GENERIC_ADCS_GNC SVB_Z")
adcs_sun_valid = tlm("GENERIC_ADCS GENERIC_ADCS_GNC SUN_VALID")

truth_42_svbx_diff = (adcs_svbx - svb0).abs()
truth_42_svby_diff = (adcs_svby - svb1).abs()
truth_42_svbz_diff = (adcs_svbz - svb2).abs()
diff_margin = 0.03

wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

if adcs_sun_valid == 0
  wait_check_expression("truth_42_svbx_diff <= diff_margin # #{truth_42_svbx_diff} <= #{diff_margin}", 15)

  wait_check_expression("truth_42_svby_diff <= diff_margin # #{truth_42_svby_diff} <= #{diff_margin}", 15)

  wait_check_expression("truth_42_svbz_diff <= diff_margin # #{truth_42_svbz_diff} <= #{diff_margin}", 15)
end

sleep(5)

##
## Reset Counters
##
cmd("GENERIC_ADCS GENERIC_ADCS_RST_COUNTERS_CC")
wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_COUNT == 0", 30)
wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == 0", 30)