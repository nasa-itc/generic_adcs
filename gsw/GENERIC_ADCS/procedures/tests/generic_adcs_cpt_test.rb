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
## DI RW Data
##
initial_error_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")

cmd("GENERIC_ADCS GENERIC_ADCS_SEND_DI_CC")

wn0 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA WN_0")
wn1 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA WN_1")
wn2 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA WN_2")

adcs_rw_momentumx = tlm("GENERIC_ADCS GENERIC_ADCS_DI RW_MOMENTUM_X")
adcs_rw_momentumy = tlm("GENERIC_ADCS GENERIC_ADCS_DI RW_MOMENTUM_Y")
adcs_rw_momentumz = tlm("GENERIC_ADCS GENERIC_ADCS_DI RW_MOMENTUM_Z")

truth_42_rwx_diff = (adcs_rw_momentumx - wn0).abs()
truth_42_rwy_diff = (adcs_rw_momentumy - wn1).abs()
truth_42_rwz_diff = (adcs_rw_momentumz - wn2).abs()
diff_margin = 0.03

wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

wait_check_expression("truth_42_rwx_diff <= diff_margin # #{truth_42_rwx_diff} <= #{diff_margin}", 15)

wait_check_expression("truth_42_rwy_diff <= diff_margin # #{truth_42_rwy_diff} <= #{diff_margin}", 15)

wait_check_expression("truth_42_rwz_diff <= diff_margin # #{truth_42_rwz_diff} <= #{diff_margin}", 15)

sleep(5)

##
## DI BVB Data
##
initial_error_count = tlm("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")

cmd("GENERIC_ADCS GENERIC_ADCS_SEND_DI_CC")

bvb0 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA BVB_0")
bvb1 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA BVB_1")
bvb2 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA BVB_2")

adcs_bvbx = tlm("GENERIC_ADCS GENERIC_ADCS_DI BVB_X")
adcs_bvby = tlm("GENERIC_ADCS GENERIC_ADCS_DI BVB_Y")
adcs_bvbz = tlm("GENERIC_ADCS GENERIC_ADCS_DI BVB_Z")

truth_42_bvbx_diff = (adcs_bvbx - bvb0).abs()
truth_42_bvby_diff = (adcs_bvby - bvb1).abs()
truth_42_bvbz_diff = (adcs_bvbz - bvb2).abs()
diff_margin = 0.03

wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

wait_check_expression("truth_42_bvbx_diff <= diff_margin # #{truth_42_bvbx_diff} <= #{diff_margin}", 15)

wait_check_expression("truth_42_bvby_diff <= diff_margin # #{truth_42_bvby_diff} <= #{diff_margin}", 15)

wait_check_expression("truth_42_bvbz_diff <= diff_margin # #{truth_42_bvbz_diff} <= #{diff_margin}", 15)

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

qn0 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_0")
qn1 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_1")
qn2 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_2")
qn3 = tlm("SIM_42_TRUTH SIM_42_TRUTH_DATA QN_3")

adcs_qbn0 = tlm("GENERIC_ADCS GENERIC_ADCS_GNC QBN_0")
adcs_qbn1 = tlm("GENERIC_ADCS GENERIC_ADCS_GNC QBN_1")
adcs_qbn2 = tlm("GENERIC_ADCS GENERIC_ADCS_GNC QBN_2")
adcs_qbn3 = tlm("GENERIC_ADCS GENERIC_ADCS_GNC QBN_3")

truth_42_qn0_diff = (adcs_qbn0 - qn0).abs()
truth_42_qn1_diff = (adcs_qbn1 - qn1).abs()
truth_42_qn2_diff = (adcs_qbn2 - qn2).abs()
truth_42_qn3_diff = (adcs_qbn3 - qn3).abs()
diff_margin = 0.03

wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == #{initial_error_count}", 30)

wait_check_expression("truth_42_qn0_diff <= diff_margin # #{truth_42_qn0_diff} <= #{diff_margin}", 15)

wait_check_expression("truth_42_qn1_diff <= diff_margin # #{truth_42_qn1_diff} <= #{diff_margin}", 15)

wait_check_expression("truth_42_qn2_diff <= diff_margin # #{truth_42_qn2_diff} <= #{diff_margin}", 15)

wait_check_expression("truth_42_qn3_diff <= diff_margin # #{truth_42_qn3_diff} <= #{diff_margin}", 15)

sleep(5)

##
## Reset Counters
##
cmd("GENERIC_ADCS GENERIC_ADCS_RST_COUNTERS_CC")
wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_COUNT == 0", 30)
wait_check("GENERIC_ADCS GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == 0", 30)