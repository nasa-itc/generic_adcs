import sys
import glob

for p in glob.glob('/gems/gems/openc3-cosmos-nos3-*/targets/GENERIC_ADCS/scripts'):
    if p not in sys.path:
        sys.path.append(p)

from openc3.script import cmd, tlm, check, wait_check
from nos3.generic_adcs_lib import *

##
## This script tests the standard cFS component application functionality.
## Currently this includes: 
##   Housekeeping, request telemetry to be published on the software bus
##   NOOP, no operation but confirm correct counters increment
##   Reset counters, increment as done in NOOP and confirm ability to clear repeatably
##   Invalid ground command, confirm bad lengths and codes are rejected
##

def run_generic_adcs_app_test():
    # Get to known state
    safe_adcs()

    ##
    ##   Housekeeping, request telemetry to be published on the software bus
    ##
    for n in range(GENERIC_ADCS_TEST_LOOP_COUNT):
        get_adcs_hk()

    ##
    ## NOOP, no operation but confirm correct counters increment
    ##
    for n in range(GENERIC_ADCS_TEST_LOOP_COUNT):
        adcs_cmd("GENERIC_ADCS_DEBUG GENERIC_ADCS_NOOP_CC")

    ##
    ## Reset counters, increment as done in NOOP and confirm ability to clear repeatably
    ##
    for n in range(GENERIC_ADCS_TEST_LOOP_COUNT):
        adcs_cmd("GENERIC_ADCS_DEBUG GENERIC_ADCS_NOOP_CC")
        cmd("GENERIC_ADCS_DEBUG GENERIC_ADCS_RST_COUNTERS_CC") # Note standard `cmd` as we can't reset counters and then confirm increment
        get_adcs_hk()
        wait_check("GENERIC_ADCS_DEBUG GENERIC_ADCS_HK_TLM CMD_COUNT == 0", 15)
        wait_check("GENERIC_ADCS_DEBUG GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == 0", 15)

    ##
    ##   Invalid ground command, confirm bad lengths and codes are rejected
    ##
    for n in range(GENERIC_ADCS_TEST_LOOP_COUNT):
        # Bad length
        cmd_cnt = tlm("GENERIC_ADCS_DEBUG GENERIC_ADCS_HK_TLM CMD_COUNT")
        cmd_err_cnt = tlm("GENERIC_ADCS_DEBUG GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")
        cmd(f"GENERIC_ADCS_DEBUG GENERIC_ADCS_NOOP_CC with CCSDS_LENGTH {n+2}") # Note +2 due to CCSDS already being +1
        get_adcs_hk()
        # check(f"GENERIC_ADCS_DEBUG GENERIC_ADCS_HK_TLM CMD_COUNT == {cmd_cnt}")
        # check(f"GENERIC_ADCS_DEBUG GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == {cmd_err_cnt+1}")

    # Equivalent to Ruby's for n in 6..(5 + GENERIC_ADCS_TEST_LOOP_COUNT)
    for n in range(6, 6 + GENERIC_ADCS_TEST_LOOP_COUNT):
        # Bad command codes
        cmd_cnt = tlm("GENERIC_ADCS_DEBUG GENERIC_ADCS_HK_TLM CMD_COUNT")
        cmd_err_cnt = tlm("GENERIC_ADCS_DEBUG GENERIC_ADCS_HK_TLM CMD_ERR_COUNT")
        cmd(f"GENERIC_ADCS_DEBUG GENERIC_ADCS_NOOP_CC with CCSDS_FC {n+1}")
        get_adcs_hk()
        # check(f"GENERIC_ADCS_DEBUG GENERIC_ADCS_HK_TLM CMD_COUNT == {cmd_cnt}")
        # check(f"GENERIC_ADCS_DEBUG GENERIC_ADCS_HK_TLM CMD_ERR_COUNT == {cmd_err_cnt+1}")

    for n in range(GENERIC_ADCS_TEST_LOOP_COUNT):
        #test data makes sense against truth
        safe_adcs()
        get_adcs_data()
        confirm_adcs_data()