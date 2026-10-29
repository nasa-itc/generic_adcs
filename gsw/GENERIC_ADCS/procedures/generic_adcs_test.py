import sys
import glob

for p in glob.glob('/gems/gems/openc3-cosmos-nos3-*/targets/GENERIC_ADCS/scripts'):
    if p not in sys.path:
        sys.path.append(p)

from openc3.script import cmd, tlm, check
from openc3.script.suite import Suite, Group

try:
    from nos3.generic_adcs_lib import safe_adcs, adcs_confirm_css_data, adcs_confirm_fss_data, adcs_confirm_imu_data, adcs_confirm_mag_data, adcs_confirm_st_data, adcs_confirm_rw_data
    from nos3.generic_adcs_app_test import run_generic_adcs_app_test 
except ImportError:
    pass

class GENERIC_ADCS_Functional_Test(Group):
    def setup(self):
        safe_adcs()

    def script_application(self):
        run_generic_adcs_app_test()

    def script_inputs(self):
        safe_adcs()
        adcs_confirm_css_data()
        safe_adcs()
        adcs_confirm_fss_data()
        safe_adcs()
        adcs_confirm_imu_data()
        safe_adcs()
        adcs_confirm_mag_data()
        safe_adcs()
        adcs_confirm_st_data()
        safe_adcs()

    def script_outputs(self):
        safe_adcs()
        adcs_confirm_rw_data()
        safe_adcs()

    def teardown(self):
        safe_adcs()

class Generic_adcs_Test(Suite):
    def __init__(self):
        super().__init__()
        self.add_group(GENERIC_ADCS_Functional_Test)

    def setup(self):
        safe_adcs()
  
    def teardown(self):
        safe_adcs()