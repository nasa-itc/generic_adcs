from openc3.script import *
from openc3.test import *
from generic_adcs_lib import *
# from generic_css_lib import *
# from generic_fss_lib import *
# from generic_imu_lib import *
# from generic_mag_lib import *
# from generic_reaction_wheel_lib import *
# from generic_st_lib import *
# from gps_lib import *

class GENERIC_ADCS_Functional_Test(Test):
  def setup(self):
    safe_adcs()

  def test_application(self):
      start("tests/generic_adcs_app_test.py")

  def test_inputs(self):
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

  def test_outputs(self):

    safe_adcs()

    adcs_confirm_rw_data()

    safe_adcs()

  def teardown(self):
    safe_adcs()

class Generic_adcs_Test(TestSuite):
  def __init__(self):
      super().__init__()
      self.add_test('GENERIC_ADCS_Functional_Test')

  def setup(self):
    safe_adcs()
  
  def teardown(self):
    safe_adcs()