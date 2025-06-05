require 'cosmos'
require 'cosmos/script'
require 'generic_adcs_lib.rb'
require 'generic_css_lib.rb'
require 'generic_fss_lib.rb'
require 'generic_imu_lib.rb'
require 'generic_mag_lib.rb'
require 'generic_reaction_wheel_lib.rb'
require 'generic_st_lib.rb'
require 'generic_torquer_lib.rb'
require 'gps_lib.rb'

class GENERIC_ADCS_Functional_Test < Cosmos::Test
  def setup
    safe_adcs()
  end

  def test_application
      start("tests/generic_adcs_app_test.rb")
  end

  def test_sensors
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
  end

  def test_actuators



  end

  def teardown
    safe_adcs()
  end
end

class Generic_adcs_Test < Cosmos::TestSuite
  def initialize
      super()
      add_test('GENERIC_ADCS_Functional_Test')
  end

  def setup
    safe_adcs()
  end
  
  def teardown
    safe_adcs()
  end
end
