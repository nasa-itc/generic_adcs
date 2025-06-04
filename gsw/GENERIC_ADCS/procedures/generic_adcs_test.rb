require 'cosmos'
require 'cosmos/script'
require 'generic_adcs_lib.rb'
require 'generic_css_test.rb'
require 'generic_fss_test.rb'
require 'generic_imu_test.rb'
require 'generic_mag_test.rb'
require 'generic_rw_test.rb'
require 'generic_st_test.rb'
require 'generic_torquer_test.rb'
require 'novatel_oem615_test.rb'

class GENERIC_ADCS_Functional_Test < Cosmos::Test
  def setup
    safe_adcs()
  end

  def test_application
      start("tests/generic_adcs_app_test.rb")
  end

  def test_sensors_basic
    adcs_passive()
    
    

    safe_adcs()
  end

  def test_actuators_basic
    adcs_passive()
    
    safe_adcs()
  end

  def test_sensors_vs_adcs
    adcs_passive()
    
    

    safe_adcs()
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
