require 'cosmos'
require 'cosmos/script'
require 'generic_adcs_lib'

class GENERIC_ADCS_Functional_Test < Cosmos::Test
  def setup
    enable_adcs()
  end

  def test_algorithms
    enable_adcs()
    goto_sunsafe()
    goto_startracker_not_at_earth()
    goto_inertial()
  end

  def test_inputs
    adcs_confirm_css_data()
    adcs_confirm_imu_data()
    adcs_confirm_mag_data()
    # Go to inertial mode to ensure the star tracker is valid
    goto_inertial()
    adcs_confirm_st_data()
    # Go to sunsafe mode to ensure the FSS is pointed at the sun
    goto_sunsafe()
    adcs_confirm_fss_data()
  end
  
  def test_outputs
    adcs_confirm_rw_data()
  end

  def teardown
  end
end

class Generic_ADCS_Test < Cosmos::TestSuite
  def initialize
      super()
      add_test('GENERIC_ADCS_Functional_Test')
  end

  def setup
    enable_adcs()
  end
  
  def teardown
  end
end
