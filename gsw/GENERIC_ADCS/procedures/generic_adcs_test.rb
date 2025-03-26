require 'cosmos'
require 'cosmos/script'
require 'mission_lib.rb'

class ADCS_LPT < Cosmos::Test
  def setup
      enable_TO_and_verify()
  end

  def test_lpt
    start("tests/generic_adcs_lpt_test.rb")
  end

  def teardown
      cmd("CFS_RADIO TO_PAUSE_OUTPUT")
  end
end

class ADCS_CPT < Cosmos::Test
  def setup
      
  end

  def test_cpt
    start("tests/generic_adcs_cpt_test.rb")
  end

  def teardown

  end
end

class Generic_adcs_Test < Cosmos::TestSuite
  def initialize
      super()
      add_test('ADCS_CPT')
      add_test('ADCS_LPT')
  end

  def setup
  end
  
  def teardown
  end
end