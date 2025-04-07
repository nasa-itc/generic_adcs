require 'cosmos'
require 'cosmos/script'
require 'generic_adcs_lib.rb'

class GENERIC_ADCS_Functional_Test < Cosmos::Test
  def setup
    safe_adcs()
  end

  def test_application
      start("tests/generic_adcs_app_test.rb")
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
