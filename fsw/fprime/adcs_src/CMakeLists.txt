####
# F prime CMakeLists.txt:
#
# SOURCE_FILES: combined list of source and autocoding files
# MOD_DEPS: (optional) module dependencies
# UT_SOURCE_FILES: list of source files for unit tests
#
####

include_directories("../../../../../fsw/apps/hwlib/fsw/public_inc")
include_directories("../src")
include_directories("../../shared")

set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Generic_adcs.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Generic_adcs.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../shared/generic_adcs_utilities.c"
  "${CMAKE_CURRENT_LIST_DIR}/../src/generic_adcs_ingest.c"
  "${CMAKE_CURRENT_LIST_DIR}/../../shared/generic_adcs_adac.c"
)

# Uncomment and add any modules that this component depends on, else
# they might not be available when cmake tries to build this component.

set(MOD_DEPS
    ${ITC_Common_LIBRARIES}
    ${NOSENGINE_LIBRARIES}
)

register_fprime_module()
