cmake_minimum_required (VERSION 2.8.12)
project (motoros C)

# NOTE: order probably matters: mpBuilder puts application dir before
# main MotoPlus include dir
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${MOTOPLUS_INCLUDE_DIRECTORIES})

set (motoros_SRCS
  Controller.c
  CtrlGroup.c
  IoServer.c
  MotionServer.c
  mpMain.c
  SimpleMessage.c
  StateServer.c
)

# additional motoros libs to link against (these are controller specific)
add_library(ParameterExtraction STATIC IMPORTED)
set_property(TARGET ParameterExtraction PROPERTY IMPORTED_LOCATION
  "${CMAKE_CURRENT_LIST_DIR}/ParameterExtraction.${MOTOPLUS_LIB_EXTENSION}")

# no longer used
# add_library(RosSetupValidation STATIC IMPORTED)
# set_property(TARGET RosSetupValidation PROPERTY IMPORTED_LOCATION
#   "${CMAKE_CURRENT_LIST_DIR}/RosSetupValidation.${MOTOPLUS_LIB_EXTENSION}")

# build the binary itself
add_executable(motoros ${motoros_SRCS})
target_link_libraries(motoros ParameterExtraction) # RosSetupValidation)
