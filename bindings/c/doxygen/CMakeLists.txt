cmake_minimum_required(VERSION 3.20)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(RapierCDocs LANGUAGES C)
endif()
find_package(Doxygen 1.9.4 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(RAPIER_C_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
file(READ "${RAPIER_C_DIR}/VERSION" DOC_VERSION)
string(STRIP "${DOC_VERSION}" DOC_VERSION)
set(DOC_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/html")
file(MAKE_DIRECTORY "${DOC_OUTPUT}")
configure_file(index.html.in "${DOC_OUTPUT}/index.html" @ONLY)
add_custom_target(rapier_docs)
foreach(DOC_DIM 2 3)
  foreach(DOC_PRECISION 32 64)
    set(DOC_VARIANT "${DOC_DIM}d-f${DOC_PRECISION}")
    set(DOC_ROBOTICS "")
    if(DOC_DIM EQUAL 3 AND DOC_PRECISION EQUAL 32)
      set(DOC_ROBOTICS "RAPIER_ROBOTICS")
    endif()
    configure_file(Doxyfile.in "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile-${DOC_VARIANT}" @ONLY)
    add_custom_target(rapier_docs_${DOC_VARIANT}
      COMMAND "${DOXYGEN_EXECUTABLE}" "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile-${DOC_VARIANT}"
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check.py"
        "${DOC_OUTPUT}/${DOC_VARIANT}/xml" "${DOC_DIM}" "${DOC_PRECISION}"
        "${CMAKE_C_COMPILER}" "${CMAKE_C_COMPILER_ID}"
      WORKING_DIRECTORY "${RAPIER_C_DIR}"
      COMMENT "Generating and checking ${DOC_VARIANT} API reference"
      VERBATIM)
    add_dependencies(rapier_docs rapier_docs_${DOC_VARIANT})
  endforeach()
endforeach()
