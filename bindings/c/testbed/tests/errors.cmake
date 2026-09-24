foreach(phase setup frame)
  execute_process(COMMAND "${EXECUTABLE}" "${phase}"
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE diagnostic)
  if(NOT result STREQUAL "1" OR NOT diagnostic MATCHES "Rapier error in error-test.*positive" OR
      diagnostic MATCHES "continued after failed call")
    message(FATAL_ERROR "Unexpected ${phase} error behavior: ${result}\n${output}\n${diagnostic}")
  endif()
endforeach()
