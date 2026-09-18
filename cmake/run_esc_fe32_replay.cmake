foreach(required_var
        REPLAY_EXE
        INPUT
        EXPECTED_SHA256
        EXPECTED_BYTES
        EXPECTED_FRAMES
        EXPECTED_CRC_FAILURES
        EXPECTED_PREFIX_REJECTIONS)
    if(NOT DEFINED ${required_var} OR "${${required_var}}" STREQUAL "")
        message(FATAL_ERROR "missing required variable: ${required_var}")
    endif()
endforeach()

if(NOT EXISTS "${REPLAY_EXE}")
    message(FATAL_ERROR "replay executable does not exist: ${REPLAY_EXE}")
endif()

if(NOT EXISTS "${INPUT}")
    message(FATAL_ERROR "required FE32 replay capture does not exist: ${INPUT}")
endif()

file(SHA256 "${INPUT}" actual_sha256)
string(TOLOWER "${actual_sha256}" actual_sha256)
string(TOLOWER "${EXPECTED_SHA256}" expected_sha256)

if(NOT actual_sha256 STREQUAL expected_sha256)
    message(FATAL_ERROR
        "FE32 replay capture SHA256 mismatch for ${INPUT}: "
        "expected ${expected_sha256}, got ${actual_sha256}")
endif()

execute_process(
    COMMAND "${REPLAY_EXE}"
            "${INPUT}"
            "${EXPECTED_BYTES}"
            "${EXPECTED_FRAMES}"
            "${EXPECTED_CRC_FAILURES}"
            "${EXPECTED_PREFIX_REJECTIONS}"
    RESULT_VARIABLE replay_result
    OUTPUT_VARIABLE replay_stdout
    ERROR_VARIABLE replay_stderr
)

if(NOT replay_result EQUAL 0)
    message(FATAL_ERROR
        "FE32 replay failed for ${INPUT} with exit code ${replay_result}\n"
        "stdout:\n${replay_stdout}\n"
        "stderr:\n${replay_stderr}")
endif()

string(STRIP "${replay_stdout}" replay_stdout)
message(STATUS "${replay_stdout}")
