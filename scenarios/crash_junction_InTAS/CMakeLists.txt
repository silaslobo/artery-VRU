add_opp_run(crash_InTAS WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(WITH_ENVMOD)
    add_opp_test(crash_InTAS SUFFIX envmod CONFIG envmod SIMTIME_LIMIT 20s)
endif()
