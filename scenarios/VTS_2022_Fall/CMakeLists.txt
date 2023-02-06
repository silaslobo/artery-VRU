add_opp_run(VTS_Fall_22 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(WITH_ENVMOD)
    add_opp_test(VTS_Fall_22 SUFFIX envmod CONFIG envmod SIMTIME_LIMIT 40s)
endif()
