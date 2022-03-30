add_opp_run(roundabout_InTAS WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(WITH_ENVMOD)
    add_opp_test(roundabout_InTAS SUFFIX envmod CONFIG envmod SIMTIME_LIMIT 40s)
endif()
