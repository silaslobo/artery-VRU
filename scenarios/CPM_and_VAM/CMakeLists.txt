add_opp_run(cpmANDvam WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(WITH_ENVMOD)
    add_opp_test(cpmANDvam SUFFIX envmod CONFIG envmod SIMTIME_LIMIT 100s)
endif()
