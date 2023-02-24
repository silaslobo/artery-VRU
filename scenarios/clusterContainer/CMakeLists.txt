add_opp_run(clusterContainer WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(WITH_ENVMOD)
    add_opp_test(clusterContainer SUFFIX envmod CONFIG envmod SIMTIME_LIMIT 40s)
endif()
