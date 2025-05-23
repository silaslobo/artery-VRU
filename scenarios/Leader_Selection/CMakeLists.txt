add_opp_run(Leader_Selection WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(WITH_ENVMOD)
    add_opp_test(Leader_Selection SUFFIX envmod CONFIG envmod SIMTIME_LIMIT 20s)
endif()