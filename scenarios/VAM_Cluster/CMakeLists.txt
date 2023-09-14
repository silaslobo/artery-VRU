add_opp_run(VRU_Clustering WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(WITH_ENVMOD)
    add_opp_test(VRU_Clustering SUFFIX envmod CONFIG envmod SIMTIME_LIMIT 20s)
endif()