add_library(laser INTERFACE)

target_sources(laser INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/coolant.c
 ${CMAKE_CURRENT_LIST_DIR}/lb_clusters.c
 ${CMAKE_CURRENT_LIST_DIR}/ppi.c
)

target_include_directories(laser INTERFACE ${CMAKE_CURRENT_LIST_DIR})
