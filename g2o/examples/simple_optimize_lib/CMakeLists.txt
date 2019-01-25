include_directories(${CSPARSE_INCLUDE_DIR})

add_library(simple_optimize_lib SHARED
  simple_optimize_lib.cpp
)

# set_target_properties(simple_optimize PROPERTIES OUTPUT_NAME simple_optimize${EXE_POSTFIX})

target_link_libraries(simple_optimize_lib core solver_csparse)
target_link_libraries(simple_optimize_lib types_slam2d)
target_link_libraries(simple_optimize_lib types_slam3d)
