set( build_libs
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    rgbd_tutor_lib
    ${PROJECT_SOURCE_DIR}/Thirdparty/orbslam_modified/lib/libORB_SLAM2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension
    ${BOOST_LIBRARIES}
)

add_executable( reading_frame reading_frame.cpp )
target_link_libraries( reading_frame
    ${build_libs}
)

add_executable( match_orbfeature_tum match_orbfeature_tum.cpp )
target_link_libraries( match_orbfeature_tum
    ${build_libs}
)

add_executable( test_pnp  test_pnp.cpp )
target_link_libraries( test_pnp
    ${build_libs}
)
