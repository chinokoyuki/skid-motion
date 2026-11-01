add_executable(skid_motion_tests test_skid_motion.c)
target_link_libraries(skid_motion_tests PRIVATE skid::motion skid::sim)
add_test(NAME skid_motion_tests COMMAND skid_motion_tests)
