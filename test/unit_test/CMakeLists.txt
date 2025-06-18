 
cmake_minimum_required(VERSION 3.1.0)

SET(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)

find_package(Threads)

find_package(GTest CONFIG REQUIRED)

include(CTest)

include_directories(${GTEST_INCLUDE_DIRS})  # Var managed by CMake
include(GoogleTest)
add_executable(GTest_HighLevel GTest_HighLevel.cpp)
target_link_libraries(GTest_HighLevel gtest gtest_main)
target_link_libraries(GTest_HighLevel  CloCK ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

add_executable(GTest_AdjustGoal GTest_AdjustGoal.cpp)
target_link_libraries(GTest_AdjustGoal gtest gtest_main)
target_link_libraries(GTest_AdjustGoal  CloCK ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

add_executable(GTest_SplitTask GTest_SplitTask.cpp)
target_link_libraries(GTest_SplitTask gtest gtest_main)
target_link_libraries(GTest_SplitTask  CloCK ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

add_executable(GTest_Backtrack GTest_Backtrack.cpp)
target_link_libraries(GTest_Backtrack gtest gtest_main)
target_link_libraries(GTest_Backtrack CloCK ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)



gtest_discover_tests(GTest_HighLevel)
gtest_discover_tests(GTest_AdjustGoal)
gtest_discover_tests(GTest_SplitTask)
gtest_discover_tests(GTest_Backtrack)


# INCLUDE_DIRECTORIES("/usr/local/include/")
# enable_testing()

# set(CTEST_OUTPUT_ON_FAILURE 1)

# #add_executable(explorer explorer.cpp ../test_essentials.h)
# add_executable(transitionMatrix ../test_essentials.h transitionMatrix.cpp)
# add_executable(disturbance_dimensions ../test_essentials.h disturbance_dimensions.cpp)
# add_executable(processData_k ../../src/worldbuilder.h ../../src/worldbuilder.cpp processData_k.cpp)
# add_executable(kmeans_clusters ../../src/worldbuilder.h ../../src/worldbuilder.cpp kmeans_clusters.cpp)
# #visual inspection 
# add_executable(partition_clusters ../test_essentials.h partition_clusters.cpp)

add_executable(plan ../test_essentials.h plan.cpp)
# add_executable(split_step ../test_essentials.h split_step.cpp)
# add_executable(affine_transformation ../test_essentials.h affine_transform.cpp)
# add_executable(match ../test_essentials.h match.cpp)
# add_executable(find_match_state ../test_essentials.h find_match_s.cpp)
# add_executable(state_difference ../test_essentials.h state_difference.cpp)
# add_executable(match_equal ../test_essentials.h match_equal.cpp)
# add_executable(replanner ../callbacks.h replanner.cpp)
# add_executable(fill_for_dist ../test_essentials.h fill_for_dist.cpp)
# add_executable(make_sensor ../test_essentials.h make_sensor.cpp)
# add_executable(end_contact ../test_essentials.h end_contact.cpp)
# add_executable(get_disturbance ../test_essentials.h get_disturbance.cpp)
# add_executable(checkPlan ../test_essentials.h checkPlan.cpp)
# add_executable(task_to_execute ../test_essentials.h task_to_execute.cpp)
# add_executable(closed_loop_ended ../test_essentials.h closed_loop_ended.cpp)
# add_executable(axb ../test_essentials.h axb.cpp)
# add_executable(D_affine ../test_essentials.h D_affine.cpp)
# add_executable(rwf ../test_essentials.h real_world_focus.cpp)
# add_executable(wb_get_transform_sim ../test_essentials.h wb_get_transform_sim.cpp)
# add_executable(kalman ../test_essentials.h kalman.cpp)
# add_executable(filter_box ../test_essentials.h filter_box.cpp)
# add_executable(estimate_vertex ../test_essentials.h estimate_vertex.cpp)
# add_executable(calc_transform ../test_essentials.h calc_transform.cpp)
# add_executable(PID_inst PID.cpp)

# target_link_libraries(PID_inst CloCK ${OpenCV_LIBS} box2d rt)
# target_link_libraries(calc_transform CloCK ${OpenCV_LIBS} box2d rt)
# target_link_libraries(estimate_vertex CloCK ${OpenCV_LIBS} box2d rt)
# target_link_libraries(filter_box CloCK ${OpenCV_LIBS} box2d)
# target_link_libraries(kalman CloCK ${OpenCV_LIBS} box2d)
# target_link_libraries(wb_get_transform_sim CloCK ${OpenCV_LIBS} box2d)
# target_link_libraries(rwf CloCK ${OpenCV_LIBS} box2d)
# target_link_libraries(D_affine CloCK ${OpenCV_LIBS} box2d)
# target_link_libraries(axb CloCK ${OpenCV_LIBS} box2d)
# target_link_libraries(closed_loop_ended CloCK ${OpenCV_LIBS} box2d rt)
# target_link_libraries(task_to_execute CloCK ${OpenCV_LIBS} box2d rt)
# #target_link_libraries(explorer CloCK ${OpenCV_LIBS} box2d rt  )
# target_link_libraries(transitionMatrix CloCK ${OpenCV_LIBS} box2d rt  )
# target_link_libraries(disturbance_dimensions CloCK ${OpenCV_LIBS} box2d rt  )
target_link_libraries(plan CloCK ${OpenCV_LIBS} box2d rt  )
# target_link_libraries(split_step CloCK ${OpenCV_LIBS} box2d rt  )
# target_link_libraries(affine_transformation CloCK ${OpenCV_LIBS} box2d rt  )
# target_link_libraries(processData_k CloCK ${OpenCV_LIBS} box2d rt)
# target_link_libraries(partition_clusters CloCK ${OpenCV_LIBS} box2d rt)
# target_link_libraries(kmeans_clusters CloCK ${OpenCV_LIBS} box2d rt)
# target_link_libraries(match CloCK ${OpenCV_LIBS} box2d )
# target_link_libraries(find_match_state CloCK ${OpenCV_LIBS} box2d )
# target_link_libraries(state_difference CloCK ${OpenCV_LIBS} box2d )
# target_link_libraries(match_equal CloCK ${OpenCV_LIBS} box2d )
# target_link_libraries(replanner CloCK ${OpenCV_LIBS} box2d )
# target_link_libraries(fill_for_dist CloCK ${OpenCV_LIBS} box2d)
# target_link_libraries(make_sensor CloCK ${OpenCV_LIBS} box2d)
# target_link_libraries(end_contact CloCK ${OpenCV_LIBS} box2d)
# target_link_libraries(get_disturbance CloCK ${OpenCV_LIBS} box2d)
# target_link_libraries(checkPlan CloCK ${OpenCV_LIBS} box2d )


# add_test(NAME split_step COMMAND split_step 0.68 0 )

# add_test(NAME affine_transformation_ahead COMMAND affine_transformation 1 0 0)

# add_test(NAME transition COMMAND transitionMatrix)
# add_test(NAME transition_n_L_ok COMMAND transitionMatrix 0 l k)
# add_test(NAME transition_n_un_ok COMMAND transitionMatrix 0 u)
# add_test(NAME transition_n_D_crash COMMAND transitionMatrix 0 d c)
# add_test(NAME transition_n_D_ok COMMAND transitionMatrix 0 d k)
# add_test(NAME transition_n_D_safe COMMAND transitionMatrix 0 d s)
# add_test(NAME transition_y_R_behind COMMAND transitionMatrix 1 r k -1 0 -3.14)
# add_test(NAME transition_y_D_crash COMMAND transitionMatrix y d c)
# add_test(NAME transition_y_D_ok COMMAND transitionMatrix y d k)
# add_test(NAME transition_y_D_safe COMMAND transitionMatrix y d s)
# add_test(NAME transition_y_stop_crash COMMAND transitionMatrix y s c)
# add_test(NAME transition_y_stop_ok COMMAND transitionMatrix y s k)
# add_test(NAME transition_y_stop_safe COMMAND transitionMatrix y s s)
# add_test(NAME processData_k COMMAND processData_k ../target_40cm/ 1 )

# add_test(NAME kmeans_clusters COMMAND kmeans_clusters)
# add_test(NAME partition_clusters COMMAND partition_clusters ../target_40cm/)


# add_test(NAME fill_avoid1 COMMAND fill_for_dist ../target_68cm/ 0.37 0 1.5714 1)
# add_test(NAME fill_avoid2 COMMAND fill_for_dist ../target_68cm/ 0.37 0.27 0 2)
# add_test(NAME fill_avoid_close COMMAND fill_for_dist ../target_68cm/ 0.54 0 1.5714 3)
# add_test(NAME make_sensor_start COMMAND make_sensor 0 0 0 6)
# add_test(NAME make_sensor_turned COMMAND make_sensor 0 0 1.5714 7)
# add_test(NAME make_sensor_close COMMAND make_sensor 0.54 0 1.5714 4)
# add_test(NAME make_sensor_far COMMAND make_sensor 0.20 0 1.5714 5)

# add_test(NAME end_contact_obstacle_avoid COMMAND end_contact 0 0)
# add_test(NAME end_contact_obstacle_goal COMMAND end_contact 0 1)
# add_test(NAME end_contact_goal COMMAND end_contact 1 1)

# add_test(NAME get_disturbance_0dl COMMAND get_disturbance 0 2 0) #position 1, prev: DEF, curr: LEFT
# add_test(NAME get_disturbance_1l COMMAND get_disturbance 1 0) #position 1, prev: LEFT, curr: DEF
# add_test(NAME get_disturbance_2r COMMAND get_disturbance 2 1) #position 2, prev: RIGHT, curr:DEF
# add_test(NAME get_disturbance_3 COMMAND get_disturbance 3) #obstacle behind (pos 3), prev: DEF, curr: DEF
# add_test(NAME get_disturbance_3dr COMMAND get_disturbance 3 2 1) #pos 3, prev: DEF, curr: RIGHT
# add_test(NAME get_disturbance_3l COMMAND get_disturbance 3 0)
# add_test(NAME get_disturbance_3r COMMAND get_disturbance 3 1)
# add_test(NAME get_disturbance_4r COMMAND get_disturbance 4)
# add_test(NAME get_disturbance_4dr COMMAND get_disturbance 4 2 1)

# add_test(NAME disturbance_dimensions_1d COMMAND disturbance_dimensions)
# #add_test(NAME disturbance_dimensions_2d COMMAND disturbance_dimensions 2)


# # add_test(NAME explorer_no COMMAND explorer)
# # add_test(NAME explorer_cds COMMAND explorer ../cul_de_sac/ 0)
# # add_test(NAME explorer_target COMMAND explorer ../target_68cm/ 1)


# # add_test(NAME plan_no COMMAND plan)
# # add_test(NAME plan_cds COMMAND plan ../cul_de_sac/ 0)
# # add_test(NAME plan_target_40cm COMMAND plan ../target_40cm/ 1)
# # add_test(NAME plan_target_68cm COMMAND plan ../target_68cm/ 1)


# add_test(NAME match_false COMMAND match)
# add_test(NAME match_true_exact COMMAND match 1)
# add_test(NAME match_Dn_no COMMAND match 2 )
# add_test(NAME match_Dn_ratio COMMAND match 2 1)
# add_test(NAME match_Dn_position COMMAND match 2 2)
# set_property(TEST match_Dn_no PROPERTY WILL_FAIL true)
# set_property(TEST match_Dn_position PROPERTY WILL_FAIL true)
# add_test(NAME match_Dn_shape COMMAND match 2 3)
# set_property(TEST match_Dn_shape PROPERTY WILL_FAIL true)

# add_test(NAME match_Di_no COMMAND match 5 )
# add_test(NAME match_Di_position COMMAND match 5 2)
# set_property(TEST match_Di_no PROPERTY WILL_FAIL true)
# set_property(TEST match_Di_position PROPERTY WILL_FAIL true)
# add_test(NAME match_Di_shape COMMAND match 5 3)
# set_property(TEST match_Di_shape PROPERTY WILL_FAIL true)


# add_test(NAME find_match_state COMMAND find_match_state ../target_68cm/ 2 1)
# add_test(NAME find_match_state_f COMMAND find_match_state ../target_68cm/ 3 1)
# add_test(NAME find_match_3 COMMAND find_match_state ../target_68cm/ 3 1 29)
# add_test(NAME find_match_midtask11 COMMAND find_match_state ../target_68cm/ 3 1 14)
# add_test(NAME find_match_midtask2 COMMAND find_match_state ../target_68cm/ 2 1 14)
# add_test(NAME find_match_quarter COMMAND find_match_state ../target_68cm/ 2 1 7)

# set_property(TEST find_match_midtask11 PROPERTY WILL_FAIL true)
# set_property(TEST find_match_state_f PROPERTY WILL_FAIL true)

# add_test(NAME state_difference COMMAND state_difference ../target_68cm/ 1)

# add_test(NAME match_equal COMMAND match_equal)
# #add_test(NAME replanner_40 COMMAND replanner ../target_40cm/ 2 1)
# add_test(NAME replanner_68 COMMAND replanner ../target_68cm/ 2 1)
# add_test(NAME replanner_empty COMMAND replanner empty 2 1)
# add_test(NAME replanner_back COMMAND replanner empty 2 1 100 back)



# # add_test(NAME replanner_68_9 COMMAND replanner ../target_68cm/ 9 1 34)
# # add_test(NAME replanner_68_11 COMMAND replanner ../target_68cm/ 11 1 34)
# # add_test(NAME replanner_68_10 COMMAND replanner ../target_68cm/ 9 1 50)
# # add_test(NAME replanner_68_12 COMMAND replanner ../target_68cm/ 11 1 50)


# #ARGUMENTS: vertex match, goal?, how many iterations have passed?
# add_test(NAME checkPlan COMMAND checkPlan ../target_68cm/ 2 1 1)
# add_test(NAME checkPlan_4 COMMAND checkPlan ../target_68cm/ 2 1 4)
# add_test(NAME checkPlan_21 COMMAND checkPlan ../target_68cm/ 2 1 21)
# #add_test(NAME checkPlan_40cm COMMAND checkPlan ../target_40cm/ 2 1 1)
# #add_test(NAME checkPlan_40cm_4 COMMAND checkPlan ../target_40cm/ 2 1 4)
# add_test(NAME checkPlan_40cm_21 COMMAND checkPlan ../target_40cm/ 2 1 21)

# #ARGUMENTS: Dn affordance, new task direction, Di/goal affordance
# add_test(NAME task_to_execute_default_to_ob COMMAND task_to_execute 1 2 2)
# add_test(NAME task_to_execute_left_from_ob COMMAND task_to_execute 1 0 2)
# add_test(NAME task_to_execute_default_to_goal COMMAND task_to_execute 2 2 2)
# add_test(NAME task_to_execute_default COMMAND task_to_execute 0 2 0)


# #args:
# #1: disturbance transform x
# #2: disturbance transform y
# #3: disturbance transform angle (Deg)
# #4: task direction (int) (0: LEFT, 1: RIGHT, 2: DEFALLT)
# #5: disturbance affordance (int)
#                                                         #    x  y   w    dir aff 
# add_test(NAME cle_pursue_ob_fail COMMAND closed_loop_ended 0  0   0    2    2)
# set_property(TEST cle_pursue_ob_fail PROPERTY WILL_FAIL true)
# add_test(NAME cle_pursue_ob COMMAND closed_loop_ended      0.41 0   0    2    2)
# add_test(NAME cle_pursue_ob_edge COMMAND closed_loop_ended  0.4 0   0    2    2)
# add_test(NAME cle_avoid_L COMMAND closed_loop_ended         0 0   90    0    1)
# add_test(NAME cle_avoid_L_fail COMMAND closed_loop_ended    0 0   25    0    1)#fails
# set_property(TEST cle_avoid_L_fail PROPERTY WILL_FAIL true)
# add_test(NAME cle_avoid_R COMMAND closed_loop_ended         0 0   -90    1    1)#fails
# add_test(NAME cle_avoid_R_fail COMMAND closed_loop_ended    0 0   -25    1    1)#fails
# set_property(TEST cle_avoid_R_fail PROPERTY WILL_FAIL true)
# add_test(NAME cle_pursue_L COMMAND closed_loop_ended         0 0   0    0    2)
# add_test(NAME cle_pursue_L_fail COMMAND closed_loop_ended    0 0   25    0    2)#fails
# set_property(TEST cle_pursue_L_fail PROPERTY WILL_FAIL true)
# add_test(NAME cle_pursue_R COMMAND closed_loop_ended         0 0   -0    1    2)
# add_test(NAME cle_pursue_R_fail COMMAND closed_loop_ended    0 0   -25    1    2)#fails
# set_property(TEST cle_pursue_R_fail PROPERTY WILL_FAIL true)
# add_test(NAME cle_avoid_D COMMAND closed_loop_ended         .2 -.2   90    0    1)#fails
# add_test(NAME cle_avoid_D_fail COMMAND closed_loop_ended    .2 0   25    0    1)#fails
# set_property(TEST cle_avoid_D_fail PROPERTY WILL_FAIL true)

# # add_test(NAME axb_id COMMAND axb 0 0 0 0 0 0)
# # add_test(NAME axb_id_rot COMMAND axb 0 0 0 0 0 90)
# # add_test(NAME axb_id_rot53 COMMAND axb 0 0 0 0 0 53)
# # add_test(NAME axb_id_rot3753 COMMAND axb 0 0 37 0 0 53)
# # add_test(NAME axb_id_shift_rot COMMAND axb 0 0 0 .2 .3 90)
# # add_test(NAME axb_id_rot_custom COMMAND axb 1 1 0 0 0 90)
# # add_test(NAME axb_id_shift_custom COMMAND axb 1 1 0 .2 .3 0)
# # add_test(NAME axb_id_shift_rot_custom COMMAND axb 1 1 0 .2 .3 90)
# # add_test(NAME axb_id_shift_rot_custom2 COMMAND axb 1 1 53 .2 .3 37)
# # add_test(NAME axb_id_shift_rot_custom3 COMMAND axb .45 .21 53 .2 .3 37)
# # set_property(TEST axb_id_shift_rot_custom2 PROPERTY WILL_FAIL true)
# # set_property(TEST axb_id_shift_rot_custom3 PROPERTY WILL_FAIL true)

# add_test(NAME D_affine COMMAND D_affine 0 0 0 0 0 0)
# add_test(NAME D_affine_rot COMMAND D_affine 0 0 0 0 0 90)
# add_test(NAME D_affine_rot53 COMMAND D_affine 0 0 0 0 0 53)
# add_test(NAME D_affine_rot3753 COMMAND D_affine 0 0 37 0 0 53)
# add_test(NAME D_affine_shift_rot COMMAND D_affine 0 0 0 .2 .3 90)
# add_test(NAME D_affine_rot_custom COMMAND D_affine 1 1 0 0 0 90)
# add_test(NAME D_affine_shift_custom COMMAND D_affine 1 1 0 .2 .3 0)
# add_test(NAME D_affine_shift_rot_custom COMMAND D_affine 1 1 0 .02 .03 9)
# add_test(NAME D_affine_shift_rot_custom2 COMMAND D_affine 1 1 53 .05 .02 3)
# add_test(NAME D_affine_shift_rot_custom3 COMMAND D_affine .45 .21 53 -.02 .03 7)
# add_test(NAME D_affine_shift_rot_custom4 COMMAND D_affine 1 1 53 .05 .02 -4)
# add_test(NAME D_affine_shift_rot_custom5 COMMAND D_affine .45 .21 53 .02 -.03 7)
# #testing error when size different
# # add_test(NAME D_affine_ds COMMAND D_affine 0 0 0 0 0 0 0.01 0.06)
# # add_test(NAME D_affine_rot_ds COMMAND D_affine 0 0 0 0 0 90 0.01 0.06)
# # add_test(NAME D_affine_rot53_ds COMMAND D_affine 0 0 0 0 0 53 0.01 0.06)
# # add_test(NAME D_affine_rot3753_ds COMMAND D_affine 0 0 37 0 0 53 0.01 0.06)
# # add_test(NAME D_affine_shift_rot_ds COMMAND D_affine 0 0 0 .2 .3 90 0.01 0.06)
# # add_test(NAME D_affine_rot_custom_ds COMMAND D_affine 1 1 0 0 0 90 0.01 0.06)
# # add_test(NAME D_affine_shift_custom_ds COMMAND D_affine 1 1 0 .2 .3 0 0.01 0.06)
# # add_test(NAME D_affine_shift_rot_custom_ds COMMAND D_affine 1 1 0 .02 .03 9 0.01 0.06)
# # add_test(NAME D_affine_shift_rot_custom2_ds COMMAND D_affine 1 1 53 .05 .02 3 0.01 0.06)
# # add_test(NAME D_affine_shift_rot_custom3_ds COMMAND D_affine .45 .21 53 -.02 .03 7 0.01 0.06)
# # add_test(NAME D_affine_shift_rot_custom4_ds COMMAND D_affine 1 1 53 .05 .02 -4 0.01 0.06)
# # add_test(NAME D_affine_shift_rot_custom5_ds COMMAND D_affine .45 .21 53 .02 -.03 7 0.01 0.06)

# add_test(NAME rwf_ob_D COMMAND rwf .45 .21 53 1 2)
# add_test(NAME rwf_ob_l COMMAND rwf .45 .21 53 1 0)
# add_test(NAME rwf_ob_r COMMAND rwf .45 .21 53 1 1)
# add_test(NAME rwf_tar_D COMMAND rwf .45 .21 53 2 2)
# add_test(NAME rwf_tar_l COMMAND rwf .45 .21 53 2 0)
# add_test(NAME rwf_tar_r COMMAND rwf .45 .21 53 2 1)
# add_test(NAME rwf_none_D COMMAND rwf .45 .21 53 0 2)
# add_test(NAME rwf_none_l COMMAND rwf .45 .21 53 0 0)
# add_test(NAME rwf_none_r COMMAND rwf .45 .21 53 0 1)

# add_test(NAME wb_gt COMMAND wb_get_transform_sim ../target_68cm/)

# #add_test(NAME filter_box_DEF1 COMMAND filter_box 0 0 0 2) #straight ahead, default direction
# add_test(NAME filter_box_LEFT1 COMMAND filter_box 0 0 0 0) #straight ahead, left direction
# add_test(NAME filter_box_RIGHT1 COMMAND filter_box 0 0 0 1) #straight ahead, right direction
# #add_test(NAME filter_box_DEF2 COMMAND filter_box 0 0 .75 2) #45 deg turn, default direction
# # add_test(NAME filter_box_DEF3 COMMAND filter_box 0 0 -1.26 2) #neg, not divisor of pi, default direction
# # add_test(NAME filter_box_DEF4 COMMAND filter_box 0 0 3.14 2) #pi turn, default direction
# # add_test(NAME filter_box_DEF5 COMMAND filter_box .35 0 2.57 2) #straight ahead, default direction
# # add_test(NAME filter_box_DEF6 COMMAND filter_box .35 .45 2.57 2) #straight ahead, default direction
# # add_test(NAME filter_box_DEF7 COMMAND filter_box .35 .45 -2.57 2) #straight ahead, default direction
# # add_test(NAME filter_box_DEF8 COMMAND filter_box -.35 .45 2.57 2) #straight ahead, default direction
# # add_test(NAME filter_box_DEF9 COMMAND filter_box .35 -.45 -2.57 2) #straight ahead, default direction
# # add_test(NAME filter_box_DEF10 COMMAND filter_box .35 -.45 2.57 2) #straight ahead, default direction
# # add_test(NAME filter_box_DEF11 COMMAND filter_box -.35 .45 -2.57 2) #straight ahead, default direction


# add_test(NAME estimate_vertex_origin COMMAND estimate_vertex 0 0 0)
# add_test(NAME estimate_vertex_split1 COMMAND estimate_vertex 1 0 0) #exactly at end of first split
# add_test(NAME estimate_vertex_split1_52 COMMAND estimate_vertex 1.52 0 0) #halfway between 1 and 2
# add_test(NAME estimate_vertex_split_pi2 COMMAND estimate_vertex 1 0 1.57) #exactly at end of first split
# add_test(NAME estimate_vertex_split_pi8 COMMAND estimate_vertex 1 0 .39) #exactly at end of first split
# add_test(NAME estimate_vertex_split_small_yshift COMMAND estimate_vertex 1 .05 0) #halfway between 1 and 2
# add_test(NAME estimate_vertex_split_large_yshift COMMAND estimate_vertex 1 1 0) #halfway between 1 and 2
# add_test(NAME estimate_vertex_split_large_out COMMAND estimate_vertex 2.5 1 0) #halfway between 1 and 2
# set_property(TEST estimate_vertex_split_large_yshift PROPERTY WILL_FAIL true)
# add_test(NAME estimate_vertex_empty COMMAND estimate_vertex 0 0 0 0)

# add_test(NAME calc_transform_fw COMMAND calc_transform -0.3 0 ) #robot moves fw,
# add_test(NAME calc_transform_back COMMAND calc_transform 0.3 0 ) #robot moves back
# add_test(NAME calc_transform_l COMMAND calc_transform 0 -0.3 ) #robot moves l
# add_test(NAME calc_transform_r COMMAND calc_transform 0 0.3 ) #robot moves r

# add_test(NAME PID_inst COMMAND PID_inst) #robot shifts and rotates


set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)

