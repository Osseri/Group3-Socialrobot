:- module(predicate__service_globalVariable,
    []).
    
:- nb_linkval(predicate__batteryState_low_robot_battery_threshold, 30).			%batteryState
:- nb_linkval(predicate__batteryState_high_robot_battery_threshold, 70).			%batteryState

:- nb_linkval(predicate__tilted_Object_low_angle_threshold, 10).					%tilted_Object
:- nb_linkval(predicate__tilted_Object_high_angle_threshold, 350).				%tilted_Object

:- nb_linkval(predicate__standing_Object_low_angle_threshold, 10).				%standing_Object
:- nb_linkval(predicate__standing_Object_high_angle_threshold, 350).				%standing_Object


:- nb_linkval(predicate__empty_hand_intersectPer_threshold, 0.003).	%0.05			%empty_hand
:- nb_linkval(predicate__overlap_hand_intersectPer_threshold, 0.003).		%0.05		%overlap_hand

:- nb_linkval(predicate__joint_opened_hand_finger_number, 2).					%opened_hand
:- nb_linkval(predicate__joint_closed_hand_finger_number, 2).					%closed_hand
:- nb_linkval(predicate__joint_check_joint_angle_threshold, -19.65).	%-15			%joint_check %10 for gazebo 15 for vrep

:- nb_linkval(predicate__detected_object_min_time_threshold, 30).				%detected_object
:- nb_linkval(predicate__detected_object_max_time_threshold, 60).				%detected_object
