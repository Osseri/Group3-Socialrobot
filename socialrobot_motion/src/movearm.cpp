#include "movearm.h"

using namespace socialrobot_motion;

MoveArm::MoveArm(void):
    nh("~"),
    leftarm_group("left_arm"),
    rightarm_group("right_arm")
{
    leftarm_group.setPlanningTime(5.0);
    rightarm_group.setPlanningTime(5.0);
    //ROS_INFO("Reference frame: %s", leftarm_group.getPlanningFrame().c_str());
    //ROS_INFO("Reference frame: %s", leftarm_group.getEndEffectorLink().c_str());
    display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
}

MoveArm::~MoveArm(void)
{
    ;
}


void MoveArm::update_scene(socialrobot_motion::MotionPlan::Request& req)
{
    ROS_INFO("(MoveArm) update_scene...");

    //update current state(ROBOT)
    if(req.targetBody == req.LEFT_ARM
        || req.targetBody == req.RIGHT_ARM)
    {
        plan_status = socialrobot_motion::MotionPlan::Response::SUCCESS;
    }
    else
    {
        plan_status = socialrobot_motion::MotionPlan::Response::ERROR_INPUT;
    }

    // adding/removing objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;  

    for(size_t i=0; i<req.obstacle_ids.size(); i++)
    {
        std::string id = req.obstacle_ids[i];

        // collision object
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = leftarm_group.getPlanningFrame();
        collision_object.id = id;
        if (id.compare(left_attached_object) == 0 || id.compare(right_attached_object)==0)
            continue;

        // check whether the id is already resistered.
        if(collision_ids.find(id) != collision_ids.end())
        {
            // move a collision object(update pose)
            collision_object.operation = collision_object.MOVE;
        }
        else
        {
            // add a collision object
            collision_ids.insert(id);

            // create a primitive shape
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = req.obstacles[i].size.x;
            primitive.dimensions[1] = req.obstacles[i].size.y;
            primitive.dimensions[2] = req.obstacles[i].size.z;

            collision_object.primitives.push_back(primitive);
            collision_object.operation = collision_object.ADD;
        }

        // setup pose
        geometry_msgs::Pose object_pose;
        object_pose.orientation = req.obstacles[i].center.orientation;
        object_pose.position = req.obstacles[i].center.position;

        collision_object.primitive_poses.push_back(object_pose);
        collision_objects.push_back(collision_object);
    }
    planning_scene_interface.applyCollisionObjects(collision_objects);  

    // attach or detach the target object
    std::string target_object = req.targetObject;
     
    if(req.targetBody == req.LEFT_ARM)
    {
        // detach
        if((target_object.size()<1) && (left_attached_object.size()>4))
        {
            leftarm_group.detachObject(target_object);
            left_attached_object.clear();
        }
        // attach
        else if((target_object.size()>4) && (left_attached_object.size()<1))
        {
            if(collision_ids.find(target_object) != collision_ids.end())
            {
                leftarm_group.attachObject(target_object);
                left_attached_object = target_object;
            }
        }
    }
    else if(req.targetBody == req.RIGHT_ARM)
    {
        // detach
        if((target_object.size()<1) && (right_attached_object.size()>4))
        {
            rightarm_group.detachObject(target_object);
            right_attached_object.clear();
        }
        // attach
        else if((target_object.size()>4) && (right_attached_object.size()<1))
        {
            if(collision_ids.find(target_object) != collision_ids.end())
            {
                rightarm_group.attachObject(target_object);
                right_attached_object = target_object;
            }
        }
    }


    //update planning_scene
    leftarm_group.setStartState(*leftarm_group.getCurrentState());
    rightarm_group.setStartState(*rightarm_group.getCurrentState());
}

void MoveArm::compute_path(socialrobot_motion::MotionPlan::Request& req)
{
    ROS_INFO("(MoveArm) compute_path...");
    if(plan_status != 0)
        return;

    moveit::planning_interface::MoveItErrorCode success;
    leftarm_group.clearPathConstraints();
    rightarm_group.clearPathConstraints();
       
    // set a goal pose
    if(req.goalType == req.CARTESIAN_SPACE_GOAL || req.goalType == req.CARTESIAN_WITH_ORIENTATION_CONSTRAINTS)
    {
        // constraints
        // 1. orientation
        // 2. waypoints
        moveit_msgs::Constraints constraints;

        // setup goal
        //const std::string base_link_name = leftarm_group.getPlanningFrame();

        geometry_msgs::Pose target_goal;
        target_goal.position = req.targetPose.position;
        target_goal.orientation = req.targetPose.orientation;

        if(req.targetBody == req.LEFT_ARM)
        {
            //constraints
            if(req.goalType == req.CARTESIAN_WITH_ORIENTATION_CONSTRAINTS)
            {
                moveit_msgs::OrientationConstraint ocm;
                ocm.link_name = leftarm_group.getEndEffectorLink();
                auto current_pose = leftarm_group.getCurrentPose(ocm.link_name);

                tf::Quaternion q_orig, q_rot, q_new;
                tf::quaternionMsgToTF(current_pose.pose.orientation, q_orig);
                tf::quaternionMsgToTF(target_goal.orientation, q_rot);
                q_new = q_rot * q_orig;
                q_new.normalize();

                ocm.header.frame_id = current_pose.header.frame_id;
                ocm.orientation.x = q_new[0];
                ocm.orientation.y = q_new[1];
                ocm.orientation.z = q_new[2];
                ocm.orientation.w = q_new[3];
                ocm.absolute_x_axis_tolerance = 0.1;
                ocm.absolute_y_axis_tolerance = 0.1;
                ocm.absolute_z_axis_tolerance = 0.1;
                target_goal.orientation = ocm.orientation;
                //ROS_INFO("o: %3.3f, %3.3f, %3.3f, %3.3f", q_new[0], q_new[1], q_new[2], q_new[3]);
                
                //constraints.orientation_constraints.push_back(ocm);
                //leftarm_group.setPathConstraints(constraints);
            }
            else if(req.goalType == req.CARTESIAN_WITH_POSITION_CONSTRAINTS)
            {
                auto link_name = leftarm_group.getEndEffectorLink();
                auto current_pose = leftarm_group.getCurrentPose(link_name);
                tf::Quaternion q_orig, q_rot, q_new;
                tf::quaternionMsgToTF(current_pose.pose.orientation, q_orig);
                tf::quaternionMsgToTF(target_goal.orientation, q_rot);

                q_new = q_rot * q_orig;
                q_new.normalize();

                target_goal.position = current_pose.pose.position;
                target_goal.orientation.x = q_new[0];
                target_goal.orientation.y = q_new[1];
                target_goal.orientation.z = q_new[2];
                target_goal.orientation.w = q_new[3];
            }     

            leftarm_group.setPoseTarget(target_goal);
            success = leftarm_group.plan(my_plan);

            // if you work with a real robot, uncomment below line
            if(success)
                ;//leftarm_group.move();   
        }
        else if (req.targetBody == req.RIGHT_ARM)
        {
            //constraints
            if(req.goalType == req.CARTESIAN_WITH_ORIENTATION_CONSTRAINTS)
            {
                moveit_msgs::OrientationConstraint ocm;
                ocm.link_name = rightarm_group.getEndEffectorLink();
                auto current_pose = rightarm_group.getCurrentPose(ocm.link_name);

                tf::Quaternion q_orig, q_rot, q_new;
                tf::quaternionMsgToTF(current_pose.pose.orientation, q_orig);
                tf::quaternionMsgToTF(target_goal.orientation, q_rot);
                q_new = q_rot * q_orig;
                q_new.normalize();

                ocm.header.frame_id = current_pose.header.frame_id;
                ocm.orientation.x = q_new[0];
                ocm.orientation.y = q_new[1];
                ocm.orientation.z = q_new[2];
                ocm.orientation.w = q_new[3];
                ocm.absolute_x_axis_tolerance = 0.1;
                ocm.absolute_y_axis_tolerance = 0.1;
                ocm.absolute_z_axis_tolerance = 0.1;
                target_goal.orientation = ocm.orientation;

                //constraints.orientation_constraints.push_back(ocm);
                //rightarm_group.setPathConstraints(constraints);
            }
            else if(req.goalType == req.CARTESIAN_WITH_POSITION_CONSTRAINTS)
            {
                auto link_name = rightarm_group.getEndEffectorLink();
                auto current_pose = rightarm_group.getCurrentPose(link_name);
                tf::Quaternion q_orig, q_rot, q_new;
                tf::quaternionMsgToTF(current_pose.pose.orientation, q_orig);
                tf::quaternionMsgToTF(target_goal.orientation, q_rot);

                q_new = q_rot * q_orig;
                q_new.normalize();

                target_goal.position = current_pose.pose.position;
                target_goal.orientation.x = q_new[0];
                target_goal.orientation.y = q_new[1];
                target_goal.orientation.z = q_new[2];
                target_goal.orientation.w = q_new[3];
            }

            rightarm_group.setPoseTarget(target_goal);
            success = rightarm_group.plan(my_plan);  

            // if you work with a real robot, uncomment below line
            if(success)
                ;//rightarm_group.move();
        }
        
    }
    else if(req.goalType == req.JOINT_SPACE_GOAL)
    {
        // setup goal


        if(req.targetBody == req.LEFT_ARM)
        {
            leftarm_group.setJointValueTarget(req.targetJointState);
            success = leftarm_group.plan(my_plan);

            // if you work with a real robot, uncomment below line
            if(success)
                ;//leftarm_group.move();  
        }
        else if(req.targetBody == req.RIGHT_ARM)
        {
            rightarm_group.setJointValueTarget(req.targetJointState);
            success = rightarm_group.plan(my_plan);  

            // if you work with a real robot, uncomment below line
            if(success)
                ;//rightarm_group.move();
        }
    }


    // restore the result
    joint_trajectory.joint_names.clear();
    joint_trajectory.points.clear();
    joint_trajectory = my_plan.trajectory_.joint_trajectory;

    if(success)
        plan_status = socialrobot_motion::MotionPlan::Response::SUCCESS;
    else
        plan_status = socialrobot_motion::MotionPlan::Response::ERROR_NO_SOLUTION;
}

void MoveArm::visualize_trajectory()
{
    if(plan_status == socialrobot_motion::MotionPlan::Response::SUCCESS)
    {
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
    }
}