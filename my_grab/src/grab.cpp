
#include <actionlib/client/simple_action_client.h>
#include <dh_hand_driver/ActuateHandAction.h>
#include <dh_hand_driver/hand_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<dh_hand_driver::ActuateHandAction> Client;
// dh_hand_driver
class DH_HandActionClient {
   private:
    // Called once when the goal completes
    void DoneCb(const actionlib::SimpleClientGoalState& state,
                const dh_hand_driver::ActuateHandResultConstPtr& result) {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("result  : %i", result->opration_done);
    }

    // when target active, call this once
    void ActiveCb() {
        ROS_INFO("Goal just went active");
    }

    // received feedback
    void FeedbackCb(
        const dh_hand_driver::ActuateHandFeedbackConstPtr& feedback) {
        ROS_INFO("Got Feedback: %i", feedback->position_reached);
    }

   public:
    DH_HandActionClient(const std::string client_name, bool flag = true) : client(client_name, flag) {
    }

    //client start
    void Start(int32_t motorID, int32_t setpos, int32_t setforce) {
        ROS_INFO("wait server");
        client.waitForServer();
        //set goal
        dh_hand_driver::ActuateHandGoal goal;
        // AG2E just has one motor (ID:1)
        // AG3E has two motor (ID:1 and 2)
        goal.MotorID = motorID;
        goal.force = setforce;
        goal.position = setpos;

        ROS_INFO("Send goal %d %d %d", goal.MotorID, goal.force, goal.position);
        //sen goal
        client.sendGoal(goal,
                        boost::bind(&DH_HandActionClient::DoneCb, this, _1, _2),
                        boost::bind(&DH_HandActionClient::ActiveCb, this),
                        boost::bind(&DH_HandActionClient::FeedbackCb, this, _1));
        ROS_INFO("wait result");

        client.waitForResult(ros::Duration(15.0));

        //process the result
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Send commond succeeded");
        else {
            ROS_WARN("Cancel Goal!");
            client.cancelAllGoals();
        }

        printf("Current State: %s\n", client.getState().toString().c_str());
    }

   private:
    Client client;
};

void GetPoseMsg() {
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grab_demo");
    ros::NodeHandle node_handle;

    // Start a thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Define the planning group name
    static const std::string PLANNING_GROUP = "manipulator_i5";

    // Create a planning group interface object and set up a planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPoseReferenceFrame("base_link");

    // Create a planning scene interface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create a robot model information object
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Create an object of the visualization class
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    // Load remote control tool
    visual_tools.loadRemoteControl();

    // Create text
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.2;
    visual_tools.publishText(text_pose, "AUBO Demo", rvt::RED, rvt::XLARGE);
    // Text visualization takes effect
    visual_tools.trigger();

    // Get the coordinate system of the basic information
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // Get the end of the basic information
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Visual terminal prompt (blocking)
    visual_tools.prompt("Press 'next'1 in the RvizVisualToolsGui window to start the demo");

    //***************************************************************************************************  Home Position

    std::vector<double> home_position;
    home_position.push_back(-0.001255);
    home_position.push_back(-0.148822);
    home_position.push_back(-1.406503);
    home_position.push_back(0.311441);
    home_position.push_back(-1.571295);
    home_position.push_back(-0.002450);
    move_group.setJointValueTarget(home_position);
    move_group.move();

    //**************************************************************************************************   First example: planning and moving to a target pose
    // Set the target pose , RPY mode (rotation around the reference axis X, Y, Z)
    tf::Quaternion q;
    q.setRPY(3.14, 0, -1.57);  //radian

    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = -0.4;
    target_pose1.position.y = -0.3;
    target_pose1.position.z = 0.30;
    target_pose1.orientation.x = q.x();
    target_pose1.orientation.y = q.y();
    target_pose1.orientation.z = q.z();
    target_pose1.orientation.w = q.w();

    move_group.setPoseTarget(target_pose1);

    // Call the planner for planning calculations Note: This is just planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");

    // visual planning path in Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
    // Parameter 1 (trajectory_): path information
    // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // Perform planning actions
    move_group.execute(my_plan);

    // Move to the home point position
    move_group.setJointValueTarget(home_position);
    move_group.move();

    // **************************************************************************************************   第三实例：机器臂根据设置的路径约束从起始点运动到初始点
    // **************************************************************************************************   The third example: the robot arm moves from point A to point B according to the set path constraint.

    q.setRPY(3.14, 0, -1.57);

    // Define the path constraint
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "wrist3_Link";
    ocm.header.frame_id = "base_link";

    // Set the pose to be constrained at the end (consistent with base_link according to the settings)
    ocm.orientation.w = q.w();
    ocm.orientation.x = q.x();
    ocm.orientation.y = q.y();
    ocm.orientation.z = q.z();
    ocm.absolute_x_axis_tolerance = 0.2;  //(Specify the tolerance of the axis)
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2;
    ocm.weight = 1.0;

    // Add path constraints
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);

    // Set initial position
    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.position.x = -0.4;
    start_pose2.position.y = 0.05;
    start_pose2.position.z = 0.54;
    start_pose2.orientation.x = q.x();
    start_pose2.orientation.y = q.y();
    start_pose2.orientation.z = q.z();
    start_pose2.orientation.w = q.w();

    // 机械臂首先要运动到起始位置
    // the robot must first move to the starting position
    move_group.setPoseTarget(start_pose2);
    move_group.move();

    // Reset the joint_model_group of the starting position for visual track display
    start_state.setFromIK(joint_model_group, start_pose2);
    move_group.setStartState(start_state);

    // Set the target pose
    geometry_msgs::Pose target_pose3_1;
    target_pose3_1.position.x = -0.4;
    target_pose3_1.position.y = -0.19;
    target_pose3_1.position.z = 0.41;
    target_pose3_1.orientation.x = q.x();
    target_pose3_1.orientation.y = q.y();
    target_pose3_1.orientation.z = q.z();
    target_pose3_1.orientation.w = q.w();
    move_group.setPoseTarget(target_pose3_1);

    // The default time for the kinematics solver calculation plan is 5s. Increasing the time can increase the success rate.
    move_group.setPlanningTime(20.0);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "success" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose2, "start");
    visual_tools.publishAxisLabeled(target_pose3_1, "goal");
    visual_tools.publishText(text_pose, "AUBO Constrained Goal Example3", rvt::RED, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // Perform planning actions
    move_group.execute(my_plan);

    // Move to the home point position
    move_group.setPoseTarget(start_pose2);
    move_group.move();

    visual_tools.prompt("next step 4");

    // Clear path constraint
    move_group.clearPathConstraints();

    // **************************************************************************************************   Fourth example: planning and moving a Cartesian interpolation path

    //  Add three waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose2);

    geometry_msgs::Pose target_pose3 = start_pose2;

    target_pose3.position.z -= 0.2;
    waypoints.push_back(target_pose3);  // down

    target_pose3.position.y -= 0.15;
    waypoints.push_back(target_pose3);  // right

    target_pose3.position.z += 0.2;
    target_pose3.position.y += 0.2;
    target_pose3.position.x -= 0.2;
    waypoints.push_back(target_pose3);  // up and left

    // Reduce the speed of the robot arm by the scaling factor of the maximum speed of each joint. Please note that this is not the speed of the final effector.
    move_group.setMaxVelocityScalingFactor(0.5);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm.
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;  //(The jump threshold is set to 0.0)
    const double eef_step = 0.01;       //(interpolation step)

    // Calculate Cartesian interpolation path: return path score (0~1, -1 stands for error)
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "AUBO Joint Space Goal Example4", rvt::RED, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    }
    visual_tools.trigger();

    // Move to the home point position
    my_plan.trajectory_ = trajectory;
    move_group.execute(my_plan);

    // Clear path constraint
    move_group.setJointValueTarget(home_position);
    move_group.move();

    // END_TUTORIAL
    ros::shutdown();
    return 0;
}
