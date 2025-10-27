#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char * argv[])
{
// Start up ROS 2
rclcpp::init(argc, argv);

// Creates a node named "lin_traj_chomp". The node is set up to automatically
// handle any settings (parameters) we might want to change later without editing the code.
auto const node = std::make_shared<rclcpp::Node>(
    "lin_traj_chomp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);

// Creates a "logger" that we can use to print out information or error messages
// as our program runs.
auto const logger = rclcpp::get_logger("lin_traj_chomp");

// This code creates a separate thread, which is like a mini-program running alongside our main program.
// This thread uses a ROS 2 "executor" to continuously process information about the robot's state.
// By running this in its own thread, our ROS 2 node can keep getting updates about the robot without
// interrupting the main flow of our program.
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
auto spinner = std::thread([&executor]() { executor.spin(); });

// Create the MoveIt MoveGroup Interfaces
// These interfaces are used to plan and execute movements, set target poses,
// and perform other motion-related tasks for each respective part of the robot.
// The use of auto allows the compiler to automatically deduce the type of variable.
using moveit::planning_interface::MoveGroupInterface;
auto arm_group_interface = MoveGroupInterface(node, "niryo_arm");

// Ottieniamo la posa corrente dell'end-effector prima della pianificazione
auto current_pose = arm_group_interface.getCurrentPose();

RCLCPP_INFO(logger, "Posa iniziale dell'end-effector:");
RCLCPP_INFO(logger, "  Posizione: x=%.3f, y=%.3f, z=%.3f",
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z);
RCLCPP_INFO(logger, "  Orientamento: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);

// Specify a planning pipeline to be used for further planning
arm_group_interface.setPlanningPipelineId("chomp");

// Specify the maximum amount of time in seconds to use when planning
arm_group_interface.setPlanningTime(1.0);

// Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
arm_group_interface.setMaxVelocityScalingFactor(1.0);

//  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
arm_group_interface.setMaxAccelerationScalingFactor(1.0);

// Display helpful logging messages on the terminal
RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

// Construct and initialize MoveItVisualTools
auto moveit_visual_tools =
    moveit_visual_tools::MoveItVisualTools{ node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                            arm_group_interface.getRobotModel() };

// Erase all existing visual markers from the RViz visualization.
moveit_visual_tools.deleteAllMarkers();

// Load the remote control interface for MoveIt visual tools.
moveit_visual_tools.loadRemoteControl();

// Lambda function to draw a title in the RViz visualization
auto const draw_title = [&moveit_visual_tools](auto text) {
    // Nested lambda to create a pose for the text
    auto const text_pose = [] {
        // Create an identity transform
        auto msg = Eigen::Isometry3d::Identity();
        // Set the z-coordinate to 1.0 (1 meter above the origin)
        msg.translation().z() = 1.0;
        return msg;
    }();

    // Publish the text to RViz
    // Parameters: pose, text content, color (WHITE), and size (XLARGE)
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
};

// Lambda function to display a prompt message in RViz
auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
};

// Lambda function to visualize a trajectory as a line in RViz
auto const draw_trajectory_tool_path =
    [&moveit_visual_tools, jmg = arm_group_interface.getRobotModel()->getJointModelGroup("niryo_arm")](
    auto const trajectory) {moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);};

// Set a target pose for the end effector of the arm
auto const posa_presa = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.357;
    msg.pose.position.y = 0.001;
    msg.pose.position.z = 0.320;
    msg.pose.orientation.x = 0.612;
    msg.pose.orientation.y = 0.437;
    msg.pose.orientation.z = 0.537;
    msg.pose.orientation.w = 0.382;
    return msg;
}();

auto const posa_rilascio = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = node->now();
    msg.pose.position.x = -0.267;
    msg.pose.position.y = -0.215;
    msg.pose.position.z = 0.149;
    msg.pose.orientation.x = 0.753;
    msg.pose.orientation.y = -0.250;
    msg.pose.orientation.z = -0.327;
    msg.pose.orientation.w = 0.514;
    return msg;
}();

auto const posa_finale = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.295;
    msg.pose.position.y = - 0.0;
    msg.pose.position.z = 0.429;
    msg.pose.orientation.x = 0.707;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.707;
    msg.pose.orientation.w = - 0.0;
    return msg;
}();


//  Funzione
auto plan_and_execute = [&](const geometry_msgs::msg::PoseStamped& arm_target_pose) -> bool {
    // Trova il modello cinematico del robot
    const moveit::core::JointModelGroup* joint_model_group =
    arm_group_interface.getCurrentState()->getJointModelGroup("niryo_arm");
    // Crea uno stato robotico per calcolare l'IK
    moveit::core::RobotState target_state(*arm_group_interface.getCurrentState());
    // Calcola la cinematica inversa per trovare i valori articolari corrispondenti alla arm_target_pose
    bool found_ik = target_state.setFromIK(joint_model_group, arm_target_pose.pose, true);

    if (found_ik) {
    std::vector<double> joint_target;
    target_state.copyJointGroupPositions(joint_model_group, joint_target);
    // Stampa i valori articolari trovati dall'IK
    RCLCPP_INFO(node->get_logger(), "IK trovata! Target articolare:");
    for (size_t i = 0; i < joint_target.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "  Joint %zu: %f", i, joint_target[i]);
    }
    // Imposta il target articolare ottenuto dall'IK
    arm_group_interface.setJointValueTarget(joint_target);
    } else {
    RCLCPP_ERROR(node->get_logger(), "IK non trovata per la posa desiderata!");
    RCLCPP_INFO(node->get_logger(), "Posizione desiderata: x=%f, y=%f, z=%f",
                arm_target_pose.pose.position.x, arm_target_pose.pose.position.y, arm_target_pose.pose.position.z);
    return false;
    }

    // Create a plan to that target pose
    // This will give us two things:
    // 1. Whether the planning was successful (stored in 'success')
    // 2. The actual motion plan (stored in 'plan')
    auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
    }();

    // Try to execute the movement plan if it was created successfully
    // If the plan wasn't successful, report an error
    // Execute the plan
    if (success)
    {
    // Visualize the planned trajectory in RViz
    draw_trajectory_tool_path(plan.trajectory_);
    // Trigger the visualization update
    moveit_visual_tools.trigger();

    // Prompt the user to continue to execution
    // prompt("Press 'next' in the RvizVisualToolsGui window to execute");

    // Update the title in RViz to show we're executing the plan
    // draw_title("Executing");

    // Trigger another visualization update
    moveit_visual_tools.trigger();
    // Execute the planned motion
    arm_group_interface.execute(plan);

    // Aspetta un attimo per aggiornare lo stato del robot
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Ottieni la posa finale dell'end-effector dopo l'esecuzione
    auto final_pose = arm_group_interface.getCurrentPose();

    RCLCPP_INFO(logger, "Posa finale dell'end-effector:");
    RCLCPP_INFO(logger, "  Posizione: x=%.3f, y=%.3f, z=%.3f",
                final_pose.pose.position.x,
                final_pose.pose.position.y,
                final_pose.pose.position.z);
    RCLCPP_INFO(logger, "  Orientamento: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                final_pose.pose.orientation.x,
                final_pose.pose.orientation.y,
                final_pose.pose.orientation.z,
                final_pose.pose.orientation.w);
    }
    else
    {
    // If planning failed, update the title in RViz
    draw_title("Planning Failed!");

    // Trigger the visualization update
    moveit_visual_tools.trigger();

    // Log an error message
    RCLCPP_ERROR(logger, "Planning failed!");
    }
    // Exit
    return true;
};


// Creazione di oggetti di collisione
auto const collision_objects = [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger] {
    // Stampiamo il frame per il debug
    RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

    // Vettore di collision object
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // 1. Creazione del pavimento
    moveit_msgs::msg::CollisionObject floor;
    floor.header.frame_id = frame_id;
    floor.header.stamp = node->now();
    floor.id = "floor";

    shape_msgs::msg::SolidPrimitive floor_primitive;
    floor_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    floor_primitive.dimensions = {3.0, 3.0, 0.1};

    geometry_msgs::msg::Pose floor_pose;
    floor_pose.position.x = 0.0;
    floor_pose.position.y = 0.0;
    floor_pose.position.z = -(floor_primitive.dimensions[2] / 2.0);  // Faccio si che la superficie superiore del pavimento tocchi la base del robot
    floor_pose.orientation.w = 1.0;

    floor.primitives.push_back(floor_primitive);
    floor.primitive_poses.push_back(floor_pose);
    floor.operation = floor.ADD;

    collision_objects.push_back(floor);

    // 2. Creazione della box 1
    moveit_msgs::msg::CollisionObject box;
    box.header.frame_id = frame_id;
    box.header.stamp = node->now();
    box.id = "box1";

    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    box_primitive.dimensions = {0.122, 0.122, 0.244};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.38;
    box_pose.position.y = - 0.01;
    box_pose.position.z = 0.13;
    box_pose.orientation.w = 1.0;

    box.primitives.push_back(box_primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;

    collision_objects.push_back(box);


    // 3. Creazione della box 2
    moveit_msgs::msg::CollisionObject box2;
    box2.header.frame_id = frame_id;
    box2.header.stamp = node->now();
    box2.id = "box2";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.092, 0.092, 0.092};

    geometry_msgs::msg::Pose box2_pose;
    box2_pose.position.x = - 0.29;
    box2_pose.position.y = - 0.2;
    box2_pose.position.z = 0.04;
    box2_pose.orientation.x = 0.0;
    box2_pose.orientation.y = 0.0;
    box2_pose.orientation.z = 0.204;
    box2_pose.orientation.w = 0.979;

    box2.primitives.push_back(primitive);
    box2.primitive_poses.push_back(box2_pose);
    box2.operation = box2.ADD;

    collision_objects.push_back(box2);


    // 4. Creazione della sfera (Oggetto da prendere)
    moveit_msgs::msg::CollisionObject sphere;
    sphere.header.frame_id = frame_id;
    sphere.header.stamp = node->now();
    sphere.id = "sphere";

    shape_msgs::msg::SolidPrimitive primitive1;
    primitive1.type = primitive1.SPHERE;
    primitive1.dimensions = {0.0192};

    geometry_msgs::msg::Pose sphere_pose;
    sphere_pose.position.x = 0.41;
    sphere_pose.position.y = - 0.01;
    sphere_pose.position.z = 0.28;
    sphere_pose.orientation.w = 1.0;

    sphere.primitives.push_back(primitive1);
    sphere.primitive_poses.push_back(sphere_pose);
    sphere.operation = sphere.ADD;

    collision_objects.push_back(sphere);

    // Log per debug
    RCLCPP_INFO(logger, "Aggiunti %lu oggetti alla scena di collisione.", collision_objects.size());

    return collision_objects;
}();

// Set up a virtual representation of the robot's environment
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

// Add an object to this virtual environment that the robot needs to avoid colliding with
planning_scene_interface.applyCollisionObjects(collision_objects);

// Update the visualization to show the new title and wait for user input
moveit_visual_tools.trigger();


rclcpp::sleep_for(std::chrono::seconds(1)); // Tempo per aggiornare la scena

// Esegui il primo target
if (!plan_and_execute(posa_presa)) {
    RCLCPP_ERROR(logger, "Errore nel raggiungere la prima posa, interrompo il programma.");
    return 1;
}

// ATTACCARE L'OGGETTO AL TOOL_LINK
moveit_msgs::msg::AttachedCollisionObject attached_object;
attached_object.link_name = "tool_link";  // Collegare al tool_link del Niryo
attached_object.object.id = "sphere";
attached_object.object.operation = attached_object.object.ADD;
planning_scene_interface.applyAttachedCollisionObject(attached_object);
RCLCPP_INFO(logger, "Sfera attaccata al tool_link!");

// Esegui il secondo target
if (!plan_and_execute(posa_rilascio)) {
    RCLCPP_ERROR(logger, "Errore nel raggiungere la seconda posa, interrompo il programma.");
    return 1;
}

// RILASCIARE L'OGGETTO
moveit_msgs::msg::AttachedCollisionObject detach_object;
detach_object.object.id = "sphere";
detach_object.object.operation = detach_object.object.REMOVE;
planning_scene_interface.applyAttachedCollisionObject(detach_object);
rclcpp::sleep_for(std::chrono::seconds(1));

// Esegui il terzo target
if (!plan_and_execute(posa_finale)) {
    RCLCPP_ERROR(logger, "Errore nel raggiungere la terza posa, interrompo il programma.");
    return 1;
}

rclcpp::shutdown();

// Wait for the spinner thread to finish
spinner.join();

// Exit the program
return 0;
}
