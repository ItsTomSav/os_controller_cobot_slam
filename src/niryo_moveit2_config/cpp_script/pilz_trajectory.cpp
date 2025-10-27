#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    // Inizializza ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pilz_trajectory");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Crea MoveGroupInterface per il Niryo Ned2
    static const std::string PLANNING_GROUP = "niryo_arm";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    move_group.setPlannerId("PTP"); // Imposta il planner Pilz -> PointToPoint (posso utilizzare anche "LIN" o "CIRC")

    // Setup visual tools per RViz
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "world", rvt::RVIZ_MARKER_TOPIC);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Mostra frame di riferimento
    visual_tools.trigger();

    // Variabili per memorizzare la pianificazione
    moveit::core::MoveItErrorCode success;
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Posa iniziale del robot
    auto start_pose = move_group.getCurrentPose().pose;

    // 1. Definisci un segmento lineare
    geometry_msgs::msg::Pose target_pose1 = start_pose;
    target_pose1.position.x += 0.2; // Muovi il robot lungo l'asse X di 20 cm
    move_group.setPoseTarget(target_pose1);

    RCLCPP_INFO(node->get_logger(), "Pianifico segmento lineare...");
    success = move_group.plan(plan);
    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Pianificazione riuscita! Eseguo il movimento...");
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Pianificazione fallita!");
        return 1;
    }

    // 2. Definisci un movimento circolare (raggio 15 cm attorno a Z)
    geometry_msgs::msg::Pose target_pose2 = target_pose1;
    std::vector<geometry_msgs::msg::Pose> waypoints;

    const double radius = 0.15; // Raggio del cerchio
    const double steps = 20;    // Numero di punti nella traiettoria circolare
    for (size_t i = 0; i <= steps; ++i)
    {
        double angle = (2 * M_PI * i) / steps; // Angolo corrente
        geometry_msgs::msg::Pose waypoint = target_pose1;
        waypoint.position.x += radius * cos(angle);
        waypoint.position.y += radius * sin(angle);
        waypoints.push_back(waypoint);
    }

    RCLCPP_INFO(node->get_logger(), "Pianifico segmento circolare...");
    double fraction = move_group.computeCartesianPath(
        waypoints,       // Punti della traiettoria
        0.01,            // Risoluzione
        0.0,             // Salto massimo consentito
        trajectory, true // Pianifica con Pilz
    );

    if (fraction > 0.95)
    {
        RCLCPP_INFO(node->get_logger(), "Pianificazione riuscita al %.2f%%! Eseguo il movimento...", fraction * 100);
        move_group.execute(trajectory);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Pianificazione fallita! Percentuale completata: %.2f%%", fraction * 100);
        return 1;
    }

    // Pulizia
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    RCLCPP_INFO(node->get_logger(), "Esecuzione completata.");
    rclcpp::shutdown();
    return 0;
}
