#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

struct RecordedPoint {
  double time_from_start_sec;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

struct RecordedSegment {
  size_t step_index;
  std::string step_description;
  int pause_ms;
  std::vector<RecordedPoint> points;
};

void save_recorded_json(const std::string& filepath,
                        const std::vector<std::string>& joint_names,
                        const std::vector<RecordedSegment>& segments)
{
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open file for recording: " << filepath << std::endl;
    return;
  }

  ofs << "{\n";
  ofs << "  \"joint_names\": [";
  for (size_t i = 0; i < joint_names.size(); ++i) {
    ofs << "\"" << joint_names[i] << "\"" << (i + 1 < joint_names.size() ? ", " : "");
  }
  ofs << "],\n";

  ofs << "  \"segments\": [\n";
  for (size_t s = 0; s < segments.size(); ++s) {
    const auto& seg = segments[s];
    ofs << "    {\n";
    ofs << "      \"step_index\": " << seg.step_index << ",\n";
    ofs << "      \"step_description\": \"" << seg.step_description << "\",\n";
    ofs << "      \"pause_ms\": " << seg.pause_ms << ",\n";
    ofs << "      \"points\": [\n";

    for (size_t p = 0; p < seg.points.size(); ++p) {
      const auto& rpt = seg.points[p];
      ofs << "        {\n";
      ofs << "          \"time_from_start_sec\": " << rpt.time_from_start_sec << ",\n";
      ofs << "          \"positions\": [";
      for (size_t j = 0; j < rpt.positions.size(); ++j) {
        ofs << rpt.positions[j] << (j + 1 < rpt.positions.size() ? ", " : "");
      }
      ofs << "],\n";

      ofs << "          \"velocities\": [";
      for (size_t j = 0; j < rpt.velocities.size(); ++j) {
        ofs << rpt.velocities[j] << (j + 1 < rpt.velocities.size() ? ", " : "");
      }
      ofs << "],\n";

      ofs << "          \"accelerations\": [";
      for (size_t j = 0; j < rpt.accelerations.size(); ++j) {
        ofs << rpt.accelerations[j] << (j + 1 < rpt.accelerations.size() ? ", " : "");
      }
      ofs << "]\n";

      ofs << "        }" << (p + 1 < seg.points.size() ? "," : "") << "\n";
    }

    ofs << "      ]\n";
    ofs << "    }" << (s + 1 < segments.size() ? "," : "") << "\n";
  }
  ofs << "  ]\n";
  ofs << "}\n";
  ofs.close();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("scanning_trajectory_node", node_options);

  // Safely get or declare parameters (handles automatically_declare_parameters_from_overrides)
  if (!node->has_parameter("max_steps")) node->declare_parameter<int>("max_steps", 0);
  if (!node->has_parameter("velocity_scaling")) node->declare_parameter<double>("velocity_scaling", 0.10);
  if (!node->has_parameter("acceleration_scaling")) node->declare_parameter<double>("acceleration_scaling", 0.10);
  if (!node->has_parameter("record_trajectory")) node->declare_parameter<bool>("record_trajectory", false);
  if (!node->has_parameter("trajectory_output_path")) node->declare_parameter<std::string>("trajectory_output_path", "recorded_scan_trajectory.json");
  if (!node->has_parameter("row_count")) node->declare_parameter<int>("row_count", 4);
  if (!node->has_parameter("row_spacing_m")) node->declare_parameter<double>("row_spacing_m", 0.15);
  if (!node->has_parameter("lateral_offset_m")) node->declare_parameter<double>("lateral_offset_m", 0.15);
  if (!node->has_parameter("look_angle_deg")) node->declare_parameter<double>("look_angle_deg", 15.0);

  int max_steps = node->get_parameter("max_steps").as_int();
  double max_vel_scaling = node->get_parameter("velocity_scaling").as_double();
  double max_acc_scaling = node->get_parameter("acceleration_scaling").as_double();
  bool record_trajectory = node->get_parameter("record_trajectory").as_bool();
  std::string output_path = node->get_parameter("trajectory_output_path").as_string();
  int row_count = node->get_parameter("row_count").as_int();
  double row_spacing_m = node->get_parameter("row_spacing_m").as_double();
  double lateral_offset_m = node->get_parameter("lateral_offset_m").as_double();
  double look_angle_deg = node->get_parameter("look_angle_deg").as_double();

  if (row_count < 1 ||
      row_spacing_m <= 0.0 ||
      lateral_offset_m <= 0.0 ||
      look_angle_deg < 0.0 ||
      look_angle_deg > 25.0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid scan geometry: row_count must be >= 1, row_spacing_m and "
      "lateral_offset_m must be > 0, and look_angle_deg must be in [0, 25].");
    rclcpp::shutdown();
    return 1;
  }

  // Spin executor in background thread for TF & joint_states subscriptions
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  RCLCPP_INFO(node->get_logger(), "==========================================================");
  RCLCPP_INFO(node->get_logger(), " Initializing Cartesian Scanning Trajectory Node (UR5e)");
  RCLCPP_INFO(node->get_logger(), "==========================================================");

  if (record_trajectory) {
    RCLCPP_INFO(node->get_logger(), "TRAJECTORY RECORDING ENABLED -> Output file: '%s'", output_path.c_str());
  } else {
    RCLCPP_INFO(node->get_logger(), "Trajectory Recording Disabled (set record_trajectory:=true to record)");
  }

  // 1. Planning group "ur5e_arm", tip link "pruner_link"
  static const std::string PLANNING_GROUP = "ur5e_arm";
  static const std::string EE_LINK = "pruner_link";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setEndEffectorLink(EE_LINK);

  RCLCPP_INFO(node->get_logger(), "Planning Frame:     %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End Effector Link:  %s", move_group.getEndEffectorLink().c_str());

  // Set velocity and acceleration scaling factors
  move_group.setMaxVelocityScalingFactor(max_vel_scaling);
  move_group.setMaxAccelerationScalingFactor(max_acc_scaling);
  RCLCPP_INFO(node->get_logger(), "SPEED SCALING: Max Velocity = %.2f (%.0f%%), Max Acceleration = %.2f (%.0f%%)",
              max_vel_scaling, max_vel_scaling * 100.0, max_acc_scaling, max_acc_scaling * 100.0);

  // ------------------------------------------------------------------
  // STEP 1: Move to Initial "home" Pose & Capture Fixed Reference Frame
  // ------------------------------------------------------------------
  RCLCPP_INFO(node->get_logger(), "\n--- STEP 1: Moving to Initial 'home' Pose ---");
  move_group.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  bool home_plan_ok = (move_group.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!home_plan_ok) {
    RCLCPP_ERROR(node->get_logger(), "Initial Home planning FAILED! Aborting.");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  size_t home_pts = home_plan.trajectory_.joint_trajectory.points.size();
  double home_dur = 0.0;
  if (home_pts > 0) {
    home_dur = rclcpp::Duration(home_plan.trajectory_.joint_trajectory.points.back().time_from_start).seconds();
  }

  std::cout << "\n========================================================================\n"
            << "Initial Home Plan computed: " << home_pts << " waypoints, estimated duration " << home_dur << " s.\n"
            << "Type 'yes' and press Enter to move to HOME pose on REAL robot: " << std::flush;

  std::string user_input;
  std::getline(std::cin, user_input);
  if (user_input != "yes") {
    RCLCPP_WARN(node->get_logger(), "Aborted by user prior to home move.");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 0;
  }

  RCLCPP_INFO(node->get_logger(), "Executing move to HOME pose...");
  moveit::core::MoveItErrorCode home_exec_res = move_group.execute(home_plan);
  if (home_exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Home move FAILED! (Error Code: %d). Aborting.", home_exec_res.val);
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Successfully arrived at HOME pose.");

  // ------------------------------------------------------------------
  // STEP 2: Capture Baseline Position & Orientation Reference
  // ------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped home_pose_stamped = move_group.getCurrentPose(EE_LINK);
  geometry_msgs::msg::Pose home_pose = home_pose_stamped.pose;

  Eigen::Vector3d home_pos(home_pose.position.x, home_pose.position.y, home_pose.position.z);
  Eigen::Quaterniond home_quat(home_pose.orientation.w, home_pose.orientation.x, home_pose.orientation.y, home_pose.orientation.z);
  home_quat.normalize();

  RCLCPP_INFO(node->get_logger(), "\n--- Captured Fixed Home Reference ---");
  RCLCPP_INFO(node->get_logger(), "  Home Position (world/base_link): [%.3f, %.3f, %.3f]", home_pos.x(), home_pos.y(), home_pos.z());
  RCLCPP_INFO(node->get_logger(), "  Home Orientation (Quat w,x,y,z):  [%.3f, %.3f, %.3f, %.3f]", home_quat.w(), home_quat.x(), home_quat.y(), home_quat.z());

  // ------------------------------------------------------------------
  // STEP 3: Define parameterized grid waypoint sequence
  // ------------------------------------------------------------------
  double look_rad = look_angle_deg * M_PI / 180.0;

  RCLCPP_INFO(node->get_logger(), "\n--- Configured Cartesian Scanning Step Parameters ---");
  RCLCPP_INFO(node->get_logger(), "  Row Count:                  %d", row_count);
  RCLCPP_INFO(node->get_logger(), "  Row Spacing:                %.3f m", row_spacing_m);
  RCLCPP_INFO(node->get_logger(), "  Lateral Offset:             +/- %.3f m", lateral_offset_m);
  RCLCPP_INFO(node->get_logger(), "  Look Rotation Angle:        %.1f deg", look_angle_deg);

  // Confirmed Rotation Axes via hardware axis_test:
  // - Local Y Axis (UnitY) = Up/Down Pitch Tilt (+look_rad = Down tilt)
  // - Local X Axis (UnitX) = Left/Right Yaw Pan  (+look_rad = Left pan, -look_rad = Right pan)
  Eigen::Quaterniond rot_pitch_down(Eigen::AngleAxisd(look_rad, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond rot_yaw_left(Eigen::AngleAxisd(look_rad, Eigen::Vector3d::UnitX()));
  Eigen::Quaterniond rot_yaw_right(Eigen::AngleAxisd(-look_rad, Eigen::Vector3d::UnitX()));

  Eigen::Quaterniond quat_look_down = (home_quat * rot_pitch_down).normalized();
  Eigen::Quaterniond quat_look_left = (home_quat * rot_yaw_left).normalized();
  Eigen::Quaterniond quat_look_right = (home_quat * rot_yaw_right).normalized();

  // Print Canonical Axis Rotation Diagnostic Offset (Purely computed, 0 robot movement)
  auto print_relative_rotation = [&](const std::string& label, const Eigen::Quaterniond& target_q) {
    Eigen::Quaterniond rel_q = (home_quat.inverse() * target_q).normalized();
    if (rel_q.w() < 0.0) {
      rel_q.w() = -rel_q.w();
      rel_q.x() = -rel_q.x();
      rel_q.y() = -rel_q.y();
      rel_q.z() = -rel_q.z();
    }
    double rot_x_deg = 2.0 * std::atan2(rel_q.x(), rel_q.w()) * 180.0 / M_PI;
    double rot_y_deg = 2.0 * std::atan2(rel_q.y(), rel_q.w()) * 180.0 / M_PI;
    double rot_z_deg = 2.0 * std::atan2(rel_q.z(), rel_q.w()) * 180.0 / M_PI;

    RCLCPP_INFO(node->get_logger(), "  %-16s -> Relative Local Axes [Local X: %6.1f deg, Local Y: %6.1f deg, Local Z: %6.1f deg]",
                label.c_str(), rot_x_deg, rot_y_deg, rot_z_deg);
  };

  RCLCPP_INFO(node->get_logger(), "\n--- Relative Waypoint Rotation Diagnostics (Relative to Home) ---");
  RCLCPP_INFO(node->get_logger(), "  %-16s -> Relative Local Axes [Local X:    0.0 deg, Local Y:    0.0 deg, Local Z:    0.0 deg] (Reference)", "home_quat");
  print_relative_rotation("quat_look_down", quat_look_down);
  print_relative_rotation("quat_look_left", quat_look_left);
  print_relative_rotation("quat_look_right", quat_look_right);

  // Construct absolute waypoints list:
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::vector<std::string> step_descriptions;
  std::vector<int> step_pause_ms; // Per-step pause duration in milliseconds

  size_t step_counter = 1;
  auto add_step = [&](const Eigen::Vector3d& pos, const Eigen::Quaterniond& q, const std::string& label, int pause_ms = 1000) {
    geometry_msgs::msg::Pose p;
    p.position.x = pos.x(); p.position.y = pos.y(); p.position.z = pos.z();
    p.orientation.w = q.w(); p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z();
    waypoints.push_back(p);

    std::string full_desc = std::to_string(step_counter++) + ". " + label;
    step_descriptions.push_back(full_desc);
    step_pause_ms.push_back(pause_ms);
  };

  // ------------------------------------------------------------------
  // BUILD PARAMETERIZED ROW SEQUENCE
  // ------------------------------------------------------------------
  // Standalone Pre-Row Step 1-3 (Look Down at Home)
  add_step(home_pos, home_quat,      "Home Base", 1000);
  add_step(home_pos, quat_look_down, "Initial Look Down", 1000);
  add_step(home_pos, home_quat,      "Return to Neutral Gaze", 1000);

  auto format_m = [](double value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    return stream.str();
  };

  // With the defaults this covers:
  // Row 0: z+0.00 m, Row 1: z+0.15 m, Row 2: z+0.30 m,
  // Row 3: z+0.45 m (one additional image-height row).
  for (int row = 0; row < row_count; ++row) {
    std::string r_prefix = "Row " + std::to_string(row) + ": ";
    Eigen::Vector3d row_center =
      home_pos + Eigen::Vector3d(0.0, 0.0, row * row_spacing_m);
    Eigen::Vector3d row_left =
      row_center + Eigen::Vector3d(0.0, lateral_offset_m, 0.0);
    Eigen::Vector3d row_right =
      row_center + Eigen::Vector3d(0.0, -lateral_offset_m, 0.0);

    // Streamlined 9-step pattern per row:
    add_step(
      row_left, home_quat,
      r_prefix + "Move Left (+Y " + format_m(lateral_offset_m) + "m)",
      1000);
    add_step(row_left,   quat_look_left,  r_prefix + "Look Left", 1000);
    add_step(row_left,   quat_look_right, r_prefix + "Look Right", 500); // 0.5s pause for quick head turn
    add_step(row_left,   home_quat,       r_prefix + "Return to Neutral Gaze", 1000);
    add_step(
      row_right, home_quat,
      r_prefix + "Move Right (-Y " + format_m(2.0 * lateral_offset_m) +
        "m direct)",
      1000);
    add_step(row_right,  quat_look_left,  r_prefix + "Look Left", 1000);
    add_step(row_right,  quat_look_right, r_prefix + "Look Right", 500); // 0.5s pause for quick head turn
    add_step(row_right,  home_quat,       r_prefix + "Return to Neutral Gaze", 1000);
    add_step(row_center, home_quat,       r_prefix + "Return to Center", 1000);

    // Move up to next row center if not the last row
    if (row < row_count - 1) {
      Eigen::Vector3d next_row_center =
        home_pos +
        Eigen::Vector3d(0.0, 0.0, (row + 1) * row_spacing_m);
      add_step(
        next_row_center, home_quat,
        "Move Up to Row " + std::to_string(row + 1) +
          " Center (+Z " + format_m(row_spacing_m) + "m)",
        1000);
    }
  }

  size_t total_waypoints_count = waypoints.size();

  // Log full waypoint list description for user review
  RCLCPP_INFO(node->get_logger(), "\n==========================================================");
  RCLCPP_INFO(node->get_logger(), " STREAMLINED WAYPOINT SEQUENCE (%zu STEPS TOTAL):", total_waypoints_count);
  RCLCPP_INFO(node->get_logger(), "==========================================================");
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto& p = waypoints[i];
    RCLCPP_INFO(node->get_logger(), "  %-44s [Pause: %4d ms] | Pos: [%.3f, %.3f, %.3f] | Quat: [%.3f, %.3f, %.3f, %.3f]",
                step_descriptions[i].c_str(), step_pause_ms[i], p.position.x, p.position.y, p.position.z,
                p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  }

  // Truncate waypoints if max_steps parameter is specified and valid
  if (max_steps > 0 && static_cast<size_t>(max_steps) < total_waypoints_count) {
    RCLCPP_INFO(node->get_logger(),
                "\nRunning LIMITED test: first %d of %zu TOTAL waypoints",
                max_steps, total_waypoints_count);
    waypoints.resize(static_cast<size_t>(max_steps));
    step_descriptions.resize(static_cast<size_t>(max_steps));
    step_pause_ms.resize(static_cast<size_t>(max_steps));
  } else {
    RCLCPP_INFO(node->get_logger(),
                "\nRunning FULL sequence: all %zu waypoints",
                total_waypoints_count);
  }

  // ------------------------------------------------------------------
  // STEP 4: Single Terminal Prompt for Sequence Execution
  // ------------------------------------------------------------------
  std::cout << "\n========================================================================\n"
            << "Ready to execute scanning sequence (" << waypoints.size() << " steps out of " << total_waypoints_count << " total).\n"
            << "Execution will run step-by-step at " << (max_vel_scaling * 100.0) << "% speed.\n"
            << "Type 'yes' and press Enter to proceed: " << std::flush;

  std::getline(std::cin, user_input);
  if (user_input != "yes") {
    RCLCPP_WARN(node->get_logger(), "Scanning sequence execution aborted by user.");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 0;
  }

  // ------------------------------------------------------------------
  // STEP 5: Per-Step Cartesian Planning & Execution Loop
  // ------------------------------------------------------------------
  double max_allowed_joint_jump_deg = 15.0; // 15 degrees max per step limit
  double max_allowed_joint_jump_rad = max_allowed_joint_jump_deg * M_PI / 180.0;

  bool sequence_ok = true;
  size_t active_steps = waypoints.size();

  std::vector<std::string> recorded_joint_names;
  std::vector<RecordedSegment> recorded_segments;

  for (size_t k = 0; k < active_steps; ++k) {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(node->get_logger(), "Interrupt signal caught! Halting sequence.");
      sequence_ok = false;
      break;
    }

    geometry_msgs::msg::Pose curr_pose = move_group.getCurrentPose(EE_LINK).pose;
    geometry_msgs::msg::Pose target_pose = waypoints[k];

    std::vector<geometry_msgs::msg::Pose> seg_waypoints = { curr_pose, target_pose };
    moveit_msgs::msg::RobotTrajectory seg_trajectory;

    double fraction = move_group.computeCartesianPath(seg_waypoints, 0.01, 0.0, seg_trajectory);
    double fraction_pct = fraction * 100.0;

    if (fraction < 0.95) {
      RCLCPP_ERROR(node->get_logger(), "Step %zu/%zu (%s) Cartesian path incomplete (%.1f%% < 95%%)! Aborting sequence.",
                   k + 1, active_steps, step_descriptions[k].c_str(), fraction_pct);
      sequence_ok = false;
      break;
    }

    // Time-parametrize segment trajectory at configured velocity/acceleration scaling
    robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), PLANNING_GROUP);
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), seg_trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if (!iptp.computeTimeStamps(rt, max_vel_scaling, max_acc_scaling)) {
      RCLCPP_ERROR(node->get_logger(), "Step %zu/%zu time parameterization failed! Aborting sequence.", k + 1, active_steps);
      sequence_ok = false;
      break;
    }

    moveit::planning_interface::MoveGroupInterface::Plan seg_plan;
    rt.getRobotTrajectoryMsg(seg_plan.trajectory_);

    // Perform Joint-Displacement Safety Check on segment points
    const auto& seg_pts = seg_plan.trajectory_.joint_trajectory.points;
    const auto& seg_jnames = seg_plan.trajectory_.joint_trajectory.joint_names;
    bool seg_safety_ok = true;

    for (size_t idx = 1; idx < seg_pts.size(); ++idx) {
      for (size_t j = 0; j < seg_pts[idx].positions.size(); ++j) {
        double diff_deg = std::abs(seg_pts[idx].positions[j] - seg_pts[idx - 1].positions[j]) * 180.0 / M_PI;
        if (diff_deg > max_allowed_joint_jump_deg) {
          std::string j_name = (j < seg_jnames.size()) ? seg_jnames[j] : std::to_string(j);
          RCLCPP_ERROR(node->get_logger(), "SAFETY VIOLATION at Step %zu/%zu (%s), Trajectory Point %zu!", k + 1, active_steps, step_descriptions[k].c_str(), idx);
          RCLCPP_ERROR(node->get_logger(), "  Joint '%s' displacement = %.2f deg (EXCEEDS %.1f deg limit)! Aborting.",
                       j_name.c_str(), diff_deg, max_allowed_joint_jump_deg);
          seg_safety_ok = false;
          break;
        }
      }
      if (!seg_safety_ok) break;
    }

    if (!seg_safety_ok) {
      sequence_ok = false;
      break;
    }

    double seg_dur_sec = 0.0;
    if (!seg_pts.empty()) {
      seg_dur_sec = rclcpp::Duration(seg_pts.back().time_from_start).seconds();
    }

    RCLCPP_INFO(node->get_logger(), "\n--- [Step %zu/%zu] %s ---",
                k + 1, active_steps, step_descriptions[k].c_str());
    RCLCPP_INFO(node->get_logger(), "  Target Pos: [%.3f, %.3f, %.3f] | Quat: [%.3f, %.3f, %.3f, %.3f]",
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
    RCLCPP_INFO(node->get_logger(), "  Segment Plan: %zu points, Estimated Duration = %.2f s (at %.0f%% speed)",
                seg_pts.size(), seg_dur_sec, max_vel_scaling * 100.0);

    // Dynamic per-step pause (0.5s for look-left->look-right, 1.0s for all other steps)
    int cur_pause_ms = step_pause_ms[k];
    RCLCPP_INFO(node->get_logger(), "Pausing %.1f second(s) before executing Step %zu/%zu...",
                cur_pause_ms / 1000.0, k + 1, active_steps);
    std::this_thread::sleep_for(std::chrono::milliseconds(cur_pause_ms));

    if (!rclcpp::ok()) {
      RCLCPP_WARN(node->get_logger(), "Interrupt caught during pause! Halting sequence.");
      sequence_ok = false;
      break;
    }

    RCLCPP_INFO(node->get_logger(), "Executing Step %zu/%zu on REAL robot...", k + 1, active_steps);
    moveit::core::MoveItErrorCode seg_exec_res = move_group.execute(seg_plan);

    if (seg_exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Step %zu/%zu Execution FAILED! (Error Code: %d). Aborting sequence.",
                   k + 1, active_steps, seg_exec_res.val);
      sequence_ok = false;
      break;
    }

    // Record trajectory segment if recording enabled
    if (record_trajectory) {
      if (recorded_joint_names.empty()) {
        recorded_joint_names = seg_plan.trajectory_.joint_trajectory.joint_names;
      }
      RecordedSegment rec_seg;
      rec_seg.step_index = k + 1;
      rec_seg.step_description = step_descriptions[k];
      rec_seg.pause_ms = step_pause_ms[k];
      for (const auto& pt : seg_pts) {
        RecordedPoint rpt;
        rpt.time_from_start_sec = rclcpp::Duration(pt.time_from_start).seconds();
        rpt.positions = pt.positions;
        rpt.velocities = pt.velocities;
        rpt.accelerations = pt.accelerations;
        rec_seg.points.push_back(rpt);
      }
      recorded_segments.push_back(rec_seg);
      save_recorded_json(output_path, recorded_joint_names, recorded_segments);
      RCLCPP_INFO(node->get_logger(), "RECORDED Step %zu/%zu (%zu points) -> Saved to '%s'",
                  k + 1, active_steps, seg_pts.size(), output_path.c_str());
    }

    geometry_msgs::msg::Pose actual_pose = move_group.getCurrentPose(EE_LINK).pose;
    RCLCPP_INFO(node->get_logger(), "Step %zu/%zu Complete. Actual Arrived Pose: [%.3f, %.3f, %.3f]",
                k + 1, active_steps, actual_pose.position.x, actual_pose.position.y, actual_pose.position.z);
  }

  // ------------------------------------------------------------------
  // STEP 6: Final Return to Home
  // ------------------------------------------------------------------
  if (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "\n--- Final Return to 'home' Pose ---");
    move_group.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan final_home_plan;

    if (move_group.plan(final_home_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Executing return to HOME...");
      moveit::core::MoveItErrorCode final_home_res = move_group.execute(final_home_plan);
      if (final_home_res == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Final return to HOME completed successfully.");
      } else {
        RCLCPP_ERROR(node->get_logger(), "Final return to HOME FAILED! (Error Code: %d).", final_home_res.val);
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "Final return to HOME planning FAILED!");
    }
  }

  if (record_trajectory && !recorded_segments.empty()) {
    RCLCPP_INFO(node->get_logger(), "\n==========================================================");
    RCLCPP_INFO(node->get_logger(), " TRAJECTORY RECORDING COMPLETE: %zu total segments saved to '%s'",
                recorded_segments.size(), output_path.c_str());
    RCLCPP_INFO(node->get_logger(), "==========================================================");
  }

  RCLCPP_INFO(node->get_logger(), "==========================================================");
  RCLCPP_INFO(node->get_logger(), " Cartesian Scanning Trajectory Node Finished.");
  RCLCPP_INFO(node->get_logger(), "==========================================================");

  executor.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}
