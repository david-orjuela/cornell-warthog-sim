#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <map>

using json = nlohmann::json;

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

bool load_recorded_json(const std::string& filepath,
                        std::vector<std::string>& joint_names,
                        std::vector<RecordedSegment>& segments,
                        std::string& error_msg)
{
  std::ifstream ifs(filepath);
  if (!ifs.is_open()) {
    error_msg = "Could not open JSON trajectory file: " + filepath;
    return false;
  }

  try {
    json j = json::parse(ifs);
    joint_names = j.at("joint_names").get<std::vector<std::string>>();

    for (const auto& seg_json : j.at("segments")) {
      RecordedSegment seg;
      seg.step_index = seg_json.at("step_index").get<size_t>();
      seg.step_description = seg_json.at("step_description").get<std::string>();
      seg.pause_ms = seg_json.at("pause_ms").get<int>();

      for (const auto& pt_json : seg_json.at("points")) {
        RecordedPoint rpt;
        rpt.time_from_start_sec = pt_json.at("time_from_start_sec").get<double>();
        rpt.positions = pt_json.at("positions").get<std::vector<double>>();
        if (pt_json.contains("velocities")) {
          rpt.velocities = pt_json.at("velocities").get<std::vector<double>>();
        }
        if (pt_json.contains("accelerations")) {
          rpt.accelerations = pt_json.at("accelerations").get<std::vector<double>>();
        }
        seg.points.push_back(rpt);
      }
      segments.push_back(seg);
    }
  } catch (const std::exception& e) {
    error_msg = std::string("JSON parsing error: ") + e.what();
    return false;
  }

  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("scanning_trajectory", node_options);

  if (!node->has_parameter("trajectory_input_path")) {
    node->declare_parameter<std::string>("trajectory_input_path", "recorded_scan_trajectory.json");
  }
  if (!node->has_parameter("max_steps")) {
    node->declare_parameter<int>("max_steps", 0);
  }
  if (!node->has_parameter("pause_ms")) {
    node->declare_parameter<int>("pause_ms", 300);
  }

  std::string input_path = node->get_parameter("trajectory_input_path").as_string();
  int max_steps = node->get_parameter("max_steps").as_int();
  int override_pause_ms = node->get_parameter("pause_ms").as_int();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  std::vector<std::string> joint_names;
  std::vector<RecordedSegment> segments;
  std::string err_msg;

  if (!load_recorded_json(input_path, joint_names, segments, err_msg)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load trajectory: %s", err_msg.c_str());
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  if (segments.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Trajectory file is empty.");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  size_t active_segments = (max_steps > 0 && static_cast<size_t>(max_steps) < segments.size())
                           ? static_cast<size_t>(max_steps)
                           : segments.size();

  RCLCPP_INFO(node->get_logger(), "Loaded scanning trajectory: %zu steps (%zu joints, input: '%s').",
              active_segments, joint_names.size(), input_path.c_str());

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
    node, "/joint_trajectory_controller/follow_joint_trajectory");

  if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Action server unavailable.");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  sensor_msgs::msg::JointState::SharedPtr latest_joint_state;
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [&latest_joint_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
      latest_joint_state = msg;
    });

  auto start_wait = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !latest_joint_state) {
    if (std::chrono::steady_clock::now() - start_wait > std::chrono::seconds(5)) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!latest_joint_state) {
    RCLCPP_ERROR(node->get_logger(), "Failed to receive /joint_states within 5 seconds.");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  std::map<std::string, double> current_positions;
  for (size_t i = 0; i < latest_joint_state->name.size(); ++i) {
    if (i < latest_joint_state->position.size()) {
      current_positions[latest_joint_state->name[i]] = latest_joint_state->position[i];
    }
  }

  const auto& expected_start_pt = segments[0].points[0];
  double max_allowed_start_diff_deg = 5.0;
  double max_allowed_start_diff_rad = max_allowed_start_diff_deg * M_PI / 180.0;
  bool start_pose_ok = true;
  double max_found_diff_deg = 0.0;

  for (size_t j = 0; j < joint_names.size(); ++j) {
    const std::string& j_name = joint_names[j];
    if (current_positions.find(j_name) == current_positions.end()) {
      RCLCPP_ERROR(node->get_logger(), "Joint '%s' missing from /joint_states.", j_name.c_str());
      start_pose_ok = false;
      break;
    }

    double cur_pos_rad = current_positions[j_name];
    double exp_pos_rad = expected_start_pt.positions[j];
    double diff_deg = std::abs(cur_pos_rad - exp_pos_rad) * 180.0 / M_PI;
    max_found_diff_deg = std::max(max_found_diff_deg, diff_deg);

    if (diff_deg > max_allowed_start_diff_deg) {
      start_pose_ok = false;
    }
  }

  // Move to Home if robot is away from Home pose
  if (!start_pose_ok) {
    std::cout << "\nRobot is NOT at HOME pose (Max diff: " << max_found_diff_deg << " deg).\n"
              << "Type 'yes' and press Enter to smoothly move arm to HOME pose first: " << std::flush;

    std::string user_input_home;
    std::getline(std::cin, user_input_home);
    if (user_input_home != "yes") {
      RCLCPP_WARN(node->get_logger(), "Initial move-to-home aborted.");
      executor.cancel();
      if (spinner.joinable()) spinner.join();
      rclcpp::shutdown();
      return 0;
    }

    FollowJointTrajectory::Goal move_home_goal;
    move_home_goal.trajectory.joint_names = joint_names;

    double home_move_dur = 5.0;
    int num_pts = 25;

    std::vector<double> start_vec;
    for (size_t j = 0; j < joint_names.size(); ++j) {
      start_vec.push_back(current_positions[joint_names[j]]);
    }

    for (int i = 0; i <= num_pts; ++i) {
      double t = home_move_dur * (static_cast<double>(i) / num_pts);
      double s = 0.5 * (1.0 - std::cos(M_PI * t / home_move_dur));
      double s_dot = (M_PI / (2.0 * home_move_dur)) * std::sin(M_PI * t / home_move_dur);

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.resize(joint_names.size());
      pt.velocities.resize(joint_names.size(), 0.0);
      pt.accelerations.resize(joint_names.size(), 0.0);

      for (size_t j = 0; j < joint_names.size(); ++j) {
        double delta = expected_start_pt.positions[j] - start_vec[j];
        pt.positions[j] = start_vec[j] + s * delta;
        pt.velocities[j] = s_dot * delta;
      }
      pt.time_from_start = rclcpp::Duration::from_seconds(t);
      move_home_goal.trajectory.points.push_back(pt);
    }

    RCLCPP_INFO(node->get_logger(), "Moving arm to HOME pose (5.0s)...");
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    auto goal_handle_future = action_client->async_send_goal(move_home_goal, send_goal_options);

    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
        !goal_handle_future.get()) {
      RCLCPP_ERROR(node->get_logger(), "Move to HOME failed.");
      executor.cancel();
      if (spinner.joinable()) spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    auto result_future = action_client->async_get_result(goal_handle_future.get());
    if (result_future.wait_for(std::chrono::duration<double>(home_move_dur + 5.0)) != std::future_status::ready ||
        result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(node->get_logger(), "Move to HOME execution failed.");
      executor.cancel();
      if (spinner.joinable()) spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Arrived at HOME pose.");
  }

  std::cout << "\nExecute hardcoded scanning replay (" << active_segments << " steps)? Type 'yes': " << std::flush;
  std::string user_input;
  std::getline(std::cin, user_input);
  if (user_input != "yes") {
    RCLCPP_WARN(node->get_logger(), "Replay aborted.");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 0;
  }

  bool replay_ok = true;
  for (size_t k = 0; k < active_segments; ++k) {
    if (!rclcpp::ok()) {
      replay_ok = false;
      break;
    }

    const auto& seg = segments[k];
    int cur_pause_ms = (override_pause_ms > 0) ? override_pause_ms : seg.pause_ms;
    std::this_thread::sleep_for(std::chrono::milliseconds(cur_pause_ms));

    if (!rclcpp::ok()) break;

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names;

    for (const auto& rpt : seg.points) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = rpt.positions;
      pt.velocities = rpt.velocities;
      pt.accelerations = rpt.accelerations;
      pt.time_from_start = rclcpp::Duration::from_seconds(rpt.time_from_start_sec);
      goal.trajectory.points.push_back(pt);
    }

    double seg_duration = seg.points.empty() ? 0.0 : seg.points.back().time_from_start_sec;
    RCLCPP_INFO(node->get_logger(), "[Step %zu/%zu] %s", k + 1, active_segments, seg.step_description.c_str());

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    auto goal_handle_future = action_client->async_send_goal(goal, send_goal_options);

    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
        !goal_handle_future.get()) {
      RCLCPP_ERROR(node->get_logger(), "[Step %zu/%zu] Send failed.", k + 1, active_segments);
      replay_ok = false;
      break;
    }

    auto result_future = action_client->async_get_result(goal_handle_future.get());
    if (result_future.wait_for(std::chrono::duration<double>(seg_duration + 5.0)) != std::future_status::ready ||
        result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(node->get_logger(), "[Step %zu/%zu] Execution failed.", k + 1, active_segments);
      replay_ok = false;
      break;
    }
  }

  // Smooth final return to Home pose at the end of scanning
  if (replay_ok && active_segments == segments.size() && rclcpp::ok()) {
    const auto& home_positions = segments[0].points[0].positions;
    const auto& last_segment_pts = segments.back().points;
    if (!last_segment_pts.empty()) {
      const auto& current_end_positions = last_segment_pts.back().positions;
      double max_dist_to_home = 0.0;
      for (size_t j = 0; j < home_positions.size(); ++j) {
        max_dist_to_home = std::max(max_dist_to_home, std::abs(current_end_positions[j] - home_positions[j]));
      }

      if (max_dist_to_home > 0.0175) {
        RCLCPP_INFO(node->get_logger(), "Returning arm to HOME pose...");
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        FollowJointTrajectory::Goal home_goal;
        home_goal.trajectory.joint_names = joint_names;

        double return_dur = 3.5;
        int num_pts = 20;

        for (int i = 0; i <= num_pts; ++i) {
          double t = return_dur * (static_cast<double>(i) / num_pts);
          double s = 0.5 * (1.0 - std::cos(M_PI * t / return_dur));
          double s_dot = (M_PI / (2.0 * return_dur)) * std::sin(M_PI * t / return_dur);

          trajectory_msgs::msg::JointTrajectoryPoint pt;
          pt.positions.resize(home_positions.size());
          pt.velocities.resize(home_positions.size(), 0.0);
          pt.accelerations.resize(home_positions.size(), 0.0);

          for (size_t j = 0; j < home_positions.size(); ++j) {
            double delta = home_positions[j] - current_end_positions[j];
            pt.positions[j] = current_end_positions[j] + s * delta;
            pt.velocities[j] = s_dot * delta;
          }
          pt.time_from_start = rclcpp::Duration::from_seconds(t);
          home_goal.trajectory.points.push_back(pt);
        }

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        auto goal_handle_future = action_client->async_send_goal(home_goal, send_goal_options);

        if (goal_handle_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready && goal_handle_future.get()) {
          auto result_future = action_client->async_get_result(goal_handle_future.get());
          result_future.wait_for(std::chrono::duration<double>(return_dur + 5.0));
        }
      }
    }
  }

  if (replay_ok) {
    RCLCPP_INFO(node->get_logger(), "Scanning trajectory replay completed successfully.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Scanning trajectory replay aborted due to error.");
  }

  executor.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}
