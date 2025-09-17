#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include "anscer_navigation/action/multi_map_navigate.hpp"

#include <sqlite3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <thread>

using MultiMapNavigate = anscer_navigation::action::MultiMapNavigate;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

struct Wormhole
{
  std::string from;
  std::string to;
  double x, y, yaw;
};

class MultiMapNavServer : public rclcpp::Node
{
public:
  MultiMapNavServer()
      : Node("multi_map_node")
  {
    action_server_ = rclcpp_action::create_server<MultiMapNavigate>(
        this,
        "multi_map_navigate",
        std::bind(&MultiMapNavServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MultiMapNavServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MultiMapNavServer::handle_accepted, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    std::string db_path = "/home/innomer/Multi-Map-Navigation/maps/multimap.db";
    if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK)
    {
      RCLCPP_ERROR(this->get_logger(), "Cannot open wormhole DB!");
    }

    current_map_ = "Hallway"; // default starting map
  }

  ~MultiMapNavServer() { sqlite3_close(db_); }

private:
  rclcpp_action::Server<MultiMapNavigate>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  sqlite3 *db_;
  std::string current_map_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const MultiMapNavigate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: %s (%.2f, %.2f, %.2f)",
                goal->target_map.c_str(), goal->x, goal->y, goal->yaw);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiMapNavigate>>)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiMapNavigate>> goal_handle)
  {
    std::thread([this, goal_handle]()
                {
      auto goal = goal_handle->get_goal();
      auto result = std::make_shared<MultiMapNavigate::Result>();

      if (goal->target_map == current_map_) {
        if (!navigate_to(goal->x, goal->y, goal->yaw)) {
          result->success = false;
          result->message = "Failed to navigate in current map";
          goal_handle->abort(result);
          return;
        }
      } else {
        auto path = find_path(current_map_, goal->target_map);
        if (path.empty()) {
          result->success = false;
          result->message = "No wormhole path found";
          goal_handle->abort(result);
          return;
        }

        for (auto &step : path) {
          RCLCPP_INFO(this->get_logger(), "Traversing wormhole %s -> %s", step.from.c_str(), step.to.c_str());
          if (!navigate_to(step.x, step.y, step.yaw)) {
            result->success = false;
            result->message = "Failed to reach wormhole";
            goal_handle->abort(result);
            return;
          }
          if (!switch_map(step.to)) {
            result->success = false;
            result->message = "Failed to switch map";
            goal_handle->abort(result);
            return;
          }
          current_map_ = step.to;
        }

        if (!navigate_to(goal->x, goal->y, goal->yaw)) {
          result->success = false;
          result->message = "Final navigation failed";
          goal_handle->abort(result);
          return;
        }
      }

      result->success = true;
      result->message = "Navigation complete";
      goal_handle->succeed(result); })
        .detach();
  }

  std::vector<Wormhole> load_wormholes()
  {
    std::vector<Wormhole> wormholes;
    const char *sql = "SELECT map_from, map_to, x, y, yaw FROM wormhole;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) == SQLITE_OK)
    {
      while (sqlite3_step(stmt) == SQLITE_ROW)
      {
        Wormhole w;
        w.from = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
        w.to = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
        w.x = sqlite3_column_double(stmt, 2);
        w.y = sqlite3_column_double(stmt, 3);
        w.yaw = sqlite3_column_double(stmt, 4);
        wormholes.push_back(w);
        RCLCPP_INFO(this->get_logger(),
                    "Loaded wormhole: %s -> %s (%.2f, %.2f, %.2f)",
                    w.from.c_str(), w.to.c_str(), w.x, w.y, w.yaw);
      }
    }
    sqlite3_finalize(stmt);
    return wormholes;
  }

  std::vector<Wormhole> find_path(const std::string &start, const std::string &goal)
  {
    auto wormholes = load_wormholes();
    std::unordered_map<std::string, std::vector<Wormhole>> graph;
    for (auto &w : wormholes)
    {
      graph[w.from].push_back(w);
      RCLCPP_INFO(this->get_logger(), "Graph edge: %s -> %s", w.from.c_str(), w.to.c_str());
    }
    std::queue<std::pair<std::string, std::vector<Wormhole>>> q;
    q.push({start, {}});

    while (!q.empty())
    {
      auto [cur, path] = q.front();
      RCLCPP_INFO(this->get_logger(), "Visiting map: %s, path length: %zu", cur.c_str(), path.size());

      q.pop();

      if (cur == goal)
        return path;

      for (auto &edge : graph[cur])
      {
        // check if this wormhole is already in path to prevent cycles
        bool in_path = false;
        for (auto &p : path)
        {
          if (p.from == edge.from && p.to == edge.to)
          {
            in_path = true;
            break;
          }
        }
        if (in_path)
          continue;

        auto new_path = path;
        new_path.push_back(edge);
        q.push({edge.to, new_path});
      }
    }

    return {}; // no path found
  }

  bool navigate_to(double x, double y, double yaw)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5)))
      return false;
    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    auto future = nav_client_->async_send_goal(goal_msg);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
      return false;
    auto goal_handle = future.get();
    if (!goal_handle)
      return false;
    auto result_future = nav_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::minutes(2)) != std::future_status::ready)
      return false;
    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  bool switch_map(const std::string &map_name)
  {
    auto client = this->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map");
    if (!client->wait_for_service(std::chrono::seconds(5)))
      return false;
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = "maps/" + map_name + "_map.yaml";
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
      return false;
    auto result = future.get();
    return result->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiMapNavServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
