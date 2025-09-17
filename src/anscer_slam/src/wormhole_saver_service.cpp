#include <rclcpp/rclcpp.hpp>
#include "anscer_slam/srv/save_wormhole.hpp"
#include <sqlite3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <cstdlib>

class WormholeSaverService : public rclcpp::Node
{
public:
    WormholeSaverService() : Node("wormhole_saver_service"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Create service
        service_ = this->create_service<anscer_slam::srv::SaveWormhole>(
            "save_wormhole",
            std::bind(&WormholeSaverService::handle_save_wormhole, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // Open SQLite DB
        std::string db_path = "maps/multimap.db";
        if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open wormhole DB!");
        } else {
            const char* sql_create = "CREATE TABLE IF NOT EXISTS wormhole ("
                                     "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                                     "map_from TEXT NOT NULL, "
                                     "map_to TEXT NOT NULL, "
                                     "x REAL NOT NULL, "
                                     "y REAL NOT NULL, "
                                     "yaw REAL NOT NULL);";
            char* errmsg;
            sqlite3_exec(db_, sql_create, nullptr, nullptr, &errmsg);
            if (errmsg) {
                RCLCPP_ERROR(this->get_logger(), "SQLite error: %s", errmsg);
                sqlite3_free(errmsg);
            }
        }
    }

    ~WormholeSaverService() {
        sqlite3_close(db_);
    }

private:
    rclcpp::Service<anscer_slam::srv::SaveWormhole>::SharedPtr service_;
    sqlite3* db_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void handle_save_wormhole(
        const std::shared_ptr<anscer_slam::srv::SaveWormhole::Request> request,
        std::shared_ptr<anscer_slam::srv::SaveWormhole::Response> response)
    {
        std::string map_from = request->map_from;
        std::string map_to = request->map_to;
        std::string map_file = request->map_name;

        // 1. Get robot pose from TF
        geometry_msgs::msg::PoseStamped pose;
        try {
            auto transformStamped = tf_buffer_.lookupTransform(
                "map", "base_link", rclcpp::Time(0), rclcpp::Duration(1,0));
            pose.header = transformStamped.header;
            pose.pose.position.x = transformStamped.transform.translation.x;
            pose.pose.position.y = transformStamped.transform.translation.y;
            pose.pose.position.z = transformStamped.transform.translation.z;
            pose.pose.orientation = transformStamped.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
            response->success = false;
            response->message = "TF lookup failed";
            return;
        }

        // 2. Save the map using map_saver_cli
        std::string save_cmd = "ros2 run nav2_map_server map_saver_cli -f maps/" + map_file;
        RCLCPP_INFO(this->get_logger(), "Saving map: %s", map_file.c_str());
        int ret = std::system(save_cmd.c_str());
        if (ret != 0) {
            response->success = false;
            response->message = "Map saving failed";
            return;
        }

        // 3. Insert wormhole into DB
        char* errmsg;
        std::string sql_insert = "INSERT INTO wormhole(map_from, map_to, x, y, yaw) VALUES("
                                 "'" + map_from + "', '" + map_to + "', " +
                                 std::to_string(pose.pose.position.x) + ", " +
                                 std::to_string(pose.pose.position.y) + ", " +
                                 std::to_string(get_yaw_from_quaternion(pose.pose.orientation)) +
                                 ");";

        if (sqlite3_exec(db_, sql_insert.c_str(), nullptr, nullptr, &errmsg) != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "SQLite insert failed: %s", errmsg);
            sqlite3_free(errmsg);
            response->success = false;
            response->message = "DB insert failed";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Map saved and wormhole recorded: %s -> %s",
                    map_from.c_str(), map_to.c_str());
        response->success = true;
        response->message = "Success";
    }

    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        return std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WormholeSaverService>());
    rclcpp::shutdown();
    return 0;
}
