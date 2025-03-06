#include <rclcpp/rclcpp.hpp>
#include <servo_msgs/srv/save_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fstream>
#include <vector>

class TrajectoryRecorder : public rclcpp::Node {
public:
    TrajectoryRecorder() : Node("trajectory_recorder") {
        // Service to start trajectory recording
        save_trajectory_service_ = this->create_service<servo_msgs::srv::SaveTrajectory>(
            "save_trajectory",
            std::bind(&TrajectoryRecorder::save_trajectory_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Subscribe to odometry topic to track robot movement
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajectoryRecorder::odom_callback, this, std::placeholders::_1)
        );

        // Publisher for visualizing the trajectory in RViz
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trajectory_markers", 10
        );
    }

private:
    rclcpp::Service<servo_msgs::srv::SaveTrajectory>::SharedPtr save_trajectory_service_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr recording_timer_;

    std::vector<geometry_msgs::msg::PoseStamped> positions;          // Stores ongoing odometry positions
    std::vector<geometry_msgs::msg::PoseStamped> recorded_positions; // Stores final recorded trajectory

    rclcpp::Time start_time_;
    double recording_duration_;

    // Service callback: starts recording for a given duration
    void save_trajectory_callback(
        const std::shared_ptr<servo_msgs::srv::SaveTrajectory::Request> request,
        std::shared_ptr<servo_msgs::srv::SaveTrajectory::Response> response)
    {
        recorded_positions.clear();  // Clear previous recordings
        start_time_ = this->now();    // Start timing the recording
        recording_duration_ = request->duration;

        RCLCPP_INFO(this->get_logger(), "Starting trajectory recording for %.2f seconds...", recording_duration_);

        // Start a timer that records positions every 100ms
        recording_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajectoryRecorder::record_position, this)
        );

        response->success = true;
        response->message = "Trajectory recording started.";
    }

    // Periodically records positions within the specified duration
    void record_position() {
        if ((this->now() - start_time_).seconds() >= recording_duration_) {
            recording_timer_->cancel();  // Stop recording when time is up
            RCLCPP_INFO(this->get_logger(), "Recording complete. Saving trajectory...");
            save_to_file();
            return;
        }

        // Ensure there's data before recording
        if (!positions.empty()) {
            const geometry_msgs::msg::PoseStamped& current_pose = positions.back();
            if (recorded_positions.empty() || has_moved(recorded_positions.back(), current_pose)) {
                recorded_positions.push_back(current_pose);
                publishMarkers(); // Visualize trajectory points in RViz
            }
        }
    }

    // Saves the recorded trajectory to a CSV file
    void save_to_file() {
        std::ofstream file("recorded_trajectory.csv");
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing.");
            return;
        }

        file << "Timestamp,X,Y,Z\n";
        for (const auto& pose : recorded_positions) {
            file << pose.header.stamp.sec << "."
                 << pose.header.stamp.nanosec << ","
                 << pose.pose.position.x << ","
                 << pose.pose.position.y << ","
                 << pose.pose.position.z << "\n";
        }
        file.close();

        RCLCPP_INFO(this->get_logger(), "Trajectory saved to recorded_trajectory.csv");
    }

    // Checks if the robot has moved significantly from the last recorded position
    bool has_moved(const geometry_msgs::msg::PoseStamped& last, const geometry_msgs::msg::PoseStamped& current) {
        return std::abs(current.pose.position.x - last.pose.position.x) > 0.01 ||
               std::abs(current.pose.position.y - last.pose.position.y) > 0.01 ||
               std::abs(current.pose.position.z - last.pose.position.z) > 0.01;
    }

    // Callback function: Updates positions based on odometry data
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position = msg->pose.pose.position;
        pose.header.stamp = this->now(); // Timestamp the position
        positions.push_back(pose);

        // Keep memory usage in check by limiting the number of stored positions
        if (positions.size() > 1000) {
            positions.erase(positions.begin());
        }

        publishMarkers(); // Update trajectory visualization
    }

    // Publishes trajectory markers to RViz
    void publishMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t i = 0; i < positions.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";  // Markers are in the odom frame
            marker.header.stamp = this->now();
            marker.ns = "trajectory";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = positions[i].pose.position;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0;
            marker.color.r = 1.0;  // Red color for markers
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>());
    rclcpp::shutdown();
    return 0;
}

