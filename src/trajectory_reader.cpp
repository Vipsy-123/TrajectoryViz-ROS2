#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>
#include <vector>

class TrajectoryReader : public rclcpp::Node {
public:
    TrajectoryReader() : Node("trajectory_reader") {
        // Publisher for trajectory visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_markers", 10);

        // Load the saved trajectory and publish markers
        load_and_publish_trajectory();
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Load trajectory data from a file and publish it as markers
    void load_and_publish_trajectory() {
        std::vector<geometry_msgs::msg::PoseStamped> trajectory = load_trajectory("recorded_trajectory.csv");

        // Check if trajectory data is available before publishing
        if (trajectory.empty()) {
            RCLCPP_WARN(this->get_logger(), "No trajectory data available!");
            return;
        }

        publishMarkers(trajectory);
    }

    // Reads the trajectory data from a CSV file and returns a list of poses
    std::vector<geometry_msgs::msg::PoseStamped> load_trajectory(const std::string& filename) {
        std::vector<geometry_msgs::msg::PoseStamped> trajectory;
        std::ifstream file(filename);

        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file.");
            return trajectory;
        }

        std::string line;
        std::getline(file, line); // Skip the first line (header)

        // Read each line containing trajectory data
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            geometry_msgs::msg::PoseStamped pose;

            // Parse timestamp (seconds and nanoseconds)
            std::getline(ss, token, ',');
            pose.header.stamp.sec = std::stoi(token);
            pose.header.stamp.nanosec = std::stoi(token.substr(token.find('.') + 1));

            // Parse X, Y, and Z coordinates
            std::getline(ss, token, ',');
            pose.pose.position.x = std::stod(token);

            std::getline(ss, token, ',');
            pose.pose.position.y = std::stod(token);

            std::getline(ss, token, ',');
            pose.pose.position.z = std::stod(token);

            pose.header.frame_id = "odom";
            trajectory.push_back(pose);
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points.", trajectory.size());
        return trajectory;
    }

    // Publishes the trajectory as visualization markers in RViz
    void publishMarkers(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory) {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < trajectory.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";  
            marker.header.stamp = this->now();
            marker.ns = "trajectory";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = trajectory[i].pose.position;

            // Set marker size
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;

            // Set marker color
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %zu trajectory markers.", trajectory.size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryReader>());
    rclcpp::shutdown();
    return 0;
}

