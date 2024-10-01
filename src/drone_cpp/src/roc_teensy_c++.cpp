#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <iostream>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <vector>

#define FLOAT_SIZE sizeof(float)
#define INT_SIZE sizeof(int)

class UDPServer : public rclcpp::Node {
public:
    UDPServer(const std::string& server_ip, int server_port, const std::string& client_ip, int client_port)
        : Node("udp_server"), server_ip_(server_ip), server_port_(server_port), client_ip_(client_ip), client_port_(client_port) {

        // Initialize server and client addresses
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket.");
            return;
        }

        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_addr.s_addr = inet_addr(server_ip.c_str());
        server_addr_.sin_port = htons(server_port);

        memset(&client_addr_, 0, sizeof(client_addr_));
        client_addr_.sin_family = AF_INET;
        client_addr_.sin_addr.s_addr = inet_addr(client_ip.c_str());
        client_addr_.sin_port = htons(client_port);

        imu_pub_mpu6050_ = this->create_publisher<sensor_msgs::msg::Imu>("mpu6050_imu", 10);
        imu_pub_pololu_ = this->create_publisher<sensor_msgs::msg::Imu>("pololu_imu", 10);
        imu_pub_fuse_ = this->create_publisher<sensor_msgs::msg::Imu>("fuse_imu", 10);
        quaternion_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("quaternion", 10);
        euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("euler_angles", 10);
        rc_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("rc_channels", 10);

        // Start receiver thread
        std::thread([this]() { this->receiver(); }).detach();
    }

private:
    int sockfd_;
    struct sockaddr_in server_addr_, client_addr_;
    std::string server_ip_;
    int server_port_;
    std::string client_ip_;
    int client_port_;

    // ROS 2 publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_mpu6050_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_pololu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_fuse_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr quaternion_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rc_pub_;

    void receiver() {
        if (bind(sockfd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket.");
            return;
        }

        char buffer[1024];
        socklen_t len = sizeof(client_addr_);

        // Perform SYN-ACK handshake
        sendto(sockfd_, "\x01\x01", 2, 0, (struct sockaddr *)&client_addr_, len);
        recvfrom(sockfd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr_, &len);
        if (buffer[0] != '\x01' || buffer[1] != '\x02') {
            RCLCPP_ERROR(this->get_logger(), "Failed SYN-ACK handshake.");
            return;
        }

        sendto(sockfd_, "\x01\x03", 2, 0, (struct sockaddr *)&client_addr_, len);
        RCLCPP_INFO(this->get_logger(), "Connection established.");

        while (true) {
            int n = recvfrom(sockfd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr_, &len);
            if (n <= 0) continue;

            if (buffer[0] == '\x01' && buffer[1] == '\x10') {
                buffer[1] = '\x11';
                sendto(sockfd_, buffer, 3, 0, (struct sockaddr *)&client_addr_, len);
            } else {
                process_message(buffer, n);
            }
        }
    }

    void process_message(const char* message, size_t length) {
        char type = message[0];
        const char* payload = message + 1;

        switch (type) {
            case 'm': {
                std::vector<float> imu_data = unpack_floats(payload, 6);
                publish_imu(imu_data, imu_pub_mpu6050_);
                break;
            }
            case 'c': {
                std::vector<float> euler_data = unpack_floats(payload, 3);
                std::vector<float> quaternion_data = unpack_floats(payload + 3 * FLOAT_SIZE, 4);
                std::vector<float> fuse_imu_data = unpack_floats(payload + 7 * FLOAT_SIZE, 6);

                publish_euler(euler_data);
                publish_quaternion(quaternion_data);
                publish_imu(fuse_imu_data, imu_pub_fuse_);
                break;
            }
            case 'p': {
                std::vector<float> pololu_data = unpack_floats(payload, 6);
                publish_imu(pololu_data, imu_pub_pololu_);
                break;
            }
            case 'r': {
                std::vector<int> rc_data = unpack_ints(payload, 16);
                publish_rc(rc_data);
                break;
            }
            default:
                break;
        }
    }

    std::vector<float> unpack_floats(const char* data, size_t count) {
        std::vector<float> values(count);
        memcpy(values.data(), data, count * FLOAT_SIZE);
        return values;
    }

    std::vector<int> unpack_ints(const char* data, size_t count) {
        std::vector<int> values(count);
        memcpy(values.data(), data, count * INT_SIZE);
        return values;
    }

    void publish_imu(const std::vector<float>& imu_data, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher) {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.linear_acceleration.x = imu_data[0];
        imu_msg.linear_acceleration.y = imu_data[1];
        imu_msg.linear_acceleration.z = imu_data[2];
        imu_msg.angular_velocity.x = imu_data[3];
        imu_msg.angular_velocity.y = imu_data[4];
        imu_msg.angular_velocity.z = imu_data[5];
        publisher->publish(imu_msg);
    }

    void publish_quaternion(const std::vector<float>& quaternion_data) {
        auto quat_msg = geometry_msgs::msg::Quaternion();
        quat_msg.x = quaternion_data[0];
        quat_msg.y = quaternion_data[1];
        quat_msg.z = quaternion_data[2];
        quat_msg.w = quaternion_data[3];
        quaternion_pub_->publish(quat_msg);
    }

    void publish_euler(const std::vector<float>& euler_data) {
        auto euler_msg = geometry_msgs::msg::Vector3();
        euler_msg.x = euler_data[0];
        euler_msg.y = euler_data[1];
        euler_msg.z = euler_data[2];
        euler_pub_->publish(euler_msg);
    }

    void publish_rc(const std::vector<int>& rc_data) {
        auto rc_msg = std_msgs::msg::Int32MultiArray();
        rc_msg.data = rc_data;
        rc_pub_->publish(rc_msg);
    }
};

void start_udp_server() {
    rclcpp::init(0, nullptr);

    auto server = std::make_shared<UDPServer>("0.0.0.0", 12000, "192.168.1.199", 8888);

    rclcpp::spin(server);
    rclcpp::shutdown();
}

int main(int argc, char* argv[]) {
    start_udp_server();
    return 0;
}
