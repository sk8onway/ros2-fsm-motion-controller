#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"

using std::placeholders::_1;

turtlesim::msg::Pose::SharedPtr g_pose;
turtlesim::msg::Pose g_goal;

enum State {
    FORWARD,
    STOP_FORWARD,
    TURN,
    STOP_TURN
};

State g_state = FORWARD;
bool g_first_goal_set = false;
bool g_new_goal = true;

#define PI 3.14159265

// 🔥 Normalize angle to [-pi, pi]
double normalize_angle(double angle) {
    return atan2(sin(angle), cos(angle));
}

void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    g_pose = msg;
}

bool hasReachedPosition() {
    double dist = hypot(g_pose->x - g_goal.x, g_pose->y - g_goal.y);
    return dist < 0.1;
}

bool hasReachedAngle() {
    double error = normalize_angle(g_goal.theta - g_pose->theta);
    return fabs(error) < 0.01;
}

bool hasStopped() {
    return fabs(g_pose->linear_velocity) < 0.0001 &&
           fabs(g_pose->angular_velocity) < 0.0001;
}

void commandTurtle(
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
    float linear, float angular)
{
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear;
    twist.angular.z = angular;
    pub->publish(twist);
}

void printGoal(rclcpp::Logger logger) {
    RCLCPP_INFO(logger, "New goal → x: %.2f y: %.2f theta: %.2f",
                g_goal.x, g_goal.y, g_goal.theta);
}

// ---------------- FSM STATES ----------------

void forward(auto pub) {
    if (!g_new_goal && hasReachedPosition()) {
        commandTurtle(pub, 0, 0);
        g_state = STOP_FORWARD;
    } else {
        g_new_goal = false;
        commandTurtle(pub, 1.5, 0.0);
    }
}

void stopForward(auto pub, auto logger) {
    commandTurtle(pub, 0, 0);

    if (hasStopped()) {
        RCLCPP_INFO(logger, "Position reached. Preparing to turn.");

        g_goal.theta = normalize_angle(g_pose->theta + PI/2.0);
        g_goal.x = g_pose->x;
        g_goal.y = g_pose->y;

        printGoal(logger);
        g_state = TURN;
        g_new_goal = true;
    }
}

void turn(auto pub) {
    double error = normalize_angle(g_goal.theta - g_pose->theta);

    if (!g_new_goal && fabs(error) < 0.01) {
        commandTurtle(pub, 0, 0);
        g_state = STOP_TURN;
    } else {
        g_new_goal = false;
        commandTurtle(pub, 0.0, 2.0 * error);
    }
}

void stopTurn(auto pub, auto logger) {
    commandTurtle(pub, 0, 0);

    if (hasStopped()) {
        RCLCPP_INFO(logger, "Turn completed. Moving forward.");

        g_goal.x = cos(g_pose->theta)*2 + g_pose->x;
        g_goal.y = sin(g_pose->theta)*2 + g_pose->y;
        g_goal.theta = g_pose->theta;

        printGoal(logger);
        g_state = FORWARD;
        g_new_goal = true;
    }
}

// ---------------- MAIN ----------------

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("draw_square");

    auto pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/turtle1/cmd_vel", rclcpp::QoS(1));

    auto sub = node->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", rclcpp::QoS(1), poseCallback);

    auto client = node->create_client<std_srvs::srv::Empty>("reset");

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(16),
        [node, pub]() {

            if (!g_pose) return;

            // First goal
            if (!g_first_goal_set) {
                g_first_goal_set = true;

                g_goal.x = cos(g_pose->theta)*2 + g_pose->x;
                g_goal.y = sin(g_pose->theta)*2 + g_pose->y;
                g_goal.theta = g_pose->theta;

                RCLCPP_INFO(node->get_logger(), "Starting square motion");
                printGoal(node->get_logger());
            }

            switch (g_state) {
                case FORWARD:
                    forward(pub);
                    break;
                case STOP_FORWARD:
                    stopForward(pub, node->get_logger());
                    break;
                case TURN:
                    turn(pub);
                    break;
                case STOP_TURN:
                    stopTurn(pub, node->get_logger());
                    break;
            }
        });

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "Waiting for reset service...");
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    client->async_send_request(request);

    rclcpp::spin(node);
    rclcpp::shutdown();
}