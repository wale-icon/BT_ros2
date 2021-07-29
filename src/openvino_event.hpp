#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <unistd.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <object_msgs/msg/objects_in_boxes.hpp>

static std::vector<std::string> detected_objects;

void OpenVINOCallback(const object_msgs::msg::ObjectsInBoxes::SharedPtr msg)
{
    int cnt = 0;
    for(auto & obj : msg->objects_vector)
    {
        std::cout << "["<< ++cnt <<"] Detect object: '" << obj.object.object_name << "'." << std::endl;
        detected_objects.push_back(std::string(obj.object.object_name));
    }
}

class OpenVINOEvent : public BT::SyncActionNode
{
    public:
        OpenVINOEvent(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("openvino_event");
            sub_ = node_->create_subscription<object_msgs::msg::ObjectsInBoxes>("/ros2_openvino_toolkit/detected_objects", 1000, OpenVINOCallback);
            sleep(1);
            time_init = node_->get_clock()->now();
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<std::string>("object") };
        }

        virtual BT::NodeStatus tick() override
        {
            std::string expect_object;

            if (!getInput<std::string>("object", expect_object)) {
                throw BT::RuntimeError("missing required input [object]");
            }

            auto duration = node_->get_clock()->now();

            while (duration.seconds() - time_init.seconds() < 1.) {
              rclcpp::spin_some(node_);
              duration = node_->get_clock()->now();
            }

            time_init = duration;
            int cnt = std::count(detected_objects.begin(), detected_objects.end(), expect_object);
            detected_objects.clear();

            // The number of detected objects should be greater than 10 to avoid misbehavior
            if (cnt > 10) {
                fprintf(stderr, "Object['%s'] count=%d\n", expect_object.c_str(), cnt);
                return BT::NodeStatus::SUCCESS;
            }
            else {
                return BT::NodeStatus::FAILURE;
            }

          }

    private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Subscription<object_msgs::msg::ObjectsInBoxes>::SharedPtr sub_;
      rclcpp::Time time_init;
};
