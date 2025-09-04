/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <rclcpp/rclcpp.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/contacts.pb.h>
#include <champ_msgs/msg/contacts_stamped.hpp>
#include <champ/utils/urdf_loader.h>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <string>

class ContactSensor : public rclcpp::Node
{
public:
    ContactSensor()
        : Node("contacts_sensor", rclcpp::NodeOptions()
                                      .allow_undeclared_parameters(true)
                                      .automatically_declare_parameters_from_overrides(true)),
          foot_contacts_{false, false, false, false}
    {
        // Initialize foot_links_ with joint names
        auto joint_names = champ::URDF::getLinkNames(this->get_node_parameters_interface());
        foot_links_.push_back(joint_names[2]);
        foot_links_.push_back(joint_names[6]);
        foot_links_.push_back(joint_names[10]);
        foot_links_.push_back(joint_names[14]);

        // Initialize the publisher
        contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);

        // Set up the Ignition Transport subscription
        if (!ign_node_.Subscribe("/world/default/physics/contacts",
                                 &ContactSensor::ignitionCallback, this))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to /world/default/physics/contacts");
        }
    }

    void publishContacts()
    {
        champ_msgs::msg::ContactsStamped contacts_msg;
        contacts_msg.header.stamp = this->get_clock()->now();
        contacts_msg.contacts.resize(4);

        for (size_t i = 0; i < 4; ++i)
        {
            contacts_msg.contacts[i] = foot_contacts_[i];
        }

        contacts_publisher_->publish(contacts_msg);
    }

private:
    void ignitionCallback(const ignition::msgs::Contacts &msg)
    {
        std::fill(std::begin(foot_contacts_), std::end(foot_contacts_), false);

        for (int i = 0; i < msg.contact_size(); ++i)
        {
            std::vector<std::string> results;
            std::string collision = msg.contact(i).collision1().name();
            boost::split(results, collision, [](char c) { return c == ':'; });

            for (size_t j = 0; j < foot_links_.size(); ++j)
            {
                if (foot_links_[j] == results[2])
                {
                    foot_contacts_[j] = true;
                    break;
                }
            }
        }
    }

    bool foot_contacts_[4];
    std::vector<std::string> foot_links_;
    rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_publisher_;
    ignition::transport::Node ign_node_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContactSensor>();
    rclcpp::Rate loop_rate(50);

    while (rclcpp::ok())
    {
        node->publishContacts();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
