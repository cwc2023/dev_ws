// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()//
  : Node("minimal_subscriber") //节点名为minimal_subscriber
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>( //构造函数使用create_subscription类来执行回调
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());//打印接受到的消息
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;//订阅者声明
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);//初始化
  rclcpp::spin(std::make_shared<MinimalSubscriber>());//创建一个节点MinimalSubscriber，然后进入spin循环
  rclcpp::shutdown();   //关闭节点
  return 0;
  //Since this node has the same dependencies as the publisher node, there’s nothing new to add to package.xml.
}
