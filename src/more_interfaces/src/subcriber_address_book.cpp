#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

class AddressBookSubscriber : public rclcpp::Node
{
public:
  AddressBookSubscriber()
  : Node("address_book_subscriber")
  {
    address_book_subscriber_ = this->create_subscription<more_interfaces::msg::AddressBook>(
      "address_book", 10, std::bind(&AddressBookSubscriber::topic_callback, this, std::placeholders::_1));
  }
/*
create_subscription是ROS2中的一个函数，它可以创建一个订阅者对象。
在这个例子中，this->create_subscription<more_interfaces::msg::AddressBook>("address_book", 10,
创建了一个订阅者对象，它订阅了名为"address_book"的主题，并且每次接收到消息时，都会调用一个回调函数。
在ROS2中，create_subscription函数的第二个参数是订阅者队列的大小。这个参数指定了订阅者队列中可以缓存
的消息数量。如果订阅者不能及时处理消息，那么这些消息将被缓存到队列中，直到队列被填满。在这个例子中，10指定了订阅者队列的大小为10
*/




/*
std::bind是一个函数模板，它可以将一个函数对象和一些参数绑定在一起，返回一个新的函数对象。这个新的函数对象可以像原来的函数对象一样调用，
但是它的参数已经被绑定了。在这个例子中，std::bind(&AddressBookSubscriber::topic_callback, this, std::placeholders::_1)
将AddressBookSubscriber::topic_callback函数和this指针绑定在一起，并将第一个参数绑定到占位符std::placeholders::_1上。
当新的函数对象被调用时，它将调用AddressBookSubscriber::topic_callback(this, arg1)
*/
private:
  void topic_callback(const more_interfaces::msg::AddressBook::SharedPtr msg) const
  {
    std::cout << "I heard:\nFirst:" << msg->first_name <<
      "  Last:" << msg->last_name << "\nPhone:" << msg->phone_number <<
      "\nPhone Type:" << msg->phone_type << std::endl;
  }

  rclcpp::Subscription<more_interfaces::msg::AddressBook>::SharedPtr address_book_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookSubscriber>());
  rclcpp::shutdown();

  return 0;
}

