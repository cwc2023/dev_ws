cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)


set(msg_files
  "msg/AddressBook.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)



add_executable(publish_address_book    src/publish_address_book.cpp)
add_executable(subcriber_address_book  src/subcriber_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp)
ament_target_dependencies(subcriber_address_book rclcpp)

install(TARGETS
    publish_address_book
    subcriber_address_book
  DESTINATION lib/${PROJECT_NAME})


rosidl_get_typesupport_target(cpp_typesupport_target  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(publish_address_book "${cpp_typesupport_target}")
target_link_libraries(subcriber_address_book "${cpp_typesupport_target}")


ament_package()
