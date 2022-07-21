/**
 * This is for compile testing only, do not run this.
 */

#include <test_api.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto test_node = rclcpp::Node::make_shared("test_node");

  // publish
  using test_namespace::test_api::SimpleQoS;
  auto pub = test_node->create_publisher<SimpleQoS::MessageType>(SimpleQoS::topic_name(), SimpleQoS::qos());
  SimpleQoS::MessageType msg;
  msg.data = "hello";
  pub->publish(msg);

  // subscribe
  test_node->create_subscription<SimpleQoS::MessageType>(
    SimpleQoS::topic_name(), SimpleQoS::qos(),
    [](const SimpleQoS::MessageType::SharedPtr) {}
  );

  // client
  using test_namespace::test_api::TestService;
  auto client = test_node->create_client<TestService::ServiceType>(TestService::service_name());
  auto req = std::make_shared<TestService::ServiceType::Request>();
  client->async_send_request(req);

  // server
  auto server = test_node->create_service<TestService::ServiceType>(
    TestService::service_name(), [](const TestService::ServiceType::Request::SharedPtr) {});

  rclcpp::spin(test_node);

  return 0;
}
