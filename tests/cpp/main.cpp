/**
 * This is for compile testing only, do not run this.
 */

#include <test_api.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto test_node = rclcpp::Node::make_shared("test_node");

  // publish
  using test_namespace::test_api::SimpleQoS;
  auto pub = test_node->create_publisher<SimpleQoS::MessageType>(
    SimpleQoS::topic_name(), SimpleQoS::qos());
  SimpleQoS::MessageType msg;
  msg.data = "hello";
  pub->publish(msg);

  // subscribe
  test_node->create_subscription<SimpleQoS::MessageType>(
    SimpleQoS::topic_name(), SimpleQoS::qos(),
    [](SimpleQoS::MessageType::SharedPtr) {}
  );

  // client
  using test_namespace::test_api::TestService;
  auto client = test_node->create_client<TestService::ServiceType>(TestService::service_name());
  auto req = std::make_shared<TestService::ServiceType::Request>();
  client->async_send_request(req);

  // server
  auto server = test_node->create_service<TestService::ServiceType>(
    TestService::service_name(),
    [](std::shared_ptr<rmw_request_id_t>, TestService::ServiceType::Request::SharedPtr,
    TestService::ServiceType::Response::SharedPtr) {});

  // action client
  using test_namespace::test_api::TestAction;
  auto action_client = rclcpp_action::create_client<TestAction::ActionType>(
    test_node, TestAction::action_name());
  TestAction::ActionType::Goal action_req;
  action_client->async_send_goal(action_req);

  // action server
  auto action_server = rclcpp_action::create_server<TestAction::ActionType>(
    test_node, TestAction::action_name(),
    [](const rclcpp_action::GoalUUID &, TestAction::ActionType::Goal::ConstSharedPtr) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<TestAction::ActionType>>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<TestAction::ActionType>>) {}
  );

  rclcpp::spin(test_node);

  return 0;
}
