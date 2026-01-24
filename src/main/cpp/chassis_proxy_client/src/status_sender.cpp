#include <chassis_proxy_client/proxy_client.hpp>



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<proxy_client::ProxyClientNode> node = std::make_shared<proxy_client::ProxyClientNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
}
