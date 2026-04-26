#include <memory>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace horiokart_navigation
{
    class CostmapForBagPlay : public rclcpp::Node
    {
    public:
        CostmapForBagPlay()
            : Node("costmap_for_bag_play")
        {
#ifdef __GNUC__
            RCLCPP_INFO(this->get_logger(), "Compiled with GCC version ");
#else
            RCLCPP_INFO(this->get_logger(), "Compiled with non-GCC compiler");
#endif

            // Declare parameters
            this->declare_parameter<std::string>("costmap_name", "costmap");

            // Get parameters
            this->get_parameter("costmap_name", costmap_name_);

            // Initialize costmap
            costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_name_, std::string{get_namespace()}, costmap_name_);
            costmap_ros_->configure();
            costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
            costmap_ros_->activate();

            RCLCPP_INFO(this->get_logger(), "CostmapForBagPlay has been initialized.");
        }

        ~CostmapForBagPlay()
        {
            costmap_ros_->deactivate();
            costmap_ros_->cleanup();
            costmap_thread_.reset();
        }

    private:
        std::string costmap_name_;

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
    };
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<horiokart_navigation::CostmapForBagPlay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}