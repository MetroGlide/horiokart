#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <mutex>

class CostmapAdapterNode : public rclcpp::Node
{
public:
  CostmapAdapterNode()
      : Node("costmap_adapter_node")
  {
    this->declare_parameter<std::string>("depth_input_topic", "/depth_costmap/occupancy_grid");
    this->declare_parameter<std::string>("master_input_topic", "");
    this->declare_parameter<std::string>("output_topic", "/depth_costmap/for_nav2");
    this->declare_parameter<bool>("conditional_overwrite", true);
    this->declare_parameter<bool>("overwrite_if_more_lethal", true);

    depth_topic_ = this->get_parameter("depth_input_topic").as_string();
    master_topic_ = this->get_parameter("master_input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    conditional_overwrite_ = this->get_parameter("conditional_overwrite").as_bool();
    overwrite_if_more_lethal_ = this->get_parameter("overwrite_if_more_lethal").as_bool();

    depth_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        depth_topic_, 10, std::bind(&CostmapAdapterNode::onDepthGrid, this, std::placeholders::_1));

    if (!master_topic_.empty())
    {
      master_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          master_topic_, 10, std::bind(&CostmapAdapterNode::onMasterGrid, this, std::placeholders::_1));
    }

    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "CostmapAdapterNode started. depth_topic=%s master_topic=%s output=%s",
                depth_topic_.c_str(), master_topic_.c_str(), output_topic_.c_str());
  }

private:
  void onDepthGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_depth_ = *msg;
    // if no master subscribed, perform merge with empty/master-less logic immediately
    if (!master_sub_)
    {
      auto merged = mergeWithMasterOptional(latest_depth_.value(), nullptr);
      publisher_->publish(merged);
    }
    else if (latest_master_.has_value())
    {
      auto merged = mergeWithMasterOptional(latest_depth_.value(), &latest_master_.value());
      publisher_->publish(merged);
    }
  }

  void onMasterGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_master_ = *msg;
    if (latest_depth_.has_value())
    {
      auto merged = mergeWithMasterOptional(latest_depth_.value(), &latest_master_.value());
      publisher_->publish(merged);
    }
  }

  // Merge incoming depth grid with optional master grid according to parameters.
  // If master is nullptr, the returned grid will be the depth grid (with possible clamping of values).
  nav_msgs::msg::OccupancyGrid mergeWithMasterOptional(const nav_msgs::msg::OccupancyGrid &depth,
                                                       const nav_msgs::msg::OccupancyGrid *master)
  {
    nav_msgs::msg::OccupancyGrid out = depth; // base on depth

    // If master provided and sizes match, perform per-cell conditional merge; otherwise forward depth as-is
    if (master && master->info.width == depth.info.width && master->info.height == depth.info.height && master->info.resolution == depth.info.resolution && master->header.frame_id == depth.header.frame_id)
    {
      size_t N = depth.data.size();
      out.data.resize(N);
      for (size_t i = 0; i < N; ++i)
      {
        int8_t d = depth.data[i];
        int8_t m = master->data[i];
        // depth: -1 unknown, 0..100 occupancy probability
        if (d == -1)
        {
          // no information from depth -> keep master
          out.data[i] = m;
          continue;
        }
        if (!conditional_overwrite_)
        {
          // always overwrite master with depth
          out.data[i] = d;
          continue;
        }
        // conditional overwrite: compare lethality (higher value == more occupied)
        if (m == -1)
        {
          // master unknown -> accept depth
          out.data[i] = d;
        }
        else
        {
          if (overwrite_if_more_lethal_)
          {
            // overwrite only if depth indicates more occupied than master
            if (d > m)
              out.data[i] = d;
            else
              out.data[i] = m;
          }
          else
          {
            // overwrite only if depth indicates less occupied (more free)
            if (d < m)
              out.data[i] = d;
            else
              out.data[i] = m;
          }
        }
      }
    }
    else
    {
      // incompatible master or none: optionally perform simple sanitization on depth (clamp to [-1,100])
      for (size_t i = 0; i < out.data.size(); ++i)
      {
        int v = out.data[i];
        if (v < -1)
          v = -1;
        if (v > 100)
          v = 100;
        out.data[i] = static_cast<int8_t>(v);
      }
    }

    // update header stamp
    out.header.stamp = this->now();
    return out;
  }

  std::string depth_topic_;
  std::string master_topic_;
  std::string output_topic_;
  bool conditional_overwrite_{true};
  bool overwrite_if_more_lethal_{true};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr depth_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr master_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

  std::mutex mutex_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_depth_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_master_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
