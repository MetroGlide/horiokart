// ...existing code...
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include "horiokart_slam/service_adapter.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"

using json = nlohmann::json;

namespace horiokart_slam
{

    void ServiceAdapter::SetServiceName(const std::string &service_name)
    {
        service_name_ = service_name;
    }

    std::string ServiceAdapter::GetServiceName() const
    {
        return service_name_;
    }

    ServiceAdapter::ServiceAdapter(const std::string &service_name)
        : service_name_(service_name)
    {
    }

    bool ServiceAdapter::LoadFromService(const std::string &service_name, const std::string &filename)
    {
        // Use rclcpp::Client to call SerializePoseGraph service
        using Serialize = slam_toolbox::srv::SerializePoseGraph;

        auto node = rclcpp::Node::make_shared("posegraph_service_adapter_client");
        auto client = node->create_client<Serialize>(service_name);

        // wait for service up to 5 seconds
        if (!client->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node->get_logger(), "Service %s not available after 5s", service_name.c_str());
            return false;
        }

        auto request = std::make_shared<Serialize::Request>();
        request->filename = filename;

        auto result_future = client->async_send_request(request);

        // wait for result up to 10 seconds
        auto status = rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(10));
        if (status != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Service call to %s failed or timed out", service_name.c_str());
            return false;
        }

        auto response = result_future.get();
        // slam_toolbox SerializePoseGraph may return success flag or file path; PoC assumes success
        (void)response;
        return true;
    }

    bool ServiceAdapter::LoadFromFiles(const std::string &posegraph_path, const std::string &dataset_path)
    {
        // PoC: do minimal verification by checking file existence
        std::ifstream ifs(posegraph_path);
        if (!ifs.good())
            return false;
        posegraph_json_["posegraph_file"] = posegraph_path;
        posegraph_json_["dataset_file"] = dataset_path;
        return true;
    }

    json ServiceAdapter::ExportToJson()
    {
        // return the minimal placeholder JSON
        return posegraph_json_;
    }

} // namespace horiokart_slam

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("posegraph_service_adapter");
    RCLCPP_INFO(node->get_logger(), "posegraph_service_adapter started (PoC)");

    // allow overriding service name via env or arg in future; use default configured name
    std::string filename = "posegraph_dump";
    horiokart_slam::ServiceAdapter adapter; // default service name set in ctor
    std::string svc = adapter.GetServiceName();
    RCLCPP_INFO(node->get_logger(), "Calling service: %s", svc.c_str());
    bool ok = adapter.LoadFromService(svc, filename);
    if (!ok)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call serialize service");
        return 1;
    }

    // for now, assume files are created in cwd
    adapter.LoadFromFiles(filename + ".posegraph", filename + ".data");
    auto out = adapter.ExportToJson();

    std::ofstream ofs("/tmp/posegraph_dump.json");
    ofs << out.dump(2);
    ofs.close();

    RCLCPP_INFO(node->get_logger(), "Wrote /tmp/posegraph_dump.json");
    rclcpp::shutdown();
    return 0;
}
