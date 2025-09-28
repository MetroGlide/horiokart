#pragma once
#include "horiokart_slam/posegraph_adapter.hpp"
#include <string>

namespace horiokart_slam
{

    class ServiceAdapter : public IPoseGraphAdapter
    {
    public:
        // service_name defaults to the canonical slam_toolbox serialization service
        explicit ServiceAdapter(const std::string &service_name = "/slam_toolbox/serialize_map");
        ~ServiceAdapter() override = default;

        bool LoadFromService(const std::string &service_name, const std::string &filename) override;
        bool LoadFromFiles(const std::string &posegraph_path, const std::string &dataset_path) override;
        json ExportToJson() override;

        // convenience accessors
        void SetServiceName(const std::string &service_name);
        std::string GetServiceName() const;

    private:
        // internal placeholders for parsed data
        json posegraph_json_;
        std::string service_name_;
    };

} // namespace horiokart_slam
