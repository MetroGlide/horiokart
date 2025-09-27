#pragma once
#include "horiokart_slam/posegraph_adapter.hpp"
#include <string>

namespace horiokart_slam
{

    class ServiceAdapter : public IPoseGraphAdapter
    {
    public:
        ServiceAdapter() = default;
        ~ServiceAdapter() override = default;

        bool LoadFromService(const std::string &service_name, const std::string &filename) override;
        bool LoadFromFiles(const std::string &posegraph_path, const std::string &dataset_path) override;
        json ExportToJson() override;

    private:
        // internal placeholders for parsed data
        json posegraph_json_;
    };

} // namespace horiokart_slam
