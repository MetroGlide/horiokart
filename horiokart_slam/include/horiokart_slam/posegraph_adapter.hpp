#pragma once
#include <string>
#include <nlohmann/json.hpp>

namespace horiokart_slam
{

    using json = nlohmann::json;

    class IPoseGraphAdapter
    {
    public:
        virtual ~IPoseGraphAdapter() = default;

        // Populate internal state from service (e.g., calling slam_toolbox SerializePoseGraph)
        virtual bool LoadFromService(const std::string &service_name, const std::string &filename) = 0;

        // Populate internal state by reading files (e.g., .posegraph/.data)
        virtual bool LoadFromFiles(const std::string &posegraph_path, const std::string &dataset_path) = 0;

        // Export the posegraph into the agreed JSON schema
        virtual json ExportToJson() = 0;
    };

} // namespace horiokart_slam
