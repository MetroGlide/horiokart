#pragma once
#include "horiokart_slam/posegraph_adapter.hpp"
#include <string>

namespace horiokart_slam
{

    // KartoAdapter: when fully implemented this will link against Karto
    // and extract nodes/edges directly from karto::Mapper / karto::Dataset.
    // Current stub implementation is safe to build without Karto and
    // returns a minimal JSON summary when given .posegraph/.data files.
    class KartoAdapter : public IPoseGraphAdapter
    {
    public:
        KartoAdapter() = default;
        ~KartoAdapter() override = default;

        bool LoadFromService(const std::string &service_name, const std::string &filename) override;
        bool LoadFromFiles(const std::string &posegraph_path, const std::string &dataset_path) override;
        json ExportToJson() override;

    private:
        json posegraph_json_;
    };

} // namespace horiokart_slam
