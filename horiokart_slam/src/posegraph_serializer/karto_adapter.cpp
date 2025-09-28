// KartoAdapter: when built with BUILD_KARTO_ADAPTER=ON this implementation
// uses the Karto SDK to load a .posegraph file and extract nodes/edges.
#include "horiokart_slam/karto_adapter.hpp"
#include <fstream>
#include <sys/stat.h>
#include <unordered_map>

// Karto headers are optional at build-time; guard includes so this file still
// compiles when BUILD_KARTO_ADAPTER is OFF (CMake will compile a stub instead).
#if defined(BUILD_KARTO_ADAPTER) && BUILD_KARTO_ADAPTER
#include <karto_sdk/Mapper.h>
#include <karto_sdk/Karto.h>
#include <karto_sdk/Types.h>
#endif

namespace horiokart_slam
{

    bool KartoAdapter::LoadFromService(const std::string &service_name, const std::string &filename)
    {
        // Prefer ServiceAdapter to call slam_toolbox; not implemented here.
        (void)service_name;
        (void)filename;
        return false;
    }

    bool KartoAdapter::LoadFromFiles(const std::string &posegraph_path, const std::string &dataset_path)
    {
        struct stat st;
        if (stat(posegraph_path.c_str(), &st) != 0)
        {
            return false;
        }

#if defined(BUILD_KARTO_ADAPTER) && BUILD_KARTO_ADAPTER
        try
        {
            karto::Mapper mapper("horiokart_karto_adapter");
            // Karto Mapper exposes LoadFromFile API
            mapper.LoadFromFile(posegraph_path);

            // Ensure poses are corrected (run optimization if needed)
            mapper.CorrectPoses();

            // Extract graph
            karto::MapperGraph *graph = mapper.GetGraph();
            if (!graph)
            {
                posegraph_json_["error"] = "empty_graph";
                return false;
            }

            // Nodes: iterate over MapperSensorManager -> GetAllScans or use GetAllProcessedScans
            auto scans = mapper.GetAllProcessedScans();
            // Build node list
            int node_idx = 0;
            // Map from Karto state id to node_idx in our exported nodes array
            std::unordered_map<int, int> stateid_to_idx;
            for (auto scan : scans)
            {
                if (!scan)
                    continue;
                // Pose2 has GetX(), GetY(), GetHeading()
                json node;
                int state_id = scan->GetStateId();
                node["id"] = node_idx;
                // export original Karto state id so callers can match if needed
                node["state_id"] = state_id;
                // Karto stores sensor data time in seconds (kt_double) on SensorData
                node["timestamp"] = scan->GetTime();
                node["pose"] = {scan->GetCorrectedPose().GetX(), scan->GetCorrectedPose().GetY(), scan->GetCorrectedPose().GetHeading()};
                posegraph_json_["nodes"].push_back(node);
                stateid_to_idx[state_id] = node_idx;
                node_idx++;
            }

            // Edges: iterate through graph->GetEdges()
            const auto &edges = graph->GetEdges();
            for (const auto &e : edges)
            {
                if (!e)
                    continue;
                json edge;
                // Edge stores source and target vertices
                auto src_v = e->GetSource();
                auto tgt_v = e->GetTarget();
                if (!src_v || !tgt_v)
                    continue;
                // Retrieve the underlying LocalizedRangeScan objects
                auto src_scan = src_v->GetObject();
                auto tgt_scan = tgt_v->GetObject();
                if (!src_scan || !tgt_scan)
                    continue;
                // We don't have direct mapping from scan->index here; use state id as best-effort
                int src_id = src_scan->GetStateId();
                int tgt_id = tgt_scan->GetStateId();
                // LinkInfo label holds relative pose and covariance
                karto::EdgeLabel *label = e->GetLabel();
                if (label)
                {
                    karto::LinkInfo *link = dynamic_cast<karto::LinkInfo *>(label);
                    if (link)
                    {
                        auto diff = link->GetPoseDifference();
                        // Map Karto state ids to exported node indices if possible
                        if (stateid_to_idx.find(src_id) != stateid_to_idx.end())
                        {
                            edge["from"] = stateid_to_idx[src_id];
                        }
                        else
                        {
                            edge["from"] = src_id;
                        }
                        if (stateid_to_idx.find(tgt_id) != stateid_to_idx.end())
                        {
                            edge["to"] = stateid_to_idx[tgt_id];
                        }
                        else
                        {
                            edge["to"] = tgt_id;
                        }
                        edge["relative_pose"] = {diff.GetX(), diff.GetY(), diff.GetHeading()};
                        // Covariance -> Matrix3; convert to flat array
                        const karto::Matrix3 &cov = link->GetCovariance();
                        // Matrix3 in this Karto SDK exposes operator()(row,col) for element access
                        edge["covariance"] = {cov(0, 0), cov(0, 1), cov(0, 2), cov(1, 0), cov(1, 1), cov(1, 2), cov(2, 0), cov(2, 1), cov(2, 2)};
                    }
                }
                posegraph_json_["edges"].push_back(edge);
            }

            posegraph_json_["note"] = "exported_with_karto_adapter";
            return true;
        }
        catch (const std::exception &ex)
        {
            posegraph_json_["error"] = std::string("karto_exception: ") + ex.what();
            return false;
        }
#else
        // BUILD_KARTO_ADAPTER not enabled: fallback to stub behavior
        posegraph_json_["posegraph_file"] = posegraph_path;
        posegraph_json_["dataset_file"] = dataset_path;
        posegraph_json_["note"] = "KartoAdapter not built (BUILD_KARTO_ADAPTER=OFF); enable to extract nodes/edges";
        return true;
#endif
    }

    json KartoAdapter::ExportToJson()
    {
        return posegraph_json_;
    }

} // namespace horiokart_slam
