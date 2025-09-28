#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include "horiokart_slam/posegraph_adapter.hpp"
#include "horiokart_slam/karto_adapter.hpp"

int main(int argc, char **argv)
{
    // Usage: karto_adapter_test [output_base_dir]
    std::string output_base = "/root/ros2_data/slam_modify";
    if (argc > 1)
    {
        output_base = argv[1];
    }

    horiokart_slam::KartoAdapter adapter;
    std::string posegraph = "/root/ros2_ws/posegraph_dump.posegraph";
    std::string datafile = "/root/ros2_ws/posegraph_dump.data";
    try
    {
        bool ok = adapter.LoadFromFiles(posegraph, datafile);
        if (!ok)
        {
            std::cerr << "KartoAdapter failed to load files\n";
            auto j = adapter.ExportToJson();
            std::cerr << "Adapter error info: " << j.dump(2) << std::endl;
            return 3;
        }
        auto j = adapter.ExportToJson();

        // Create timestamped subdirectory under output_base
        namespace fs = std::filesystem;
        try
        {
            fs::path base(output_base);
            if (!fs::exists(base))
            {
                fs::create_directories(base);
            }
            // timestamp: YYYYmmdd_HHMMSS
            auto now = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(now);
            std::tm tm = *std::localtime(&t);
            std::ostringstream ss;
            ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
            fs::path outdir = base / ss.str();
            fs::create_directories(outdir);

            fs::path outfile = outdir / "posegraph_from_karto.json";
            std::ofstream out(outfile);
            out << j.dump(2);
            out.close();
            std::cout << "Wrote " << outfile.string() << "\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << "Filesystem error: " << e.what() << std::endl;
            return 4;
        }
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 2;
    }
}
