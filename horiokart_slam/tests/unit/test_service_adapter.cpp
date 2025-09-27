// Basic unit test for ServiceAdapter LoadFromFiles and ExportToJson
#include <gtest/gtest.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include "horiokart_slam/service_adapter.hpp"

using namespace horiokart_slam;

TEST(ServiceAdapterTest, LoadFromFilesWritesJson)
{
    // create temporary files
    const std::string pg = "/tmp/test_posegraph.posegraph";
    const std::string ds = "/tmp/test_posegraph.data";
    std::ofstream(pg) << "dummy";
    std::ofstream(ds) << "dummy";

    ServiceAdapter adapter;
    bool ok = adapter.LoadFromFiles(pg, ds);
    EXPECT_TRUE(ok);

    auto j = adapter.ExportToJson();
    EXPECT_TRUE(j.contains("posegraph_file"));
    EXPECT_EQ(j["posegraph_file"], pg);
    EXPECT_EQ(j["dataset_file"], ds);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
