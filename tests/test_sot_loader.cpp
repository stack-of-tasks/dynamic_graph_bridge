
#include <gmock/gmock.h>

#include "dynamic_graph_bridge/ros.hpp"
#include "dynamic_graph_bridge/sot_loader.hh"

class MockSotLoaderTest : public ::testing::Test {
 public:
  class MockSotLoader : public SotLoader {
   public:
    ~MockSotLoader() {}

    void generateEvents() {
      usleep(20000);
      std::cerr << "Start Dynamic Graph " << std::endl;
      dynamic_graph_stopped_ = false;

      usleep(20000);
      std::cerr << "Stop Dynamic Graph " << std::endl;
      dynamic_graph_stopped_ = true;
    }

    void testLoadController() {
      // Set input  call
      int argc = 2;
      char *argv[2];
      char argv1[30] = "mocktest";
      argv[0] = argv1;
      char argv2[60] = "--input-file=libimpl_test_sot_external_interface.so";
      argv[1] = argv2;
      parseOptions(argc, argv);

      std::string finalname("libimpl_test_sot_external_interface.so");
      EXPECT_TRUE(finalname == dynamicLibraryName_);

      // Performs initialization of libimpl_test_sot_external_interface.so
      loadController();
      EXPECT_TRUE(sotRobotControllerLibrary_ != 0);
      EXPECT_TRUE(sotController_ != nullptr);
      // initialize start/stop and runCommand services
      initializeServices();

      // Constructor should default freeFlyerPose
      EXPECT_TRUE(freeFlyerPose_.header.frame_id == std::string("odom"));
      EXPECT_TRUE(freeFlyerPose_.child_frame_id == std::string("base_link"));

      // Initialie publication of sot joint states
      // and base eff
      initPublication();

      // Start the control loop thread.
      startControlLoop();

      // Start the thread generating events.
      std::thread local_events(&MockSotLoader::generateEvents, this);

      // Wait for each threads.
      SotLoader::lthread_join();  // Wait 100 ms
      local_events.join();
    }
  };

 public:
  MockSotLoader *mockSotLoader_ptr_;

  void SetUp() {
    mockSotLoader_ptr_ = new MockSotLoader();
    mockSotLoader_ptr_->initialize();
  }

  void TearDown() {
    delete mockSotLoader_ptr_;
    mockSotLoader_ptr_ = nullptr;
  }
};

TEST_F(MockSotLoaderTest, TestLoadController) {
  mockSotLoader_ptr_->testLoadController();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int r = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return r;
}
