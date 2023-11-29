#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-copy"
#include <gmock/gmock.h>
#pragma clang diagnostic pop

#include "dynamic_graph_bridge/ros.hpp"
#include "dynamic_graph_bridge/sot_loader.hh"
#include "dynamic_graph_bridge_msgs/msg/vector.hpp"
#include "test_common.hpp"

namespace test_sot_loader {
int l_argc;
char** l_argv;
}

namespace dg = dynamicgraph;

class MockSotLoaderTest : public ::testing::Test {
 public:
  class MockSotLoader : public SotLoader {
   public:
    rclcpp::Subscription<dynamic_graph_bridge_msgs::msg::Vector>::SharedPtr
    subscription_;

    ~MockSotLoader() {}

    void topic_callback(const dynamic_graph_bridge_msgs::msg::Vector::SharedPtr msg) const {
      auto lsize=msg->data.size();
    }

    void subscribe_to_a_topic() {
      subscription_ = create_subscription<dynamic_graph_bridge_msgs::msg::Vector>(
          "control_ros", 1, std::bind(&MockSotLoader::topic_callback, this,
                                      std::placeholders::_1));
    }

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
      char argv2[60] = "--input-file=libimpl_test_library.so";
      argv[1] = argv2;
      parseOptions(argc, argv);

      std::string finalname("libimpl_test_library.so");
      EXPECT_TRUE(finalname == dynamicLibraryName_);

      readSotVectorStateParam();
      // Performs initialization of libimpl_test_library.so
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

      // Create and call the clients.
      std::string file_name =
          TEST_CONFIG_PATH + std::string("simple_ros_publish.py");
      std::string result = "";
      std::string standard_output = "";
      std::string standard_error = "";
      //start_run_python_script_ros_service(file_name, result);

      // Wait for each threads.
      SotLoader::lthread_join();  // Wait 100 ms
      local_events.join();
    }
  };

 public:
  MockSotLoader *mockSotLoader_ptr_;

  // For the set of tests coded in this file.
  static void SetUpTestCase() {

    rclcpp::init(test_sot_loader::l_argc,
                 test_sot_loader::l_argv);
  }

  // For each test specified in this file
  static void TearDownTestCase() {
    rclcpp::shutdown();
  }

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

  test_sot_loader::l_argc = argc;
  test_sot_loader::l_argv = argv;


  int r = RUN_ALL_TESTS();

  return r;
}
