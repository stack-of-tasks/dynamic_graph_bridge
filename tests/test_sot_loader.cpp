#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-copy"
#include <gmock/gmock.h>
#pragma clang diagnostic pop

using namespace std::chrono_literals;

#include <rclcpp/rclcpp.hpp>

#include "dynamic_graph_bridge/ros.hpp"
#include "dynamic_graph_bridge/sot_loader.hh"
#include "dynamic_graph_bridge_msgs/msg/vector.hpp"
#include "dynamic_graph_bridge/ros_python_interpreter_server.hpp"

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
    bool service_done_;
    std::string node_name_;
    std::string response_dg_python_result_;

    MockSotLoader():
        node_name_("unittest_sot_loader") {}

    ~MockSotLoader() {
      service_done_ = false;
      response_dg_python_result_ = "";
    }

    void topic_callback(const dynamic_graph_bridge_msgs::msg::Vector::SharedPtr msg) const {
      bool ok=true;
      for(std::vector<double>::size_type idx=0;idx<msg->data.size();idx++) {
        if (msg->data[idx]!=0.0)
          ok=false;
      }
      ASSERT_TRUE(ok);
    }
    // This method is to listen to publication from the control signal (dg world)
    // to the control_ros topic (ros world).
    void subscribe_to_a_topic() {
      subscription_ = dynamic_graph_bridge::get_ros_node(node_name_)->
          create_subscription<dynamic_graph_bridge_msgs::msg::Vector>(
              "control_ros", 1, std::bind(&MockSotLoader::topic_callback, this,
                                          std::placeholders::_1));
    }

    // This method is to receive the result of client request to run_python_file
    //
    void response_dg_python(
        const rclcpp::Client<dynamic_graph_bridge_msgs::srv::RunPythonFile>::SharedFuture
        future) {
      RCLCPP_INFO(rclcpp::get_logger("dynamic_graph_bridge"), "response_dg_python:");
      auto status = future.wait_for(500ms);
      if (status == std::future_status::ready) {
        // uncomment below line if using Empty() message
        // RCLCPP_INFO(this->get_logger(), "Result: success");
        // comment below line if using Empty() message
        RCLCPP_INFO(rclcpp::get_logger("dynamic_graph_bridge"),
                    "response_dg_python - Result: %s",
                    future.get()->result.c_str());
        service_done_ = true;
        response_dg_python_result_ = future.get()->result;
      } else {
        RCLCPP_INFO(rclcpp::get_logger("dynmic_graph_bridge"),
                    "response_dg_python - Service In-Progress...");
      }
    }

    void start_run_python_script_ros_service(const std::string& file_name) {
      // Service name.
      std::string service_name = "/dynamic_graph_bridge/run_python_file";
      // Create a client from a temporary node.
      auto client = dynamic_graph_bridge::get_ros_node(node_name_)->
          create_client<dynamic_graph_bridge_msgs::srv::RunPythonFile>(
              service_name);
      ASSERT_TRUE(client->wait_for_service(1s));

      // Fill the command message.
      dynamic_graph_bridge_msgs::srv::RunPythonFile::Request::SharedPtr request =
          std::make_shared<
            dynamic_graph_bridge_msgs::srv::RunPythonFile::Request>();
      request->input = file_name;
      // Call the service.
      auto response =
          client->async_send_request(request,
                                     std::bind(&MockSotLoader::response_dg_python,
                                               this,
                                               std::placeholders::_1));
      RCLCPP_INFO(rclcpp::get_logger("dynamic_graph_bridge"),
                  "Send request to service %s - Waiting",
                  service_name.c_str());
      response.wait();
      RCLCPP_INFO(rclcpp::get_logger("dynamic_graph_bridge"),
                  "Get the answer");

    }

    void display_services(
        std::map<std::string, std::vector<std::string> >& list_service_and_types) {
      for (std::map<std::string, std::vector<std::string> >::const_iterator
               const_it = list_service_and_types.begin();
           const_it != list_service_and_types.end(); ++const_it) {
        std::cerr << const_it->first << "\t\t\t";
        for (unsigned i = 0; i < const_it->second.size(); ++i) {
          std::cerr << std::right;
          std::cerr << const_it->second[i] << std::endl;
        }
      }
    }

    void local_ros_spin() {
      RCLCPP_INFO(rclcpp::get_logger("dynamic_graph_bridge"),
                  "local_ros_spin");
      dynamic_graph_bridge::ros_spin();
    }

    void generateEvents() {
      usleep(20000);
      RCLCPP_INFO(rclcpp::get_logger("dynamic_graph_bridge"),
                  "Start Dynamic Graph ");
      dynamic_graph_stopped_ = false;

      std::string file_name =
          TEST_CONFIG_PATH + std::string("simple_ros_publish.py");
      std::string result = "";
      std::string standard_output = "";
      std::string standard_error = "";
      start_run_python_script_ros_service(file_name);


      usleep(100000);
      std::map<std::string, std::vector<std::string>> list_service_and_types;
      dynamic_graph_bridge::RosNodePtr ros_node = dynamic_graph_bridge::get_ros_node(node_name_);
      bool simple_service_exists = false;

      usleep(30720000); // Wait 30s
      RCLCPP_INFO(rclcpp::get_logger("dynamic_graph_bridge"),
                  "Stop Dynamic Graph ");
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

      // Start the thread for ros events.
      std::thread local_ros_spin_thread(&MockSotLoader::local_ros_spin, this);

      // Wait for each threads.
      SotLoader::lthread_join();
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
    dynamic_graph_bridge::ros_spin_non_blocking();
  }

  void TearDown() {
    delete mockSotLoader_ptr_;
    mockSotLoader_ptr_ = nullptr;
    dynamic_graph_bridge::ros_clean();
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
