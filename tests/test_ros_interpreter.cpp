/**
 * @file test_ros_interpreter.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <iomanip>

#include "dynamic_graph_bridge/ros_python_interpreter_server.hpp"
#include "test_common.hpp"

/***************************
 * SETUP THE TESTING CLASS *
 ***************************/

using namespace std::chrono_literals;
using namespace dynamic_graph_bridge;

/**
 * @brief This is the test environment
 */
class TestRosInterpreter : public ::testing::Test {
 protected:
  /**
   * @brief SetUp, is executed before the unit tests
   */
  void SetUp() {
    // Create the ros node
    node_name_ = "unittest_ros_python_interpreter";
    run_python_command_service_name_ =
        "/dynamic_graph_bridge/run_python_command";
    run_python_file_service_name_ = "/dynamic_graph_bridge/run_python_file";
    ros_spin_non_blocking();
  }

  /**
   * @brief TearDown, is executed after the unit tests
   */
  void TearDown() {
    // delete the ros node
    ros_clean();
  }
  /**
   * @brief Node name
   */
  std::string node_name_;

  /**
   * @brief Name of the ros service.
   */
  std::string run_python_command_service_name_;

  /**
   * @brief Name of the ros service.
   */
  std::string run_python_file_service_name_;
};

/*****************
 * Start Testing *
 *****************/

TEST_F(TestRosInterpreter, test_constructor_no_throw) {
  /* setup */

  /* test */
  ASSERT_NO_THROW(RosPythonInterpreterServer rpi;);
}

TEST_F(TestRosInterpreter, test_destructor_no_throw) {
  ASSERT_NO_THROW({ RosPythonInterpreterServer rpi; });
}

TEST_F(TestRosInterpreter, test_run_cmd_not_available_upon_construction) {
  /* Setup. */

  // Init the class to test.
  RosPythonInterpreterServer rpi;

  // Fetch the information using ROS.
  std::map<std::string, std::vector<std::string>> list_service_and_types;
  RosNodePtr ros_node = get_ros_node(node_name_);
  bool simple_service_exists = false;

  // Check if the service exists.
  list_service_and_types = ros_node->get_service_names_and_types();
  display_services(list_service_and_types);
  simple_service_exists =
      list_service_and_types.find(run_python_command_service_name_) !=
      list_service_and_types.end();

  /* Test. */
  ASSERT_FALSE(simple_service_exists);
}

TEST_F(TestRosInterpreter, test_run_file_not_available_upon_construction) {
  /* Setup. */

  // Init the class to test.
  RosPythonInterpreterServer rpi;

  // Fetch the information using ROS.
  std::map<std::string, std::vector<std::string>> list_service_and_types;
  RosNodePtr ros_node = get_ros_node(node_name_);
  bool simple_service_exists = false;

  // Check if the service exists.
  list_service_and_types = ros_node->get_service_names_and_types();
  display_services(list_service_and_types);
  simple_service_exists =
      list_service_and_types.find(run_python_file_service_name_) !=
      list_service_and_types.end();

  /* Test. */
  ASSERT_FALSE(simple_service_exists);
}

TEST_F(TestRosInterpreter, test_run_cmd_available_after_init) {
  /* Setup. */

  // Init the class to test.
  RosPythonInterpreterServer rpi;
  rpi.start_ros_service();

  // print the service available
  RosNodePtr ros_node = get_ros_node(node_name_);
  std::map<std::string, std::vector<std::string>> list_service_and_types =
      ros_node->get_service_names_and_types();
  display_services(list_service_and_types);

  // Fetch the information using ROS.
  auto client =
      get_ros_node(node_name_)
          ->create_client<dynamic_graph_bridge_msgs::srv::RunPythonCommand>(
              "/dynamic_graph_bridge/run_python_command");

  /* Test. */
  ASSERT_TRUE(client->wait_for_service(1s));
}

TEST_F(TestRosInterpreter, test_run_file_available_after_init) {
  /* Setup. */

  // Init the class to test.
  RosPythonInterpreterServer rpi;
  rpi.start_ros_service();

  // Fetch the information using ROS.
  auto client =
      get_ros_node(node_name_)
          ->create_client<dynamic_graph_bridge_msgs::srv::RunPythonFile>(
              "/dynamic_graph_bridge/run_python_file");

  /* Test. */
  ASSERT_TRUE(client->wait_for_service(1s));
}

TEST_F(TestRosInterpreter, test_call_run_command_result) {
  /* Setup. */

  // Create the ros python interpreter.
  RosPythonInterpreterServer rpi;
  rpi.start_ros_service();

  // Create and call the clients.
  std::string cmd = "1 + 1";
  std::string result = "";
  std::string standarderror = "";
  std::string standardoutput = "";
  start_run_python_command_ros_service(cmd, result, standarderror,
                                       standardoutput);

  /* Tests. */
  ASSERT_EQ(result, "2");
}

TEST_F(TestRosInterpreter, test_call_run_command_standardoutput) {
  /* Setup. */

  // Create the ros python interpreter.
  RosPythonInterpreterServer rpi;
  rpi.start_ros_service();

  // Create and call the clients.
  std::string cmd = "print(\"Banana\")";
  std::string result = "";
  std::string standarderror = "";
  std::string standardoutput = "";
  start_run_python_command_ros_service(cmd, result, standarderror,
                                       standardoutput);

  /* Tests. */
  ASSERT_EQ(standardoutput, "Banana\nprint(\"Banana\")\nOutput:Banana\n\n");
}

TEST_F(TestRosInterpreter, test_call_run_command_standarderror) {
  /* Setup. */

  // Create the ros python interpreter.
  RosPythonInterpreterServer rpi;
  rpi.start_ros_service();

  // Create and call the clients.
  std::string cmd = "a";
  std::string result = "";
  std::string standarderror = "";
  std::string standardoutput = "";
  start_run_python_command_ros_service(cmd, result, standarderror,
                                       standardoutput);

  /* Tests. */
  ASSERT_EQ(standarderror,
            "Traceback (most recent call last):\n  File \"<string>\", line 1, "
            "in <module>\nNameError: name 'a' is not defined\n");
}

TEST_F(TestRosInterpreter, test_call_run_script_result) {
  /* Setup. */

  // Create the ros python interpreter.
  RosPythonInterpreterServer rpi;
  rpi.start_ros_service();

  // Create and call the clients.
  std::string file_name = TEST_CONFIG_PATH + std::string("simple_add.py");
  std::string result = "";
  start_run_python_script_ros_service(file_name, result);

  /* Tests. */
  ASSERT_EQ(result, "");
}

TEST_F(TestRosInterpreter, test_call_run_script_standarderror) {
  /* Setup. */

  // Create the ros python interpreter.
  RosPythonInterpreterServer rpi;
  rpi.start_ros_service();

  // Create and call the clients.
  std::string file_name = TEST_CONFIG_PATH + std::string("simple_add_fail");
  std::string result = "";
  start_run_python_script_ros_service(file_name, result);

  // Prepare the test.
  std::string error_first_part = "a = 1 + 1 + b";
  std::string error_second_part = "NameError: name \'b\' is not defined";
  std::size_t found_first_part = result.find(error_first_part);
  std::size_t found_second_part = result.find(error_second_part);

  /* test */
  ASSERT_TRUE(found_first_part != std::string::npos);
  ASSERT_TRUE(found_second_part != std::string::npos);
}
