#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-copy"
#include <gmock/gmock.h>
#pragma clang diagnostic pop

#include "dynamic_graph_bridge/ros.hpp"
#include "dynamic_graph_bridge/sot_loader_basic.hh"

namespace test_sot_loader_basic {
int l_argc;
char **l_argv;
}  // namespace test_sot_loader_basic

class MockSotLoaderBasicTest : public ::testing::Test {
 public:
  class MockSotLoaderBasic : public SotLoaderBasic {
   public:
    MockSotLoaderBasic(
        const std::string &aNodeName = std::string("MockSotLoaderBasic"))
        : SotLoaderBasic(aNodeName) {}
    virtual ~MockSotLoaderBasic() {}

    void UniqueTest() {
      // This test makes sure that the class test_sot_loader_basic is able to
      // read parameters from the ROS-2 space.
      // It is suppose to read :
      // state_vector_map and populate member stateVectorMap_
      // in this test we should have two joints: [ "joint1", "joint2"]
      readSotVectorStateParam();
      ASSERT_EQ(static_cast<int>(stateVectorMap_.size()), 2);
      ASSERT_EQ(nbOfJoints_, 2);
      std::string aJoint1("joint1");
      EXPECT_TRUE(aJoint1 == stateVectorMap_[0]);
      std::string aJoint2("joint2");
      EXPECT_TRUE(aJoint2 == stateVectorMap_[1]);

      // Void call
      int argc = 1;
      char argv1[30] = "mocktest";
      char *argv[3];
      argv[0] = argv1;
      // Checking that parseOptions returns -1
      EXPECT_EQ(parseOptions(argc, argv), -1);

      // Test help call
      argc = 2;
      char argv2[30] = "--help";
      argv[1] = argv2;
      // Checking that parseOptions returns -1
      EXPECT_EQ(parseOptions(argc, argv), -1);

      // Test input file
      char argv3[60] = "--input-file=libimpl_test_library.so";
      argv[1] = argv3;
      EXPECT_EQ(parseOptions(argc, argv), 0);

      // Check that the file is what we specified
      std::string finalname("libimpl_test_library.so");
      EXPECT_TRUE(finalname == dynamicLibraryName_);

      initPublication();
      EXPECT_TRUE(joint_pub_ != 0);
      initializeServices();
      EXPECT_TRUE(service_start_ != 0);
      EXPECT_TRUE(service_stop_ != 0);

      std::shared_ptr<std_srvs::srv::Empty::Request> empty_rqst;
      std::shared_ptr<std_srvs::srv::Empty::Response> empty_resp;
      start_dg(empty_rqst, empty_resp);
      EXPECT_FALSE(dynamic_graph_stopped_);

      stop_dg(empty_rqst, empty_resp);
      EXPECT_TRUE(dynamic_graph_stopped_);

      // Remove
      CleanUp();
    }
  };

 public:
  std::shared_ptr<MockSotLoaderBasic> mockSotLoaderBasic_ptr_;
  unsigned int nb_tests_;

  MockSotLoaderBasicTest() { nb_tests_ = 0; }

  // For the set of tests coded in this file.
  static void SetUpTestCase() {
    rclcpp::init(test_sot_loader_basic::l_argc, test_sot_loader_basic::l_argv);
  }

  // For each test specified in this file
  static void TearDownTestCase() { rclcpp::shutdown(); }

  // For each test specified by TEST_F
  void SetUp() {
    std::string aSotLoaderBasicName("MockSotLoaderBasic" +
                                    std::to_string(nb_tests_));
    mockSotLoaderBasic_ptr_ =
        std::make_shared<MockSotLoaderBasic>(aSotLoaderBasicName);
    mockSotLoaderBasic_ptr_->initialize();
  }

  // For each test specified by TEST_F
  void TearDown() { nb_tests_++; }
};

TEST_F(MockSotLoaderBasicTest, UniqueTest) {
  mockSotLoaderBasic_ptr_->UniqueTest();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  test_sot_loader_basic::l_argc = argc;
  test_sot_loader_basic::l_argv = argv;

  int r = RUN_ALL_TESTS();

  return r;
}
