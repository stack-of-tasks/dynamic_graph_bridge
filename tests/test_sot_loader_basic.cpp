
#include <gmock/gmock.h>
#include "dynamic_graph_bridge/ros2_init.hh"
#include "dynamic_graph_bridge/sot_loader_basic.hh"

int my_argc;
char ** my_argv;

class MockSotLoaderBasicTest: public ::testing::Test {

public:

  class MockSotLoaderBasic: public SotLoaderBasic {
  public:


    void checkStateVectorMap() {
      readSotVectorStateParam();
      ASSERT_EQ(static_cast<int>(stateVectorMap_.size()),2);
      std::string aJoint1("joint1");
      EXPECT_TRUE(aJoint1 == stateVectorMap_[0]);
      std::string aJoint2("joint2");
      EXPECT_TRUE(aJoint2 == stateVectorMap_[1]);
    }

    void testParseOptions() {

      // Void call
      int argc=1;
      char argv1[30]="mocktest";
      char *argv[3];
      argv[0]=argv1;
      EXPECT_EQ(parseOptions(argc,argv),-1);

      // Test help call
      argc=2;
      char argv2[30]="--help";
      argv[1]=argv2;
      EXPECT_EQ(parseOptions(argc,argv),-1);

      // Test input file
      char argv3[60]="--input-file=libimpl_test_sot_external_interface.so";
      argv[1]=argv3;
      EXPECT_EQ(parseOptions(argc,argv),0);

      // Check that the file is what we specified
      std::string finalname("libimpl_test_sot_external_interface.so");
      EXPECT_TRUE(finalname == dynamicLibraryName_);
    }

    void testInitializeRosNode() {
      int argc=1;
      char *argv[1];
      char argv1[30]="mocktest";
      argv[0] = argv1;
      parseOptions(argc,argv);
      initializeServices();
      EXPECT_TRUE(service_start_ != 0);
      EXPECT_TRUE(service_stop_ != 0);
    }

    void testInitialization() {

      // Set input arguments
      int argc=2;
      char *argv[2];

      char argv1[30]="mocktest";
      argv[0]=argv1;
      char argv2[60]="--input-file=libimpl_test_sot_external_interface.so";
      argv[1]=argv2;
      parseOptions(argc,argv);

      std::string finalname("libimpl_test_sot_external_interface.so");
      EXPECT_TRUE(finalname == dynamicLibraryName_);

      // Performs initializatio of libimpl_test_sot_external_interface.so
      loadController();
      EXPECT_TRUE(sotRobotControllerLibrary_ != 0);
      EXPECT_TRUE(sotController_ != nullptr);
    }

    void test_start_stop_dg() {
      std::shared_ptr<std_srvs::srv::Empty::Request> empty_rqst;
      std::shared_ptr<std_srvs::srv::Empty::Response> empty_resp;
      start_dg(empty_rqst,empty_resp);
      EXPECT_FALSE(dynamic_graph_stopped_);

      stop_dg(empty_rqst,empty_resp);
      EXPECT_TRUE(dynamic_graph_stopped_);
    }

    void test_cleanup() {
      CleanUp();
      EXPECT_TRUE(sotController_ == NULL);
      std::cout << "sotController_: " << sotController_
                << std::endl;

      // Set input arguments
      int argc=2;
      char *argv[2];

      char argv1[30]="mocktest";
      argv[0]=argv1;
      char argv2[60]="--input-file=libimpl_test_sot_external_interface.so";
      argv[1]=argv2;
      parseOptions(argc,argv);

      std::string finalname("libimpl_test_sot_external_interface.so");
      EXPECT_TRUE(finalname == dynamicLibraryName_);

      // Performs initializatio of libimpl_test_sot_external_interface.so
      loadController();
      // Remove
      CleanUp();
    }
  };

public:

  std::shared_ptr<MockSotLoaderBasic> mockSotLoaderBasic_shrPtr_;

  void SetUp() {
    rclcpp::init(my_argc,my_argv);

    dynamicgraph::RosContext::SharedPtr aRosContext=
       std::make_shared<dynamicgraph::RosContext>();
    aRosContext->rosInit();

    mockSotLoaderBasic_shrPtr_ = std::make_shared<MockSotLoaderBasic>();
    mockSotLoaderBasic_shrPtr_->initializeFromRosContext(aRosContext);

  }

  void TearDown() {
    rclcpp::shutdown();
  }

};


TEST_F(MockSotLoaderBasicTest,)
{
  mockSotLoaderBasic_shrPtr_->checkStateVectorMap();
}

TEST_F(MockSotLoaderBasicTest,ParseOptions)
{
  mockSotLoaderBasic_shrPtr_->testParseOptions();
}

TEST_F(MockSotLoaderBasicTest,TestInitialization)
{
  mockSotLoaderBasic_shrPtr_->testInitialization();
}

TEST_F(MockSotLoaderBasicTest,TestStartStopDG)
{
  mockSotLoaderBasic_shrPtr_->test_start_stop_dg();
}

TEST_F(MockSotLoaderBasicTest,TestInitializeRosNode)
{
  mockSotLoaderBasic_shrPtr_->testInitializeRosNode();
}

TEST_F(MockSotLoaderBasicTest,CleanUp)
{
  mockSotLoaderBasic_shrPtr_->test_cleanup();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  my_argc=argc;
  my_argv=argv;

  int r=RUN_ALL_TESTS();

  return r;
}
