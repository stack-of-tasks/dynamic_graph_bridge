
#include <gmock/gmock.h>
#include "dynamic_graph_bridge/ros2_init.hh"
#include "dynamic_graph_bridge/sot_loader_basic.hh"

bool CoordinationWithSpin=false;

class MockSotLoaderBasicTest: public ::testing::Test {

public:

  class MockSotLoaderBasic: public SotLoaderBasic {
  public:
    
    
    void checkStateVectorMap() {
      EXPECT_EQ(static_cast<int>(stateVectorMap_.size()),2);
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

    void testInitialization() {
      
      // Void call
      int argc=2;
      char argv1[30]="mocktest";
      char *argv[2];
      argv[0]=argv1;
      // Test input file
      char argv2[60]="--input-file=libimpl_test_sot_external_interface.so";
      argv[1]=argv2;
      parseOptions(argc,argv);
      
      std::string finalname("libimpl_test_sot_external_interface.so");
      EXPECT_TRUE(finalname == dynamicLibraryName_);

      // Performs initializatio of libimpl_test_sot_external_interface.so
      Initialization();
      EXPECT_TRUE(sotRobotControllerLibrary_ != 0);
      std::cout << "sotRobotControllerLibrary_: " << sotRobotControllerLibrary_
              << std::endl;
      EXPECT_TRUE(sotController_ != nullptr);
      std::cout << "sotController_: " << sotController_
                << std::endl;

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
    }
  };

public:

  std::shared_ptr<MockSotLoaderBasic> mockSotLoaderBasic_shrPtr_;

  void SetUp() {
    mockSotLoaderBasic_shrPtr_ = std::make_shared<MockSotLoaderBasic>();
  }

};

TEST_F(MockSotLoaderBasicTest,CheckStateVectorMap)
{
  mockSotLoaderBasic_shrPtr_->readSotVectorStateParam();
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

TEST_F(MockSotLoaderBasicTest,CleanUp)
{
  mockSotLoaderBasic_shrPtr_->test_cleanup();
  CoordinationWithSpin=true;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc,argv);

  int r=RUN_ALL_TESTS();

  while (!CoordinationWithSpin) {
    dynamicgraph::rosInitGetExecutor()->spin_once();
    usleep(5000);
  }

  rclcpp::shutdown();

  return r;
}
