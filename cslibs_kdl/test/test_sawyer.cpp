#include <string>
#include <vector>
#include <math.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <ros/package.h>

#include <cslibs_kdl/dynamic_model.h>
#include <cslibs_kdl/kdl_conversion.h>
#include <cslibs_kdl/external_forces.h>
using namespace cslibs_kdl;

DynamicModel sawyerRobot;
std::string urdf_path;
TEST(SawyerTests, fk)
{
    std::vector<double> q = {0, M_PI, M_PI, 0, 0, M_PI, 0};
    tf::Pose res;
    int ec = sawyerRobot.getFKPose(q,res,"right_l6");
    EXPECT_TRUE(ec>=0);
    tf::Pose res4;
    ec = sawyerRobot.getFKPose(q,res4,"right_l4");
    EXPECT_TRUE(ec>=0);
    KDL::Frame T1;
    ec = sawyerRobot.getFKPose(q,T1,"right_l1");
    EXPECT_TRUE(ec>=0);

    std::vector<double> jointAngles = { -1.12, 0.76, -2.36, 1.45, 1.81, -0.46, 4.04};
    KDL::Vector pos1(0.080478, -0.05083, 0.317);
    KDL::Frame p1;
    ec = sawyerRobot.getFKPose(jointAngles, p1, "right_l1");
    EXPECT_TRUE(ec>=0);
    EXPECT_NEAR(pos1.x(), p1.p.x(), 1e-2);
    EXPECT_NEAR(pos1.y(), p1.p.y(), 1e-2);
    EXPECT_NEAR(pos1.z(), p1.p.z(), 1e-2);
    KDL::Frame p4;
    ec = sawyerRobot.getFKPose(jointAngles, p4, "right_l4");
    EXPECT_TRUE(ec>=0);
    KDL::Vector pos4(0.43247, -0.372694, 0.18323);
    EXPECT_NEAR(pos4.x(), p4.p.x(), 1e-2);
    EXPECT_NEAR(pos4.y(), p4.p.y(), 1e-2);
    EXPECT_NEAR(pos4.z(), p4.p.z(), 1e-2);
    KDL::Frame p6;
    ec = sawyerRobot.getFKPose(jointAngles, p6, "right_l6");
    EXPECT_TRUE(ec>=0);
    KDL::Vector pos6(0.403418, -0.756068, 0.295885);
    EXPECT_NEAR(pos6.x(), p6.p.x(), 1e-2);
    EXPECT_NEAR(pos6.y(), p6.p.y(), 1e-2);
    EXPECT_NEAR(pos6.z(), p6.p.z(), 1e-2);

}

TEST(SawyerTests, IK)
{
    try{
        std::vector<double> zero;
        std::vector<double> jointAngles  = {0, M_PI, M_PI, 0, 0, M_PI, 0};
        zero.resize(7,0);
        int nrTests = 10000;
        int fails = 0;
        for(int i = 0; i < nrTests; ++i){
            tf::Pose fk_pose;
            sawyerRobot.getRandomConfig(jointAngles);
            int ec = sawyerRobot.getFKPose(jointAngles,fk_pose,"right_l6");
            EXPECT_TRUE(ec>=0);
            if(ec>=0)
            {
                std::vector<double> ik_solution;
                int ecIK = sawyerRobot.getIKSolution(fk_pose,ik_solution,zero);
                if(ecIK < 0){
                    //                geometry_msgs::Pose msg;
                    //                tf::poseTFToMsg(fk_pose,msg);
                    //                std::cout << "number of test" << i<< std::endl << "Pose: " << msg << std::endl;

                    //                for(auto phi : jointAngles) {
                    //                    std::cout << phi << std::endl;
                    //                }
                    ++fails;
                }
                //            EXPECT_TRUE(ecIK >= 0);
                if(ecIK >= 0){
                    tf::Pose ik_pose;
                    ecIK = sawyerRobot.getFKPose(ik_solution,ik_pose,"right_l6");
                    EXPECT_NEAR(ik_pose.getOrigin().getX(), fk_pose.getOrigin().getX(), 1e-3);
                    EXPECT_NEAR(ik_pose.getOrigin().getY(), fk_pose.getOrigin().getY(), 1e-3);
                    EXPECT_NEAR(ik_pose.getOrigin().getZ(), fk_pose.getOrigin().getZ(), 1e-3);
                    EXPECT_NEAR(ik_pose.getRotation().getX(), fk_pose.getRotation().getX(), 1e-3);
                    EXPECT_NEAR(ik_pose.getRotation().getY(), fk_pose.getRotation().getY(), 1e-3);
                    EXPECT_NEAR(ik_pose.getRotation().getZ(), fk_pose.getRotation().getZ(), 1e-3);
                    EXPECT_NEAR(ik_pose.getRotation().getW(), fk_pose.getRotation().getW(), 1e-3);
                }
            }
        }
        double successRate = 1.0 - ((double)fails)/((double)nrTests);
        EXPECT_NEAR(successRate,0.99, 1e-1);
        std::cout << "success rate: " <<  successRate << std::endl;
    } catch(const std::exception& e){
        std::cout << e.what() << std::endl;
    }
}

TEST(SawyerTests, ExtForces)
{
  ExternalForcesSerialChain extForces;
  extForces.setModel(urdf_path, "right_arm_base_link", "right_l6" , "", "", "");
  std::vector<double> jointAngles = { -1.12, 0.76, -2.36, 1.45, 1.81, -0.46, 4.04};
//  KDL::Rotation r1 = KDL::Rotation::Quaternion(-0.31631,-0.42821,0.828918,-0.171694);
  KDL::Vector pos1(0.080478, -0.05083, 0.317);
  KDL::Frame p1 = extForces.getFKPose(jointAngles, "right_l1");
  EXPECT_NEAR(pos1.x(), p1.p.x(), 1e-2);
  EXPECT_NEAR(pos1.y(), p1.p.y(), 1e-2);
  EXPECT_NEAR(pos1.z(), p1.p.z(), 1e-2);
  KDL::Frame p4 = extForces.getFKPose(jointAngles, "right_l4");
  KDL::Vector pos4(0.43247, -0.372694, 0.18323);
  EXPECT_NEAR(pos4.x(), p4.p.x(), 1e-2);
  EXPECT_NEAR(pos4.y(), p4.p.y(), 1e-2);
  EXPECT_NEAR(pos4.z(), p4.p.z(), 1e-2);
  KDL::Frame p6 = extForces.getFKPose(jointAngles, "right_l6");
  KDL::Vector pos6(0.403418, -0.756068, 0.295885);
  EXPECT_NEAR(pos6.x(), p6.p.x(), 1e-2);
  EXPECT_NEAR(pos6.y(), p6.p.y(), 1e-2);
  EXPECT_NEAR(pos6.z(), p6.p.z(), 1e-2);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_sawyer_cslibs");
    ros::NodeHandle node("~");
    std::string robot_desc_string;
    urdf_path = ros::package::getPath("cslibs_kdl");
    urdf_path += "/test/sawyer.urdf";
    std::cout << urdf_path << std::endl;
    sawyerRobot = DynamicModel(urdf_path,"right_arm_base_link","right_l6");

    return RUN_ALL_TESTS();
}
