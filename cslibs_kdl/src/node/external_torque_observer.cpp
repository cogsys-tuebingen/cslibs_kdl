#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cslibs_kdl/residual_vector.h>
struct Node{

    Node(ros::NodeHandle& nh):
        first_(true),
        nh_(nh),
        r_(60)
    {
        std::string in_topic = nh.param<std::string>("in_topic","/joint_state");
        std::string out_topic = nh.param<std::string>("out_topic", "/estimated_ext_torque");
        std::string robot_model = nh.param<std::string>("robot_description", "robot_description");
        std::string chain_root = nh.param<std::string>("chain_root", "jaco_link_base");
        std::string chain_tip = nh.param<std::string>("chain_tip", "jaco_link_hand");
        std::vector<std::string> joint_names;
        joint_names = nh.param<std::vector<std::string>>("joint_names", joint_names);

        sub_ = nh.subscribe(in_topic, 10, &Node::jointStateCb, this);
        pub_ = nh.advertise<sensor_msgs::JointState>(out_topic,3);

        observer_ = cslibs_kdl::ResidualVector(robot_model, chain_root, chain_tip);
        joint_names_ = observer_.getJointNames();
        n_joints_ = observer_.getNrOfJoints();
        data_.gx = nh.param<double>("gravity_x", 0);
        data_.gy = nh.param<double>("gravity_y", 0);
        data_.gz = nh.param<double>("gravity_z", -9.81);
        r_ = ros::Rate(nh.param<double>("frequncy", 60));
        gain_ = nh.param<double>("gain",10);
    }

    void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg)
    {
        out_msg_.header = msg->header;
        if(first_){
            std::vector<double> gains(n_joints_, gain_);
            observer_.setGains(gains);
            last_residual_ = Eigen::VectorXd::Zero(n_joints_);
            last_integral_ = Eigen::VectorXd::Zero(n_joints_);
            last_stamp_ = msg->header.stamp;
            first_ = false;
            
        }
        data_.dt = (msg->header.stamp - last_stamp_).toSec();
        if(n_joints_ != msg->name.size()){
            out_msg_.name = joint_names_;
            data_.resize(n_joints_);
            auto it_d_pos = data_.joint_positions.begin();
            auto it_d_vel = data_.joint_velocities.begin();
            auto it_d_torque = data_.torques.begin();

            for(const std::string& j : joint_names_){
                auto it_pos = msg->position.begin();
                auto it_vel = msg->velocity.begin();
                auto it_torque = msg->effort.begin();
                for(const std::string& msg_j_name : msg->name){
                    if(j == msg_j_name){
                        *it_d_pos = *it_pos;
                        *it_d_vel = *it_vel;
                        *it_d_torque = *it_torque;
                        break;
                    }
                    ++it_pos;
                    ++it_vel;
                    ++it_torque;
                }
                ++it_d_pos;
                ++it_d_vel;
                ++it_d_torque;
            }
            out_msg_.position = data_.joint_positions;
            out_msg_.velocity = data_.joint_velocities;
            out_msg_.effort = data_.torques;

        } else {
            out_msg_ = *msg;
            data_.joint_positions = msg->position;
            data_.joint_velocities = msg->velocity;
            data_.torques = msg->effort;
        }
        
        Eigen::VectorXd new_integral, new_residual;
        observer_.getResidualVector(data_, last_residual_, last_integral_, new_integral, new_residual);

        last_integral_ = new_integral;
        last_residual_ = new_residual;
        last_stamp_ = msg->header.stamp;
        std::size_t next = new_residual.rows();
        for(std::size_t  i = 0; i < std::min(next, out_msg_.effort.size()); ++i){
            out_msg_.effort[i] = new_residual(i);
        }
        pub_.publish(out_msg_);
    }

    void tick()
    {
        r_.sleep();
        ros::spinOnce();
    }

    bool first_;
    ros::NodeHandle& nh_;
    ros::Rate r_;
    ros::Time last_stamp_;
    Eigen::VectorXd last_integral_;
    Eigen::VectorXd last_residual_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    sensor_msgs::JointState out_msg_;
    cslibs_kdl::ResidualData  data_;
    cslibs_kdl::ResidualVector observer_;
    std::vector<std::string> joint_names_;
    std::size_t n_joints_;
    double gain_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "external_torque_observer");
    ros::NodeHandle nh("~");
    Node node(nh);
    while(ros::ok()){
        node.tick();
        ros::spinOnce();
    }


    return 0;
}
