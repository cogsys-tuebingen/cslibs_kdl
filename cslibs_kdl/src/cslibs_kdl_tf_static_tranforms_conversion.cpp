#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cslibs_kdl/yaml_to_kdl_tranform.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cslibs_kdl_tf_static_transforms_conversion");
    tf::TransformListener listener;
    std::vector<std::string> links = {"jaco_link_base", "jaco_link_1", "jaco_link_2", "jaco_link_3", "jaco_link_4", "jaco_link_5", "jaco_link_6"};
    std::vector<std::string> accs = {"jaco_accelerometer_0", "jaco_accelerometer_1", "jaco_accelerometer_2", "jaco_accelerometer_3", "jaco_accelerometer_4", "jaco_accelerometer_5"};
    //    std::vector<std::string> accs = {"jaco_accelerometer_1", "jaco_accelerometer_2", "jaco_accelerometer_3", "jaco_accelerometer_4", "jaco_accelerometer_5", "jaco_accelerometer_6"};

    std::vector<cslibs_kdl::KDLTransformation> tosave;
    for(int i = 0; i < 6; ++i)
    {
        listener.waitForTransform(links[i],accs[i],ros::Time(0),ros::Duration(3));
        try{
            tf::StampedTransform transform;
            listener.lookupTransform(links[i], accs[i],ros::Time(0),transform);
            cslibs_kdl::KDLTransformation t;
            t.name = accs[i];
            t.parent = links[i];
            KDL::Frame frame;
            tf::transformTFToKDL(transform, frame);
            t.frame = frame;
            tosave.push_back(t);

        }
        catch( tf::TransformException ex){
            ROS_ERROR("transfrom exception : %s",ex.what());
        }
    }

    if(tosave.size() == 6)
    {
        cslibs_kdl::save("/tmp/acc_transforms.yaml",tosave);
    }


    return 0;
}

