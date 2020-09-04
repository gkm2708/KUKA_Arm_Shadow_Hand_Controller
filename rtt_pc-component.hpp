#ifndef OROCOS_RTT_PC_COMPONENT_HPP
#define OROCOS_RTT_PC_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kdl/tree.hpp>
#include <Eigen/Core>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <gazebo_msgs/ModelState.h>

using namespace KDL;

class Rtt_pc : public RTT::TaskContext  {

  public:

    Rtt_pc(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    // Auxilary functions
    void readPorts();
    void getIKForX();
    void getDesHandPos();
    void getFrames();
    void publishData();
    void generateRandomJntSeed(KDL::JntArray&);

    bool logger, newData;
    int log_level, finger_count;

    tf::StampedTransform stf_hand;
    tf::StampedTransform stf_obj;
    tf::Transform   rel_transform,
                    transform_obj_live,
                    transform_hand_desired;

  protected:

    // Listen to current joint states of KUKA Arm
    RTT::InputPort<Eigen::VectorXd> current_joint_states;

    //RTT::InputPort<geometry_msgs::PoseStamped>input_port_obj_live;
    //RTT::InputPort<geometry_msgs::TransformStamped>tf_input_port_obj_live;

    // only to listen to onject published in gazebo
    RTT::InputPort<gazebo_msgs::ModelState> gazebo_obj_live;

    // Listen to transform published from trajectory planner
    RTT::InputPort<geometry_msgs::TransformStamped> input_port;

    // The output to kuka controller for executing motion
    RTT::OutputPort<trajectory_msgs::JointTrajectoryPoint> desired_Joint_angle_Port;

    KDL::Tree 	tree;
    KDL::Chain 	chain;
    KDL::Frames frames;
    KDL::Frame 	f;

    KDL::JntArray   jntarray,
                    desjntarray,
                    jntarraymin,
                    jntarraymax;

    std::vector<std::string> endpoints;

  private:

    //    geometry_msgs::PoseStamped msg_obj_live,msg_obj_live_local;

    geometry_msgs::TransformStamped     tf_msg_obj_live,
                                        tf_msg_obj_live_local;

    geometry_msgs::TransformStamped     msg,
                                        msg_local;

    Eigen::VectorXd                     current_joint_states_msg,
                                        current_joint_states_msg_local;

    trajectory_msgs::JointTrajectory    desired_joint_angle_msg;

    gazebo_msgs::ModelState             ms,
                                        ms_local;

};
#endif
