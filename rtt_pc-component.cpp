#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "kdl_conversions/kdl_msg.h"
#include "rtt_pc-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/plugin/PluginLoader.hpp>
#include <rtt_rosclock/rtt_rosclock.h>
#include <ros/ros.h>
#include <time.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>
#include <tf2_ros/transform_broadcaster.h>


using namespace std;

//------------------------------------------------CONSTRUCTOR-------------------------------------------------

Rtt_pc::Rtt_pc(std::string const& name) : TaskContext(name) {

    // Three Input Ports

    // Listen for current joint states
    ports()->addPort("In_hand_live",current_joint_states);

    // Listen to live object position from gazebo
    ports()->addPort("In_obj_live", gazebo_obj_live);

    // Listen to the transform from RTT_APC
    ports()->addPort("In", input_port); // for transform from RTT_APC

    // Output the desired joint states as per the transform from RTT_APC
    ports()->addPort("desired_Joint_angle_Port", desired_Joint_angle_Port);

    ROS_INFO("Rtt_pc::constructer");

}

//----------------------------------------------CONFIGURE HOOK------------------------------------------------

bool Rtt_pc::configureHook(){

    // Load robot model from urdf
    urdf::Model robot_model;
    std::string robot_desc;
    ros::NodeHandle* rosnode = new ros::NodeHandle();

    if(!rosnode->getParam("/right/robot_description", robot_desc)) {
        ROS_FATAL("Rtt_pc Could not find kuka /robot_description on param server");
        return false;
    }

    robot_model.initString(robot_desc);
    kdl_parser::treeFromUrdfModel(robot_model, tree);

    // Joint Limits for KUKA ARM
    jntarraymin.resize(7);
    jntarraymax.resize(7);
    jntarraymin.data << -2.96 , -2.09, -2.96, -2.09, -2.96, -2.09, -2.96;
    jntarraymax.data << 2.96 , 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;

    // Extract chain from 33 DOF tree including KUKA ARM and SHADOW HAND
    tree.getChain("world","rh_tool_frame",chain);

    delete rosnode;
    return true;
}

//---------------------------------------START STOP AND CLEANUP HOOK-----------------------------------------

bool Rtt_pc::startHook()   {

    ROS_INFO("Rtt_pc started !");
    return true;
}

void Rtt_pc::stopHook()    {

    ROS_INFO("Rtt_pc executes stopping !");
}

void Rtt_pc::cleanupHook() {

    ROS_INFO("Rtt_pc cleaning up !");
}

//----------------------------------------------UPDATE HOOK--------------------------------------------------

void Rtt_pc::updateHook(){

    readPorts();

    if(newData) {

        ROS_INFO("Rtt_pc Got all required data");

        //        getDesHandPos();    // calculate transform of live object position
        //        getFrames();        // Get a Frame Variable
        //        getIKForX();        // IK for desired hand position
        //        publishData();      // publish the joint space values

    } else {

        ROS_INFO("Rtt_pc executes updateHook!! No New Data !!");

    }
}

//--------------------------------------------CUSTOM FUNCTIONS------------------------------------------------
//************************************************************************************************************/

void Rtt_pc::readPorts() {

    newData = false;

    if(current_joint_states.read(current_joint_states_msg) == RTT::NewData) {

        current_joint_states_msg_local = current_joint_states_msg;
        newData = true;
        //std::cout <<"Rtt_pc loading CURRENT HAND POSE message successfully \n"<<current_joint_states_msg_local<<std::endl;
    } else {
        std::cout<<"Rtt_pc No new data from KUKA_META_CONTROLLER, using old data \n";
    }

    // Disable Below for real data until NewData Flag
    if(gazebo_obj_live.read(ms) == RTT::NewData) {

        ms_local = ms ;
        newData = true;
        std::cout <<"Rtt_pc loading CURRENT OBJECT POSITION message successfully \n"<<ms_local<<std::endl;
    } else {
        std::cout<<"Rtt_pc No new data for LIVE OBJECT POSITION \n";
    }

    if(input_port.read(msg) == RTT::NewData) {

        msg_local = msg;
        newData = true;
        //      std::cout <<"Rtt_pc loading TRANSFORMATION message successfully \n"<<msg_local<<std::endl;
    } else {
        std::cout<<"Rtt_pc No new TRANSFORM from APC \n";
    }

    // Enable Below for real data
    /*    if(input_port_obj_live.read(msg_obj_live) == RTT::NewData) {
      msg_obj_live_local = msg_obj_live;
      newData = true;
      std::cout <<"Rtt_pc loading CURRENT OBJECT POSITION message successfully \n"<<msg_obj_live_local<<std::endl;
    }
    std::cout<<"Rtt_pc No new data for LIVE OBJECT POSITION \n";*/

}
//************************************************************************************************************/
void Rtt_pc::getDesHandPos() {

    // Create transform for the live object position
    //    transform_obj_live.setOrigin(tf::Vector3(msg_obj_live_local.pose.position.x,msg_obj_live_local.pose.position.y, msg_obj_live_local.pose.position.z));
    //    transform_obj_live.setRotation(tf::Quaternion(msg_obj_live_local.pose.orientation.x, msg_obj_live_local.pose.orientation.y,msg_obj_live_local.pose.orientation.z, msg_obj_live_local.pose.orientation.w));

    // Disable below for real data until newData flag
    // Get stamped transform from message and build transform for object
    //    tf::transformStampedMsgToTF(tf_msg_obj_live, stf_obj);


    //    transform_obj_live.setOrigin(stf_obj.getOrigin());
    //    transform_obj_live.setRotation(stf_obj.getRotation());
    // NewData

    // Create transform for the relative transform from stamped transform message
    rel_transform.setOrigin(tf::Vector3(msg_local.transform.translation.x,msg_local.transform.translation.y,msg_local.transform.translation.z));
    rel_transform.setRotation(tf::Quaternion(msg_local.transform.rotation.x,msg_local.transform.rotation.y,msg_local.transform.rotation.z,msg_local.transform.rotation.w));

    // apply transformation to current object position and get desired hand position
    transform_obj_live = transform_obj_live.inverse();
    transform_hand_desired = transform_obj_live.inverseTimes(rel_transform);

    // Build stamped transform and publish
    stf_hand = tf::StampedTransform(transform_hand_desired, ros::Time(),"/world","/handDesired");
    tf::transformStampedTFToMsg(stf_hand, msg_local);

    // Disable below if visualization in rviz is not required
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(msg_local);
}
//************************************************************************************************************/
void Rtt_pc::getFrames() {
    // Enable Below for real data
    //    f = KDL::Frame(KDL::Rotation::Quaternion(msg_local.transform.rotation.x, msg_local.transform.rotation.y, msg_local.transform.rotation.z, msg_local.transform.rotation.w),KDL::Vector(msg_local.transform.translation.x, msg_local.transform.translation.y, msg_local.transform.translation.z));
    // Disable Below for real data
    f = KDL::Frame(KDL::Rotation::Quaternion(1.0, 0.0, 0.0, 0.0),KDL::Vector(-0.2, 0.1, 0.34));
}
//************************************************************************************************************/
void Rtt_pc::getIKForX() {
    //do IK for absolute pos and save the result
    if(current_joint_states_msg_local.rows() == 0) {ROS_ERROR("Rtt_pc Current_joint_states_msg should not be empty");return;}
    jntarray.data = current_joint_states_msg_local;
    desjntarray.resize(7);
    int result = -1;
    KDL::ChainFkSolverPos_recursive fksolver1(chain);
    KDL::ChainIkSolverVel_pinv iksolver1v(chain);
    KDL::ChainIkSolverPos_NR_JL iksolver1(chain, jntarraymin, jntarraymax, fksolver1,iksolver1v, 1000, 1e-3);
    // Try with random positions if for the real position a solution is not converged for ten times and then give error. Logic explained in the report.
    for(int i=0; i < 10 && result < 0; i++){
        result = iksolver1.CartToJnt(jntarray,f,desjntarray);
        generateRandomJntSeed(jntarray);
        std::cout<<"\nRtt_pc =========>  "<<result<<"  : "<<iksolver1.strError(result)<<" for values "<<"\nRtt_pc "<<f.p[0]<<" "<<f.p[1]<<" "<<f.p[2]<<" "<<std::endl;
    }
}
//************************************************************************************************************/
void Rtt_pc::generateRandomJntSeed(KDL::JntArray &jntarray)
{
    for(int i=0; i < 7; i++){
        double min = jntarraymin(i);
        double max = jntarraymax(i);
        double r= min + ((double)rand()) / RAND_MAX *(max-min);
        jntarray(i)= r;
    }
}
//************************************************************************************************************/
void Rtt_pc::publishData() {
    desired_joint_angle_msg.points.resize(1);
    desired_joint_angle_msg.joint_names.resize(7);
    desired_joint_angle_msg.points[0].positions.resize(7);
    desired_joint_angle_msg.joint_names[0] = "ra_arm_0_joint";
    desired_joint_angle_msg.joint_names[1] = "ra_arm_1_joint";
    desired_joint_angle_msg.joint_names[2] = "ra_arm_2_joint";
    desired_joint_angle_msg.joint_names[3] = "ra_arm_3_joint";
    desired_joint_angle_msg.joint_names[4] = "ra_arm_4_joint";
    desired_joint_angle_msg.joint_names[5] = "ra_arm_5_joint";
    desired_joint_angle_msg.joint_names[6] = "ra_arm_6_joint";
    for(int i=0;i<7;i++) desired_joint_angle_msg.points[0].positions[i] = desjntarray(i);
//    desired_Joint_angle_Port.write(desired_joint_angle_msg);
}
//************************************************************************************************************/
ORO_CREATE_COMPONENT(Rtt_pc)
