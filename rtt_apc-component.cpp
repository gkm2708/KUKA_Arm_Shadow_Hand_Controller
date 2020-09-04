#include "rtt_apc-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <time.h>
#include <tf/transform_listener.h>

//------------------------------------------------CONSTRUCTOR-------------------------------------------------

Rtt_apc::Rtt_apc(std::string const& name) : TaskContext(name)   {

    //  Enable below for real data from ros
    /*ports()->addPort("In_obj", input_port_obj);
  ports()->addPort("In_hand", input_port_hand);*/

    // Disable below lines until NewData Flag for real data
    ports()->addPort("In_obj", tf_input_port_obj);
    ports()->addPort("In_hand", tf_input_port_hand);

    ports()->addPort("Out", output_port);
    std::cout << "Rtt_apc constructed !" <<std::endl;
}

//----------------------------------------------CONFIGURE HOOK------------------------------------------------

bool Rtt_apc::configureHook()   {

    std::cout << "Rtt_apc configured !" <<std::endl;
    return true;
}

//---------------------------------------START STOP AND CLEANUP HOOK-----------------------------------------

bool Rtt_apc::startHook()   {

    std::cout << "Rtt_apc started !" <<std::endl;
    return true;
}

void Rtt_apc::stopHook()    {

    std::cout << "Rtt_apc executes stopping !" <<std::endl;
}

void Rtt_apc::cleanupHook() {

    std::cout << "Rtt_apc cleaning up !" <<std::endl;
}

//----------------------------------------------UPDATE HOOK--------------------------------------------------

void Rtt_apc::updateHook()  {

    loadMsg(); // Loading the message and checking if new data is available

    if(newData){

        std::cout<<"Rtt_apc executed updateHook with New Data !! calculate the transform from the recorded object Pose to the recorded arm pose"<<std::endl;
        getTransform(); // Function calculating the transform from hand to object
        publishTransform(); // Function for publishing the transform
        //std::cout<<"Rtt_apc New Transform\n"<<rel_stf_msg<<std::endl;

    }  else {

        publishTransform(); // Function to publish the old transform as no new data
        std::cout << "Rtt_apc no new Data !!" <<std::endl;

    }
}

//--------------------------------------------CUSTOM FUNCTIONS------------------------------------------------
//************************************************************************************************************/

void Rtt_apc::loadMsg(){

    newData = false;

    //Enable below for real data
    /*if(input_port_obj.read(msg_obj) == RTT::NewData)  {

        msg_obj_local = msg_obj;
        newData = true;
        std::cout <<"Rtt_apc loading OBJECT message successfully \n"<<msg_obj_local<<std::endl;

    }

    if(input_port_hand.read(msg_hand) == RTT::NewData)  {

        msg_hand_local = msg_hand;
        newData = true;
        std::cout <<"Rtt_apc loading HAND message successfully \n"<<msg_hand_local<<std::endl;
    }*/


    // Disable below lines until the comment NewCode for real data
    if(tf_input_port_obj.read(tf_msg_obj) == RTT::NewData){

        tf_msg_obj_local = tf_msg_obj;
        newData = true;
        //    std::cout <<"Rtt_apc loading OBJECT message successfully \n"<<tf_msg_obj_local<<std::endl;

    }

    if(tf_input_port_hand.read(tf_msg_hand) == RTT::NewData){

        tf_msg_hand_local = tf_msg_hand;
        newData = true;
        //    std::cout <<"Rtt_apc loading HAND message successfully \n"<<tf_msg_hand_local<<std::endl;

    }

    if(newData == false) {
        std::cout <<"Rtt_apc No New Data in this Cycle" <<std::endl;
    }
}

//************************************************************************************************************/

void Rtt_apc::getTransform() {
    try{

        //Enable below for real data
        /*
        //get stamped transform from recorded object and hand pose
        transform_obj.setOrigin(tf::Vector3(msg_obj_local.pose.position.x, msg_obj_local.pose.position.y, msg_obj_local.pose.position.z));
        transform_obj.setRotation(tf::Quaternion( msg_obj_local.pose.orientation.x, msg_obj_local.pose.orientation.y, msg_obj_local.pose.orientation.z, msg_obj_local.pose.orientation.w));
        stf_obj = tf::StampedTransform(transform_obj,ros::Time(),"/world","/object");
        transform_hand.setOrigin(tf::Vector3(msg_hand_local.pose.position.x, msg_hand_local.pose.position.y, msg_hand_local.pose.position.z));
        transform_hand.setRotation(tf::Quaternion( msg_hand_local.pose.orientation.x, msg_hand_local.pose.orientation.y, msg_hand_local.pose.orientation.z, msg_hand_local.pose.orientation.w));
        stf_hand = tf::StampedTransform(transform_hand,ros::Time(),"/world","/hand");
        // This transform does not contain the fixed offset for tool frame
        */

        // Disable below lines until the comment NewCode for real data
        // Stamped transform for recorded object and hand pose (Does not contain offset)
        tf::transformStampedMsgToTF(tf_msg_obj, stf_obj);
        tf::transformStampedMsgToTF(tf_msg_hand, stf_hand);

        //New Code

        // Extract the hand pose and include offset
        // Hardcoded Fixed Offset to care for the difference in frame
        // for recorded data and frame requirement for kinematics
        // Data is recorded with tactile glove with markers over it.
        // The markers can be at different place than the palm frame of robot.
        // a fixed offset will bring the marker frame to palm frame and then
        // we calculate the transform of object from palm frame

        // value needs to be set and this remains for real and dummy data both.
        transform_offset.setOrigin(tf::Vector3(0,0,0));
        transform_offset.setRotation(tf::Quaternion(0,0,0,1));

        // Build Hand Transform from stamped
        transform_hand.setOrigin(stf_hand.getOrigin());
        transform_hand.setRotation(stf_hand.getRotation());

        // Apply offset
        transform_hand = transform_hand.inverse();
        transform_hand = transform_hand.inverseTimes(transform_offset);

        // Build stamped hand transform with fixed offset
        stf_hand = tf::StampedTransform(transform_hand,ros::Time(),"/world","/hand");

        // Find Relative transform for hand(including offset) and object
        transform_rel = stf_obj.inverseTimes(stf_hand);

        // Build stamped rel transform and convert into message
        rel_stf = tf::StampedTransform(transform_rel, ros::Time(),"/object","/hand");
        tf::transformStampedTFToMsg(rel_stf, rel_stf_msg);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}
//************************************************************************************************************/

void Rtt_apc::publishTransform() {

    output_port.write(rel_stf_msg);

}

//************************************************************************************************************/
ORO_CREATE_COMPONENT(Rtt_apc)
