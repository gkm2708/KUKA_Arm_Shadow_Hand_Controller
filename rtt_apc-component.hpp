#ifndef OROCOS_RTT_APC_COMPONENT_HPP
#define OROCOS_RTT_APC_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

class Rtt_apc : public RTT::TaskContext {

public:

    // Mandatory for RTT
    Rtt_apc(std::string const& name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    // Custom Functions
    void loadMsg();
    void getTransform();
    void publishTransform();

    tf::Transform   transform_hand,
                    transform_obj,
                    transform_rel,
                    transform_offset;

    tf::StampedTransform    rel_stf,
                            stf_obj,
                            stf_hand;

    tf::TransformListener tl;

    geometry_msgs::TransformStamped rel_stf_msg;

    bool newData, logger;
    int log_level;

protected:

    RTT::InputPort<geometry_msgs::PoseStamped>input_port_obj,input_port_hand;
    RTT::InputPort<geometry_msgs::TransformStamped>tf_input_port_obj,tf_input_port_hand;
    RTT::OutputPort<geometry_msgs::TransformStamped>output_port;

private:
    geometry_msgs::PoseStamped  msg_obj,
                                msg_hand,
                                msg_obj_local,
                                msg_hand_local;

    geometry_msgs::TransformStamped     tf_msg_obj,
                                        tf_msg_hand,
                                        tf_msg_obj_local,
                                        tf_msg_hand_local;

};
#endif
