#ifndef OROCOS_RTT_PP_COMPONENT_HPP
#define OROCOS_RTT_PP_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>


class Rtt_pp : public RTT::TaskContext  {

public:

    Rtt_pp(std::string const& name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

protected:

    RTT::OutputPort<geometry_msgs::TransformStamped>opphr;
    RTT::OutputPort<geometry_msgs::TransformStamped>oppor;
    RTT::OutputPort<geometry_msgs::TransformStamped>oppol;

private:


    geometry_msgs::TransformStamped stf_hand_rec_msg,
                                    stf_obj_rec_msg,
                                    stf_obj_live_msg;


    tf::Transform   tf_hand_rec,
                    tf_obj_rec,
                    tf_obj_live;


    tf::StampedTransform    stf_hand_rec,
                            stf_obj_rec,
                            stf_obj_live;

    int count;
    float xhr, xorec, xol;

};
#endif
