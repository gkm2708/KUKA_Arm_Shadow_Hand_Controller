#include "rtt_pp-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <rtt/Port.hpp>
#include <tf2_ros/transform_broadcaster.h>


Rtt_pp::Rtt_pp(std::string const& name) : TaskContext(name) {

    // Ports for object live, object recorded and hand recorded
  ports()->addPort("Out_obj_live", oppol);
  ports()->addPort("Out_obj_rec", oppor);
  ports()->addPort("Out_hand_rec", opphr);

}

bool Rtt_pp::configureHook()    {

    std::cout << "Rtt_pp configured !" <<std::endl;
    xol=-0.3;
    xorec=xhr=0.2;
    count=0;
    return true;
}

bool Rtt_pp::startHook()    {

    std::cout << "Rtt_pp started !" <<std::endl;
    return true;
}

void Rtt_pp::updateHook()   {

  std::cout << "Rtt_pp executes updateHook !" <<std::endl;

  static tf2_ros::TransformBroadcaster  br1,
                                        br2,
                                        br3;


  if (count >= 0 && count < 10)     {

    // Object Recorded
    xorec = xorec - 0.01;

    tf_obj_rec.setOrigin(tf::Vector3(xorec, 0.2, 0.2));
    tf_obj_rec.setRotation(tf::Quaternion(0,0,0,1));

    stf_obj_rec = tf::StampedTransform(tf_obj_rec,ros::Time(),"/world","/objectRec");

    tf::transformStampedTFToMsg(stf_obj_rec, stf_obj_rec_msg);
    br1.sendTransform(stf_obj_rec_msg);

    // Hand Recorded
    xhr = xhr - 0.01;

    tf_hand_rec.setOrigin(tf::Vector3(xhr, 0.2, 0.3));
    tf_hand_rec.setRotation(tf::Quaternion(1,0,0,0));

    stf_hand_rec = tf::StampedTransform(tf_hand_rec,ros::Time(),"/world","/handRec");

    tf::transformStampedTFToMsg(stf_hand_rec, stf_hand_rec_msg);
    br2.sendTransform(stf_hand_rec_msg);

    // Object Live
    xol = xol - 0.01;

    tf_obj_live.setOrigin(tf::Vector3(xol, 0.2, 0.2));
    tf_obj_live.setRotation(tf::Quaternion(0,0,0,1));

    stf_obj_live = tf::StampedTransform(tf_obj_live,ros::Time(),"/world","/objectLive");

    tf::transformStampedTFToMsg(stf_obj_live, stf_obj_live_msg);
    br3.sendTransform(stf_obj_live_msg);

    count++;
  }

  else if (count >= 10){

    // Object Recorded
    xorec = xorec + 0.01;

    tf_obj_rec.setOrigin(tf::Vector3(xorec, 0.2, 0.2));
    tf_obj_rec.setRotation(tf::Quaternion(0,0,0,1));

    stf_obj_rec = tf::StampedTransform(tf_obj_rec,ros::Time(),"/world","/objectRec");

    tf::transformStampedTFToMsg(stf_obj_rec, stf_obj_rec_msg);
    br1.sendTransform(stf_obj_rec_msg);

    // Hand Recorded
    xhr = xhr + 0.01;

    tf_hand_rec.setOrigin(tf::Vector3(xhr, 0.2, 0.3));
    tf_hand_rec.setRotation(tf::Quaternion(1,0,0,0));

    stf_hand_rec = tf::StampedTransform(tf_hand_rec,ros::Time(),"/world","/handRec");

    tf::transformStampedTFToMsg(stf_hand_rec, stf_hand_rec_msg);
    br2.sendTransform(stf_hand_rec_msg);

    // Object Live
    xol = xol + 0.01;

    tf_obj_live.setOrigin(tf::Vector3(xol, 0.2, 0.2));
    tf_obj_live.setRotation(tf::Quaternion(0,0,0,1));

    stf_obj_live = tf::StampedTransform(tf_obj_live,ros::Time(),"/world","/objectLive");

    tf::transformStampedTFToMsg(stf_obj_live, stf_obj_live_msg);
    br3.sendTransform(stf_obj_live_msg);

    count++;

    if (count == 20)
            count = 0;

  }

  std::cout<<"Rtt_pp publishing msgs for count : "<<count<<std::endl;

  oppol.write(stf_obj_live_msg);
  opphr.write(stf_hand_rec_msg);
  oppor.write(stf_obj_rec_msg);

  std::cout<<"Rtt_pp published msgs"<<std::endl;

}

void Rtt_pp::stopHook()     {

    std::cout << "Rtt_pp executes stopping !" <<std::endl;

}

void Rtt_pp::cleanupHook()  {

    std::cout << "Rtt_pp cleaning up !" <<std::endl;

}

ORO_CREATE_COMPONENT(Rtt_pp)

