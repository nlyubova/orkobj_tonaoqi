/*! @file
 * @author Natalya Lyubova <nlyubova@aldebaran-robotics.com>
 * @copyright Copyright (c) Aldebaran Robotics 2014 All Rights Reserved*/

#include <std_msgs/Header.h>

#include "almsgrostonaoqi.h"

#include <qi/log.hpp>

//! @brief Define Log category
qiLogCategory("ALMsgRosToNaoqi");

namespace AL {

const std::string ALMsgRosToNaoqi::eventName = "aListValRos";

ALMsgRosToNaoqi::ALMsgRosToNaoqi(
    boost::shared_ptr<AL::ALBroker> broker,
    const std::string& name):
  AL::ALModule(broker, name)
{
  topic = "/recognized_object_array";
  //rate = 20;
}

ALMsgRosToNaoqi::~ALMsgRosToNaoqi()
{
}

void ALMsgRosToNaoqi::init()
{
  //initialize proxies and declare events
  try {
    pMemoryProxy = getParentBroker()->getMemoryProxy();
    pMemoryProxy->declareEvent(eventName);
  }
  catch (const AL::ALError& e) {
    qiLogError() << e.what();
  }

  //ros::init();
  sub_msg = node.subscribe(topic.c_str(), 1000, &ALMsgRosToNaoqi::notify);
  //sub_msg = node.subscribe(topic.c_str(), 1000, &ALMsgRosToNaoqi::notify, object_recognition_msgs);
}

void ALMsgRosToNaoqi::process()
{
  //listen
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.

  // Tell ROS how fast to run this node.
  //ros::Rate r(rate);
  if (node.ok()) {
    ros::spin();
    //rate.sleep();
  }

}

void ALMsgRosToNaoqi::notify(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
//void ALMsgRosToNaoqi::notify(const boost::shared_ptr<object_recognition_msgs::RecognizedObjectArray const>& msg)
{
  //notify
  /*AL::ALValue valOutcome;
  valOutcome.arrayPush(msg->data.c_str());
  valOutcome.arrayPush(msg->confidence);
  valOutcome.arrayPush(msg->position);
  valOutcome.arrayPush(msg->orientation);
  pMemoryProxy->raiseMicroEvent(eventName_coucou, valOutcome);*/

  object_recognition_msgs::RecognizedObjectArray::_objects_type::const_iterator it_o = msg->objects.begin();
  while (it_o != msg->objects.begin())
  {
    std::cout << it_o->type.key << " "
              << it_o->confidence << " "
              << it_o->pose.pose.pose.position << " "
              << it_o->pose.pose.pose.orientation << " "
              << std::endl;
    ++it_o;
  }
}

}
