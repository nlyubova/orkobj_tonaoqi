#ifndef MSGROSTONAOQI_H
#define MSGROSTONAOQI_H

#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <alcommon/albroker.h>
#include <alproxies/almemoryproxy.h>

#include <boost/program_options.hpp>

#include "ros/ros.h"

class Msgrostonaoqi
{
public:
  Msgrostonaoqi(std::string pip, std::string ip, int port, int pport);
  ~Msgrostonaoqi(){};
  void init(int argc, char ** argv);
  void parse_command_line(int argc, char ** argv);
  bool connectNaoQi();
  bool connectProxy();
  void notify(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_obj_;

  std::string pip_;
  std::string ip_;
  int port_;
  int pport_;
  std::string brokerName_;
  boost::shared_ptr<AL::ALBroker> m_broker;
  boost::shared_ptr<AL::ALMemoryProxy> pMemoryProxy;

  std::string eventName;
  std::string topic_obj, topic_table;
  bool initialized_naoqi;

};

#endif // MSGROSTONAOQI_H
