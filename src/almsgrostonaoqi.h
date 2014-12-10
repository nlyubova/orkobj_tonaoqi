#ifndef ACTIONRECOGNITION_H
#define ACTIONRECOGNITION_H

#include <iostream>
#include <fstream>
#include <sys/times.h>
#include <sys/time.h>
#include <math.h>
#include <string>
#include <time.h>
#include <sys/dir.h>

#include <stdio.h>
#include <stdlib.h>

#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>

#include <qi/path.hpp>

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alerror/alerror.h>

#include <alproxies/almemoryproxy.h>
#include <alproxies/alledsproxy.h>

#include <ros/ros.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

namespace AL {
class ALBroker;

class ALMsgRosToNaoqi: public AL::ALModule
{
  public:
    ALMsgRosToNaoqi(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
    virtual ~ALMsgRosToNaoqi();

    virtual void init();
    void process();

    boost::shared_ptr<AL::ALMemoryProxy> pMemoryProxy;

  private:
    void notify(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);
    //void notify(const boost::shared_ptr<object_recognition_msgs::RecognizedObjectArray const>& msg);

    static const std::string eventName;

    ros::NodeHandle node;
    //NodeExample *node_example = new NodeExample();
    ros::Subscriber sub_msg;
    std::string topic;
    //ros::Rate rate;
};

}

#endif
