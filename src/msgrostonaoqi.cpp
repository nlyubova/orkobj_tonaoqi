#include "msgrostonaoqi.hpp"

#include <string>
#include <iostream>
#include <sstream>

#include <alvalue/alvalue.h>

#include <qi/log.hpp>

//! @brief Define Log category
qiLogCategory("orkobj_tonaoqi");

Msgrostonaoqi::Msgrostonaoqi(std::string pip, std::string ip,
                             int port, int pport):
  nh_("~"),
  pip_(pip),
  ip_(ip),
  port_(port),
  pport_(pport),
  brokerName_("NaoROSBroker"),
  eventName("/recognized_object_array"),
  topic_obj("/recognized_object_array"),
  topic_table(""),
  initialized_naoqi(false),
  package_name_("orkobj_tonaoqi")
{
  initialized_naoqi = false;

  nh_.getParam("nao_ip", pip_);
  nh_.getParam("pc_ip", ip_);
  nh_.getParam("nao_port", pport_);
  nh_.getParam("pc_port", port_);
  ROS_INFO_STREAM("-- connecting to " << pip_ << " " << pport_);

  init();
}

Msgrostonaoqi::~Msgrostonaoqi()
{
  try
  {
    pMemoryProxy->unsubscribeToEvent("PictureDetected", package_name_);
  }
  catch (const AL::ALError& e) {
    qiLogError() << e.what();
  }

  try
  {
    pVRProxy->callVoid("unsubscribe", package_name_.c_str());
  }
  catch (const AL::ALError& e) {
    qiLogError() << e.what();
  }
}

void Msgrostonaoqi::init()//int argc, char ** argv)
{
  //initialize ros node parameters
  nh_.param("object_topic",topic_obj,std::string("/recognized_object_array"));
  nh_.param("table_topic",topic_table,std::string("/table_array"));
  ROS_INFO_STREAM("-- listen to the topics " << topic_obj);

  //initialize the NAOqi proxies and events
  if (!connectNaoQi() || !connectProxy())
    ROS_ERROR("Could not connect to NAO proxy");
  else
    initialized_naoqi = true;

  try
  {
    eventName = topic_obj;
    pMemoryProxy = boost::shared_ptr<AL::ALMemoryProxy>(new AL::ALMemoryProxy(m_broker));
    pMemoryProxy->declareEvent(eventName);
    //std::cout << "-- notifying the NAOqi events: " << eventName << std::endl;
  }
  catch (const AL::ALError& e) {
    qiLogError() << e.what();
  }

  //initialize the ros subscriber
  sub_obj_ = nh_.subscribe<object_recognition_msgs::RecognizedObjectArray>(topic_obj, 10, &Msgrostonaoqi::notify, this);


  try
  {
    pMemoryProxy->subscribeToEvent("PictureDetected", package_name_, "PictureDetected");
  }
  catch (const AL::ALError& e) {
    qiLogError() << e.what();
  }
}

void Msgrostonaoqi::notify(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
{
  object_recognition_msgs::RecognizedObjectArray::_objects_type::const_iterator it_o = msg->objects.begin();
  while (it_o != msg->objects.end())
  {
    std::vector <float> position;
    position.push_back(it_o->pose.pose.pose.position.x);
    position.push_back(it_o->pose.pose.pose.position.y);
    position.push_back(it_o->pose.pose.pose.position.z);
    std::vector <float> orientation;
    orientation.push_back(it_o->pose.pose.pose.orientation.x);
    orientation.push_back(it_o->pose.pose.pose.orientation.y);
    orientation.push_back(it_o->pose.pose.pose.orientation.z);
    orientation.push_back(it_o->pose.pose.pose.orientation.w);

    AL::ALValue valOutcome;
    valOutcome.arrayPush(it_o->type.key);
    valOutcome.arrayPush(it_o->confidence);
    valOutcome.arrayPush(position);
    valOutcome.arrayPush(orientation);

std::cout << "outcome: " << valOutcome << std::endl;

    if (initialized_naoqi)
      pMemoryProxy->raiseMicroEvent(eventName, valOutcome);

    ++it_o;
  }
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

bool Msgrostonaoqi::connectNaoQi()
{
  // Need this to for SOAP serialization of floats to work
  setlocale(LC_NUMERIC, "C");
  try
  {
    m_broker = AL::ALBroker::createBroker(brokerName_, ip_, port_, pip_, pport_, false);
  }
  catch(const AL::ALError& e)
  {
    ROS_ERROR( "Failed to connect broker to: %s:%d", pip_.c_str(), port_);
    //AL::ALBrokerManager::getInstance()->killAllBroker();
    //AL::ALBrokerManager::kill();
    return false;
  }
  ROS_INFO("NAOqi broker ready");
  return true;
}

bool Msgrostonaoqi::connectProxy()
{
  if (!m_broker)
  {
     ROS_ERROR("Broker is not ready. Have you called connectNaoQi()?");
     return false;
  }
  try
  {
    pMemoryProxy = boost::shared_ptr<AL::ALMemoryProxy>(new AL::ALMemoryProxy(m_broker));
  }
  catch (const AL::ALError& e)
  {
    ROS_ERROR("Could not create ALMemoryProxy.");
    return false;
  }
  ROS_INFO("Proxy to ALMemory is ready");


  //connnect to vision recognition
  try
  {
    pVRProxy = m_broker->getProxy("ALVisionRecognition");
    //pVRProxy = boost::shared_ptr<AL::ALProxy>(new AL::ALProxy(m_broker, "ALVisionRecognition"));

    pVRProxy->callVoid("setResolution", 2);
    pVRProxy->callVoid("subscribe", package_name_.c_str());
  }
  catch (const AL::ALError& e)
  {
     ROS_ERROR("Could not create ALVisionRecognitionProxy");
     return -1;
  }
  ROS_INFO("Proxy to ALVisionRecognition is ready");

    //std::string filename = "";
    //    bool ALVisionRecognitionProxy::learnFromFile(const std::string& filename, const std::string& name, const std::vector<std::string>& tags, bool isWholeImage = true, bool forced = false)
    //std::string result1 = pVRProxy->call<std::string>("learnFromFile", filename, "cola", "can", true, true);
    //std::string result = pVRProxy->call<std::string>("detectFromFile", filename);


  return true;
}

void Msgrostonaoqi::PictureDetected(const AL::ALValue &value, const AL::ALValue &msg)
{

  std::cout <<  " ***********" << msg[0]<< std::endl;
}
