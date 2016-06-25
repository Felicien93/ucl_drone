/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 */

#include <ucl_drone/imgproc2D/imgproc2D.h>

std::vector< Synchro* > Synchro::instances_list;
bool Synchro::callback_flag = false;
boost::mutex Synchro::callback_mutex;

Synchro::Synchro()
{
  instances_list.push_back(this);
  this->thread_flag = false;
}

Synchro::~Synchro()
{
}

bool Synchro::getThreadFlag()
{
  boost::mutex::scoped_lock scoped_lock(thread_mutex);
  return this->thread_flag;
}

bool Synchro::getCallbackFlag()
{
  boost::mutex::scoped_lock scoped_lock(callback_mutex);
  return callback_flag;
}

void Synchro::sendRequest()
{
  {
    boost::mutex::scoped_lock scoped_lock(callback_mutex);
    callback_flag = true;
  }
  {
    boost::mutex::scoped_lock scoped_lock(thread_mutex);
    this->thread_flag = false;
  }
  ROS_DEBUG("Synchro: sent resquest");
}

boost::shared_ptr< ProcessedImage > Synchro::getResponse()
{
  // ros::Rate r(100);
  while (!this->getThreadFlag())
  {
    // r.sleep();
    boost::this_thread::interruption_point();
  }
  boost::mutex::scoped_lock scoped_lock(thread_mutex);
  return response;
}

void Synchro::sendResponse(const boost::shared_ptr< ProcessedImage > cam_img)
{
  {
    boost::mutex::scoped_lock scoped_lock(callback_mutex);
    callback_flag = false;
  }
  for (int i = 0; i < instances_list.size(); i++)
  {
    boost::mutex::scoped_lock scoped_lock(instances_list[i]->thread_mutex);
    if (instances_list[i]->thread_flag == false)
    {
      instances_list[i]->thread_flag = true;
      instances_list[i]->response = cam_img;
    }
  }
  ROS_DEBUG("Synchro: sent response");
}
