/*
 * Copyright 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This code is based on the Gazebo ROS Magnetometer Sensor plugin.
 * Original code repository: https://github.com/Darkproduct/gazebo_ros_magnetometer_sensor
 *
 * Modified versions of the original code are licensed under the same
 * Apache License, Version 2.0.
*/

#include <gazebo_ros_magnetometer_sensor/gazebo_ros_magnetometer_sensor.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <cmath>
#include <ignition/math/Vector3.hh>
#include <memory>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRosMagnetometerSensor)

namespace gazebo
{
  GazeboRosMagnetometerSensor::GazeboRosMagnetometerSensor(): 
    SensorPlugin()
  {}

  GazeboRosMagnetometerSensor::~GazeboRosMagnetometerSensor()
  {
    if (node != nullptr)
    {
      node->shutdown();
    }
  }

  void GazeboRosMagnetometerSensor::Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
  {
    sdf=sdf_;
    sensor = std::dynamic_pointer_cast<sensors::MagnetometerSensor>(sensor_);

    if(sensor==nullptr)
    {
      ROS_FATAL("Error: Sensor pointer is NULL!");
      return;
    }

    sensor->SetActive(true);

    if(!LoadParameters())
    {
      ROS_FATAL("Error Loading Parameters!");
      return;
    }

    if (!ros::isInitialized())
    {
      ROS_FATAL("ROS has not been initialized!");
      return;
    }

    node = std::make_unique<ros::NodeHandle>(this->robot_namespace);

    magnetometer_data_publisher = node->advertise<sensor_msgs::MagneticField>(topic_name,1);
    connection = event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo& info) {
      UpdateChild(info);
    });

    last_time = sensor->LastUpdateTime();
  }

  void GazeboRosMagnetometerSensor::UpdateChild(const common::UpdateInfo &/*_info*/)
  {
    common::Time current_time = sensor->LastUpdateTime();

    if (update_rate > 0 && (current_time - last_time).Double() < 1.0 / update_rate)
    {
      return;
    }

    if(magnetometer_data_publisher.getNumSubscribers() > 0)
    {
      ignition::math::Vector3d field = sensor->MagneticField();

      magnetometer_msg.magnetic_field.x = field.X();
      magnetometer_msg.magnetic_field.y = field.Y();
      magnetometer_msg.magnetic_field.z = field.Z();

      magnetometer_msg.header.frame_id   = body_name;
      magnetometer_msg.header.stamp.sec  = current_time.sec;
      magnetometer_msg.header.stamp.nsec = current_time.nsec;

      magnetometer_data_publisher.publish(magnetometer_msg);

      ros::spinOnce();
    }

    last_time = current_time;
  }

  bool GazeboRosMagnetometerSensor::LoadParameters()
  {
    if (sdf->HasElement("robotNamespace"))
    {
      robot_namespace = sdf->Get<std::string>("robotNamespace") + "/";
      ROS_INFO_STREAM("<robotNamespace> set to: " << robot_namespace);
    }
    else
    {
      std::string scoped_name = sensor->ParentName();
      std::size_t it = scoped_name.find("::");

      robot_namespace = "/" + scoped_name.substr(0, it) + "/";
      ROS_WARN_STREAM("missing <robotNamespace>, set to: " << robot_namespace);
    }

    if (sdf->HasElement("topicName"))
    {
      topic_name = robot_namespace + sdf->Get<std::string>("topicName");
      ROS_INFO_STREAM("<topicName> set to: " << topic_name);
    }
    else
    {
      topic_name = robot_namespace + "/mag";
      ROS_WARN_STREAM("missing <topicName>, set to: " << topic_name);
    }

    if (sdf->HasElement("frameName"))
    {
      body_name = sdf->Get<std::string>("frameName");
      ROS_INFO_STREAM("<frameName> set to: " << body_name);
    }
    else
    {
      ROS_FATAL("missing <frameName>, cannot proceed");
      return false;
    }

    if (sdf->HasElement("updateRateHZ"))
    {
      update_rate = sdf->Get<double>("updateRateHZ");
      ROS_INFO_STREAM("<updateRateHZ> set to: " << update_rate);
    }
    else
    {
      update_rate = 1.0;
      ROS_WARN_STREAM("missing <updateRateHZ>, set to: " << update_rate);
    }

    return true;
  }
} // namespace gazebo