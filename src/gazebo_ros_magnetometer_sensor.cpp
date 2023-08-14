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
#include <XYZgeomag/XYZgeomag.hpp>
#include <chrono>
#include <ctime>

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
    // Load magnetic_field parameters
    if (sdf->HasElement("magneticField"))
    {
      gazebo::physics::WorldPtr world = gazebo::physics::get_world(sensor->WorldName());
      if (world){
        auto magnetic_field = sdf->GetElement("magneticField")->Get<ignition::math::Vector3d>();
        world->SetMagneticField(magnetic_field);
        ROS_INFO_STREAM("Magnetic field: " << magnetic_field);
      }
    }
    else if (sdf->HasElement("sphericalCoordinates"))
    { 
    // Load spherical_coordinates parameters
      gazebo::physics::WorldPtr world = gazebo::physics::get_world(sensor->WorldName());
      if (world){
        sdf::ElementPtr spherical_coordinates = sdf->GetElement("sphericalCoordinates");
        if (spherical_coordinates->HasElement("latitudeDeg") && spherical_coordinates->HasElement("longitudeDeg") && spherical_coordinates->HasElement("elevation")){
          auto latitude = spherical_coordinates->Get<double>("latitudeDeg");
          auto longitude = spherical_coordinates->Get<double>("longitudeDeg");
          auto elevation = spherical_coordinates->Get<double>("elevation");
          ROS_INFO_STREAM("Latitude: " << latitude);
          ROS_INFO_STREAM("Longitude: " << longitude);
          ROS_INFO_STREAM("Elevation: " << elevation);

          // Get year for magnetic field model
          float year;
          if(spherical_coordinates->HasElement("year")){
            // Get year from sdf if provided
            year = spherical_coordinates->Get<double>("year");
          }
          else{
            // Else, use current year from system clock
            auto now = std::chrono::system_clock::now();
            auto now_time = std::chrono::system_clock::to_time_t(now);
            auto now_utc = *std::gmtime(&now_time);
            year = 1900 + now_utc.tm_year + now_utc.tm_yday / 365.25;
          }
          ROS_INFO_STREAM("Year: " << year);

          // Get magnetic field from WMM2020 model
          geomag::Vector position = geomag::geodetic2ecef(latitude,longitude,elevation);
          geomag::Vector mag_field = geomag::GeoMag(year,position,geomag::WMM2020);
          geomag::Elements out = geomag::magField2Elements(mag_field, latitude, longitude);

          // Get vector in ENU frame and in Tesla
          auto magnetic_field = ignition::math::Vector3d(out.east, out.north, -out.down) * 1e-9;

          // Rotate magnetic field by heading if provided
          if (spherical_coordinates->HasElement("headingDeg")){
            auto heading = spherical_coordinates->Get<double>("headingDeg") * M_PI / 180;
            magnetic_field = ignition::math::Quaterniond(0,0,-heading) * magnetic_field;
            ROS_INFO_STREAM("Heading: " << heading);
          }
          // Set magnetic field in world
          world->SetMagneticField(magnetic_field);
          ROS_INFO_STREAM("Magnetic field: " << int(magnetic_field.X() * 1e9) << "E\t" << int(magnetic_field.Y() * 1e9) << "N\t" << int(magnetic_field.Z() * 1e9) << "D");
        }
        else{
          ROS_WARN_STREAM("sphericalCoordinates ignored, all three parameters (latitudeDeg, longitudeDeg, elevation) must be set");
        }
        
      }
    }


    return true;
  }
} // namespace gazebo