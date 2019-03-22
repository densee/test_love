#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
//#include <ros/ros.h>
#include <std_msgs/Float64.h>


#include <functional>
//#include <gazebo/gazebo.hh>
//#include <gazebo/physics/physics.hh>
//#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#define PI 3.14159265358979


namespace gazebo
{
  class ModelPush : public ModelPlugin
  {

    double time=0;
    double dt=0;

    common::Time last_update_time_;

    physics::LinkPtr 	link_body_segway;
    physics::JointPtr 	joint_Wheel_left;
    physics::JointPtr 	joint_Wheel_right;

    sensors::SensorPtr ptr_imu_body;
    sensors::ImuSensorPtr sensor_imu_body;

    double dataFromSim_angle_vel_X=0;
    double dataFromSim_angle_vel_Y=0;
    double dataFromSim_angle_vel_Z=0;

    double dataFromCal_angle_gyro_X=0;
    double dataFromCal_angle_gyro_Y=0;
    double dataFromCal_angle_gyro_Z=0;


    double val_LPF=0.99;
    double dataFromCal_accel_X=0;
    double dataFromCal_accel_Y=0;
    double dataFromCal_accel_Z=0;
    double dataFromCal_accel_sum=0;
    double dataFromCal_accel_angularY=0;
    double accSumBuf=0;


    double body_angle_Y=0;

    math::Quaternion body_World_Quaternion;
    double angle_Y_body_world=0;

    double angle_wheel_left=0;
    double angle_wheel_right=0;
    double angle_wheel_left_pre=0;
    double angle_wheel_right_pre=0;
    double angular_vel_wheel_left=0;
    double angular_vel_wheel_right=0;

    double KUWAY_theta_Kp = -80.8226906671380	;
    double KUWAY_theta_Kd = -25.2441828401950	;
    double KUWAY_phi_Kp = -3.16227766016839/5	;
    double KUWAY_phi_Kd = -4.81122004612400/2   ;

    double KUDOS_motor1_Torque=0;
    double torque_wheel_left=0;
    double KUDOS_motor2_Torque=0;
    double torque_wheel_right=0;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;



      this->link_body_segway  	= this->model->GetLink("base");
      this->joint_Wheel_left  	= this->model->GetJoint("joint_left_wheel");
      this->joint_Wheel_right  	= this->model->GetJoint("joint_right_wheel");

      this->ptr_imu_body  	= sensors::get_sensor("imu_body");
      this->sensor_imu_body = std::dynamic_pointer_cast<sensors::ImuSensor>(ptr_imu_body);



      // Listen to the update event. This event is broadcast every
      // simulation iteration.

      this->last_update_time_ = this->model->GetWorld()->GetSimTime();
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));



    }

    // Called by the world update start event
    public: void OnUpdate()
    { //this function working as realtime thread. for example. but seems to be not accurate like interrupt

      common::Time current_time = this->model->GetWorld()->GetSimTime();
      
      dt = current_time.Double() - this->last_update_time_.Double();
      time = time + dt;

      dataFromSim_angle_vel_X = this->sensor_imu_body->AngularVelocity(false)[0];
      dataFromSim_angle_vel_Y = this->sensor_imu_body->AngularVelocity(false)[1];
      dataFromSim_angle_vel_Z = this->sensor_imu_body->AngularVelocity(false)[2];
      std::cout<<"dt: "<<dt<<std::endl;
      dataFromCal_angle_gyro_X = dataFromCal_angle_gyro_X +  dataFromSim_angle_vel_X*dt;
      dataFromCal_angle_gyro_Y = dataFromCal_angle_gyro_Y +  dataFromSim_angle_vel_Y*dt;
      dataFromCal_angle_gyro_Z = dataFromCal_angle_gyro_Z +  dataFromSim_angle_vel_Z*dt;

      //data from simulations' body imu angle
      //Lowpass Filter for Acceleration
      dataFromCal_accel_X= val_LPF*dataFromCal_accel_X + (1-val_LPF)*(this->sensor_imu_body->LinearAcceleration(false)[0]);
      dataFromCal_accel_Y= val_LPF*dataFromCal_accel_Y + (1-val_LPF)*(this->sensor_imu_body->LinearAcceleration(false)[1]);
      dataFromCal_accel_Z= val_LPF*dataFromCal_accel_Z + (1-val_LPF)*(this->sensor_imu_body->LinearAcceleration(false)[2]);
      
      dataFromCal_accel_sum=sqrt( pow(dataFromCal_accel_X,2)+pow(dataFromCal_accel_Y,2)+pow(dataFromCal_accel_Z,2));
      
      dataFromCal_accel_angularY = -atan(dataFromCal_accel_X/sqrt(pow(dataFromCal_accel_Y,2)+pow(dataFromCal_accel_Z,2)));



      //data from simulations' body angle
      body_World_Quaternion = this->link_body_segway->GetWorldPose().rot;
      angle_Y_body_world=body_World_Quaternion.GetAsEuler().y;



      angle_wheel_left	=this->joint_Wheel_left->GetAngle(1).Degree();
      angular_vel_wheel_left=(angle_wheel_left-angle_wheel_left_pre)/dt;
      angle_wheel_left_pre=angle_wheel_left;

      angle_wheel_right	=-(this->joint_Wheel_right->GetAngle(1).Degree());
      angular_vel_wheel_right=(angle_wheel_right-angle_wheel_right_pre)/dt;
      angle_wheel_right_pre=angle_wheel_right;


      std::cout<<"wheelAngle\t"<<angle_wheel_left <<"\t"<< angle_wheel_right<<std::endl;
      std::cout<<"wheelAngleVel\t"<<angular_vel_wheel_left <<"\t"<< angular_vel_wheel_right<<std::endl;


      this->joint_Wheel_right->SetForce(1, 1);




      this->last_update_time_ = current_time;
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}




/*
namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO("Hello World!");
  }

};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
*/
