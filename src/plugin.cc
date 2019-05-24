#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>


/// TEST///



namespace gazebo                          
{
    //Name the 'class name' as you like
class PIDJoints : public ModelPlugin             
  {
    
    //For Link,Joint,Model part
    physics::LinkPtr body;    
    physics::LinkPtr right_wheel;    
    physics::LinkPtr left_wheel;    
    physics::JointPtr right_wheel_joint;                     
    physics::JointPtr left_wheel_joint;                     
    physics::ModelPtr model;
    
    //setting for getting <dt>(=derivative time) 
    common::Time last_update_time;
    event::ConnectionPtr update_connection; 
    double dt; 
    double time=0;
    
    //setting for IMU
    sensors::SensorPtr Sensor;
    sensors::ImuSensorPtr IMU;
    double IMU_Update;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    double linear_acc_x = 0;
    double linear_acc_y = 0;
    double linear_acc_z = 0;

    //setting for control
    double PI=3.141592;
    double angle_x = 0;
    double angle_y = 0;
    double angle_z = 0;
    double R=0;
    double angle_y_from_acc=0;
    double angle_y_from_acc_m=0;
    double right_motor_Torque, left_motor_Torque;
    double right_wheel_Torque, left_wheel_Torque;
    double right_wheel_angle, left_wheel_angle;
    double pre_right_wheel_angle = 0, pre_left_wheel_angle = 0;
    double right_wheel_angular_velocity, left_wheel_angular_velocity;
    double right_wheel_angular_acc, left_wheel_angular_acc;
    double theta_Kp = -80.8226906671380 ; //-47.2482 -38.2797
    double theta_Kd = -25.2441828401950 ; //-14.4994 -13.0518
    double phi_Kp = -3.16227766016839/5 ; //-4.4721 -1
    double phi_Kd = -4.81122004612400/2 ; //-5.2808 -2.7132
    double cnt=0;
    
    //setting for getting world angle
    double world_angle_y=0;
    math::Pose world_pose;
    math::Vector3 world_vector3;
    math::Quaternion world_Quaternion;

    //setting for sensor compensation
    double angle_y_m=0;
    double angle_y_comp=0;
    double comp_gain=0.999;
    
    
    //setting for telecommunication
    ros::NodeHandle n;
    ros::Publisher P_Times;
    ros::Publisher P_right_wheel_torque;
    ros::Publisher P_left_wheel_torque;
    ros::Publisher P_right_wheel_angle;
    ros::Publisher P_left_wheel_angle;
    ros::Publisher P_right_wheel_angular_velocity;
    ros::Publisher P_left_wheel_angular_velocity;
    ros::Publisher P_gyro_angle_y;
    ros::Publisher P_acc_angle_y;
    ros::Publisher P_complementary_angle_y;
//    
    std_msgs::Float64 m_Times;
    std_msgs::Float64 m_right_wheel_torque;
    std_msgs::Float64 m_left_wheel_torque;
    std_msgs::Float64 m_right_wheel_angle;
    std_msgs::Float64 m_left_wheel_angle;
    std_msgs::Float64 m_right_wheel_angular_velocity;
    std_msgs::Float64 m_left_wheel_angular_velocity;

    ros::Publisher P_angle_y;
    ros::Publisher P_world_angle_y;
    ros::Publisher P_angle_y_from_acc;
    ros::Publisher P_angle_y_from_acc_m;
    ros::Publisher P_angle_y_comp;
    std_msgs::Float64 m_angle_y;
    std_msgs::Float64 m_world_angle_y;
    std_msgs::Float64 m_angle_y_from_acc;
    std_msgs::Float64 m_angle_y_comp;
    
    //For model load
  public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
     // model = link + joint +sensor
      this->model = _model;

     //"Name" is the variable that you name in model.sdf file.
      this->body = this->model->GetLink("body");
      this->left_wheel = this->model->GetLink("left_wheel");
      this->right_wheel = this->model->GetLink("right_wheel");
      this->right_wheel_joint = this->model->GetJoint("right_wheel_joint");
      this->left_wheel_joint = this->model->GetJoint("left_wheel_joint");

     //setting for getting dt
      this->last_update_time = this->model->GetWorld()->GetSimTime();
      this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PIDJoints::UpdatePID, this));

     //setting for imu sensor
      this->Sensor = sensors::get_sensor("IMU");
      this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor); 
     
     //setting for communication
      P_Times = n.advertise<std_msgs::Float64>("times",1);
//      P_gyro_angle_y = n.advertise<std_msgs::Float64>("gyro_angle",1);   //?
//      P_acc_angle_y = n.advertise<std_msgs::Float64>("acc_angle",1);     //?
//      P_world_angle_y = n.advertise<std_msgs::Float64>("world_angle",1);
      P_right_wheel_torque = n.advertise<std_msgs::Float64>("right_wheel_torque",1);
      P_left_wheel_torque = n.advertise<std_msgs::Float64>("left_wheel_torque",1);
      P_right_wheel_angle = n.advertise<std_msgs::Float64>("right_wheel_angle",1);
      P_left_wheel_angle = n.advertise<std_msgs::Float64>("left_wheel_angle",1);
      P_right_wheel_angular_velocity = n.advertise<std_msgs::Float64>("right_wheel_angular_velocity",1);
      P_left_wheel_angular_velocity = n.advertise<std_msgs::Float64>("left_wheel_angular_velocity",1);
      P_angle_y=n.advertise<std_msgs::Float64>("angle_y",1);
      P_world_angle_y=n.advertise<std_msgs::Float64>("world_angle_y",1);
      P_angle_y_from_acc=n.advertise<std_msgs::Float64>("angle_y_from_acc",1);
      P_angle_y_comp=n.advertise<std_msgs::Float64>("angle_y_comp",1);
      ros::Rate loop_rate(1000);
       }

    void UpdatePID()
    {
        
      //setting for getting dt & time
      common::Time current_time = this->model->GetWorld()->GetSimTime();
      dt = current_time.Double() - this->last_update_time.Double();
      time=time+dt;
      if(time==0)
      {   
          angle_x = 0;
          angle_y = 0;
          angle_z = 0;  
          angular_velocity_x = 0;
          angular_velocity_y = 0;
          angular_velocity_z = 0;  
      }
      
//      std::cout<<"time"<<time<<std::endl;
//      std::cout<<"angle_y"<<angle_y*180/PI<<std::endl;
      //Gyroscope value
      angular_velocity_x = this->IMU->AngularVelocity(false)[0];
      angular_velocity_y = this->IMU->AngularVelocity(false)[1];
      angular_velocity_z = this->IMU->AngularVelocity(false)[2];
      angle_x =  angle_x + angular_velocity_x*dt;
      angle_y =  angle_y + angular_velocity_y*dt;
      angle_z =  angle_z + angular_velocity_z*dt;
      
//      std::cout <<"angle_y"<< angle_y << std::endl;

      //Accelerometer value
      linear_acc_x = this->IMU->LinearAcceleration(false)[0];
      linear_acc_y = this->IMU->LinearAcceleration(false)[1];
      linear_acc_z = this->IMU->LinearAcceleration(false)[2];
      R=sqrt(pow(linear_acc_x,2)+pow(linear_acc_y,2)+pow(linear_acc_z,2));
      angle_y_from_acc=atan((sqrt(pow(linear_acc_x,2)+pow(linear_acc_y,2)))/R);
      
      if(math::isnan(angle_y_from_acc))
      {
          angle_y_from_acc=angle_y_from_acc_m;
      }
          angle_y_from_acc_m=angle_y_comp;
              
      angle_y_comp = (comp_gain*(angle_y_comp+linear_acc_y*dt)+(1-comp_gain)*angle_y_from_acc);
     
      if(math::isnan(angle_y_comp))
      {
          angle_y_comp=angle_y_m;
      }
        angle_y_m=angle_y_comp;
        
//      std::cout <<"angle_y_comp"<< angle_y_comp << std::endl;
      
      //getting world_angle
      world_Quaternion = this->body->GetWorldPose().rot;
      world_angle_y=world_Quaternion.GetAsEuler().y;

      //getting states of wheel's angle & velocity
      right_wheel_angle = this->right_wheel_joint->GetAngle(1).Degree()+angle_y*180./PI;
      right_wheel_angular_velocity = (right_wheel_angle-pre_right_wheel_angle)/dt;
      pre_right_wheel_angle = right_wheel_angle;
      left_wheel_angle = this->left_wheel_joint->GetAngle(1).Degree()+angle_y*180./PI;
      left_wheel_angular_velocity = (left_wheel_angle-pre_left_wheel_angle)/dt;
      pre_left_wheel_angle = left_wheel_angle;
     
      //solving the torque  
//      right_wheel_Torque = -(theta_Kp*(angle_y)+theta_Kd*(angular_velocity_y)+phi_Kp*(right_wheel_angle*PI/180.)+phi_Kd*(PI/180.*right_wheel_angular_velocity));
//      left_wheel_Torque = -(theta_Kp*(angle_y)+theta_Kd*(angular_velocity_y)+phi_Kp*(left_wheel_angle*PI/180.)+phi_Kd*(PI/180.*left_wheel_angular_velocity));
       right_motor_Torque = -(theta_Kp*(angle_y)+theta_Kd*(angular_velocity_y)+phi_Kp*(right_wheel_angle*PI/180.)+phi_Kd*(PI/180.*right_wheel_angular_velocity));
       left_motor_Torque = -(theta_Kp*(angle_y)+theta_Kd*(angular_velocity_y)+phi_Kp*(left_wheel_angle*PI/180.)+phi_Kd*(PI/180.*left_wheel_angular_velocity));
       right_wheel_Torque = right_motor_Torque;
      left_wheel_Torque = left_motor_Torque+(1.0*(right_wheel_angle-left_wheel_angle)+0.05*(right_wheel_angular_velocity-left_wheel_angular_velocity));
      
      
     //Apply force to joint
      this->right_wheel_joint->SetForce(2,right_wheel_Torque);   //setForce(axis,Force value)
      this->left_wheel_joint->SetForce(2, left_wheel_Torque);
      //this->body->SetForce(1, 40);
//        this->left_wheel->SetTorque(math::Vector3(0, left_wheel_Torque, 0));
//        this->right_wheel->SetTorque(math::Vector3(0, right_wheel_Torque, 0));
      
    //setting for getting dt
      this->last_update_time = current_time;
      
    //getting readable angular_velocity data
      m_Times.data = time;
      m_right_wheel_angle.data = right_wheel_angle;
      m_left_wheel_angle.data = left_wheel_angle;
//      m_right_wheel_angular_velocity.data = right_wheel_angular_velocity;
//      m_left_wheel_angular_velocity.data = left_wheel_angular_velocity;
//      m_right_wheel_torque.data=right_wheel_Torque;
//      m_left_wheel_torque.data= left_wheel_Torque;
//      
//      P_Times.publish(m_Times);
      P_right_wheel_angle.publish(m_right_wheel_angle);
      P_left_wheel_angle.publish(m_left_wheel_angle);
//      P_right_wheel_angular_velocity.publish(m_right_wheel_angular_velocity);
//      P_left_wheel_angular_velocity.publish(m_left_wheel_angular_velocity);
//      P_right_wheel_torque.publish(m_right_wheel_torque);
//      P_left_wheel_torque.publish(m_left_wheel_torque);
      
      m_angle_y.data=angle_y;
      P_angle_y.publish(m_angle_y);
      m_world_angle_y.data=world_angle_y;
      P_world_angle_y.publish(m_world_angle_y);       
      m_angle_y_from_acc.data=angle_y_from_acc;
      P_angle_y_from_acc.publish(m_angle_y_from_acc);   
      m_angle_y_comp.data=angle_y_comp;
      P_angle_y_comp.publish(m_angle_y_comp);  
    }
  };
    GZ_REGISTER_MODEL_PLUGIN(PIDJoints);
}
