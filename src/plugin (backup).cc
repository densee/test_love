#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
//#include <rbdl/rbdl.h>

#define PI 3.141592


namespace gazebo
{

  class PIDJoints : public ModelPlugin
  {
    double time = 0;
    double desired_time = 5; //sec
    double dt;


// ===================================Case #

// ================================ time setting

// ===================================== //
//  HYH PARA
    double KUDOS_motor1_Torque, KUDOS_motor2_Torque;
    double KUDOS_wheel1_Torque, KUDOS_wheel2_Torque;
    double KUDOS_wheel1_angle, KUDOS_wheel2_angle;
    double KUDOS_pre_wheel1_angle = 0,KUDOS_pre_wheel2_angle = 0;
    double KUDOS_wheel1_AngularVelocity, KUDOS_wheel2_AngularVelocity;
    double KUDOS_wheel1_AngularAcc, KUDOS_wheel2_AngularAcc;

    double g = 9.81;

    ////------KUWAY PARA------////
    double KUWAY_Body_Icg = 4.64;
    double KUWAY_Body_Lcg = 0.14;
    double KUWAY_Body_m = 37.65;
    double KUWAY_Wheel_m = 0.84888;
    double KUWAY_Wheel_Jm = 0.021479;
    double KUWAY_Wheel_R = 0.385/2;
    double KUWAY_theta_Kp = -80.8226906671380	;
    double KUWAY_theta_Kd = -25.2441828401950	;
    double KUWAY_phi_Kp = -3.16227766016839/5	;
    double KUWAY_phi_Kd = -4.81122004612400/2    ;
    double KUWAY_n = 13.715;
    double KUWAY_motor_Jm = 1.412*0.0001;
    double KUWAY_motor_Kf = 3.78*0.0001;
    double KUWAY_motor_Kt = 3.756*0.01;
    double KUWAY_motor_Ke = 3.745*0.01;
    double KUWAY_motor_R = 0.0788;
    ////------KUWAY PARA------////


//  HYH Pointers
    physics::LinkPtr KUDOS_body;
    physics::JointPtr KUDOS_wheel1_joint;
    physics::JointPtr KUDOS_wheel2_joint;

    math::Pose KUDOS_World_Pose;
    math::Vector3 KUDOS_World_Vector3;
    math::Quaternion KUDOS_World_Quaternion;
    double KUDOS_World_angle_Y;

//  HYH PID
    common::PID pid_KUDOS_wheel1;
    common::PID pid_KUDOS_wheel2;

//  HYH Sensor
    sensors::SensorPtr KUDOS_Sensor;
    sensors::ImuSensorPtr KUDOS_IMU;
    double KUDOS_IMU_Update;

    double KUDOS_AngularVelocity_X;
    double KUDOS_AngularVelocity_Y;
    double KUDOS_AngularVelocity_Z;

    double KUDOS_AngularAcc_X;
    double KUDOS_AngularAcc_Y;
    double KUDOS_AngularAcc_Z;

    double KUDOS_LinearAcceleration_X = 0;
    double KUDOS_LinearAcceleration_Y = 0;
    double KUDOS_LinearAcceleration_Z = 0;
    double KUDOS_LinearAcceleration_LP = 0.999;
    double KUDOS_LinearAcceleration = 0;

    double KUDOS_Angle_X_gyro = 0;
    double KUDOS_Angle_Y_gyro = 0;
    double KUDOS_Angle_Z_gyro = 0;

    double KUDOS_Angle_X_Accel = 0;
    double KUDOS_Angle_Y_Accel = 0;
    double KUDOS_m_Angle_Y_Accel = 0;


    double KUDOS_Angle_X = 0;
    double KUDOS_Angle_Y = 0;
    double KUDOS_m_Angle_Y = 0;
    double KUDOS_Angle_Z = 0;
    double KUDOS_complementary_gain = 0.999;

// ===================================== //

// ** Pointers for each joints

    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;

// ============= node handler setting ==============
    ros::NodeHandle n;

// ============= subscriber setting ==============
    ros::Subscriber SUB; // 주체 : 가제보 (ros --> gazebo)
    ros::Subscriber S_KUDOS_wheel1_P_Gain_sub;
    ros::Subscriber S_KUDOS_wheel1_I_Gain_sub;
    ros::Subscriber S_KUDOS_wheel1_D_Gain_sub;
    ros::Subscriber S_KUDOS_wheel2_P_Gain_sub;
    ros::Subscriber S_KUDOS_wheel2_I_Gain_sub;
    ros::Subscriber S_KUDOS_wheel2_D_Gain_sub;
// ============= publisher setting ==============
    ros::Publisher P_Times;
    ros::Publisher P_KUDOS_wheel1_torque;
    ros::Publisher P_KUDOS_wheel2_torque;
    ros::Publisher P_KUDOS_wheel1_angle;
    ros::Publisher P_KUDOS_wheel2_angle;

    ros::Publisher P_KUDOS_Gyro_Angle_Y;
    ros::Publisher P_KUDOS_Accel_Angle_Y;
    ros::Publisher P_KUDOS_Complementary_Angle_Y;
    ros::Publisher P_KUDOS_World_Angle_Y;
// ============= topic setting ==============
    std_msgs::Float64 m_Times;
    std_msgs::Float64 m_KUDOS_wheel1_torque;
    std_msgs::Float64 m_KUDOS_wheel2_torque;
    std_msgs::Float64 m_KUDOS_Gyro_Angle_Y;
    std_msgs::Float64 m_KUDOS_Accel_Angle_Y;
    std_msgs::Float64 m_KUDOS_Complementary_Angle_Y;
    std_msgs::Float64 m_KUDOS_World_Angle_Y;
    std_msgs::Float64 m_KUDOS_wheel1_angle;
    std_msgs::Float64 m_KUDOS_wheel2_angle;

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model_ = _model;
      // initialize a PID class
     
      printf("[!! KUWAY Robot was Loaded !!!]\n");

      int argc = 0;
      char** argv = NULL;
      ros::init(argc,argv,"PIDJoints");

      ROS_INFO("[HYH] ==== PLUGIN_LOADED ====");

//      SUB = n.subscribe("p",1,&PIDJoints::CB, this);

//      S_KUDOS_wheel1_P_Gain_sub = n.subscribe("KUDOS_wheel1_P_Gain_setup", 1, &PIDJoints::KUDOS_wheel1_P_Gain_sub_f, this);
//      S_KUDOS_wheel1_I_Gain_sub = n.subscribe("KUDOS_wheel1_I_Gain_setup", 1, &PIDJoints::KUDOS_wheel1_I_Gain_sub_f, this);
//      S_KUDOS_wheel1_D_Gain_sub = n.subscribe("KUDOS_wheel1_D_Gain_setup", 1, &PIDJoints::KUDOS_wheel1_D_Gain_sub_f, this);

//      S_KUDOS_wheel2_P_Gain_sub = n.subscribe("KUDOS_wheel2_P_Gain_setup", 1, &PIDJoints::KUDOS_wheel2_P_Gain_sub_f, this);
//      S_KUDOS_wheel2_I_Gain_sub = n.subscribe("KUDOS_wheel2_I_Gain_setup", 1, &PIDJoints::KUDOS_wheel2_I_Gain_sub_f, this);
//      S_KUDOS_wheel2_D_Gain_sub = n.subscribe("KUDOS_wheel2_D_Gain_setup", 1, &PIDJoints::KUDOS_wheel2_D_Gain_sub_f, this);

      P_Times = n.advertise<std_msgs::Float64>("times",1);
      P_KUDOS_wheel1_torque = n.advertise<std_msgs::Float64>("KUDOS_wheel1_torque",1);
      P_KUDOS_wheel2_torque = n.advertise<std_msgs::Float64>("KUDOS_wheel2_torque",1);

      P_KUDOS_Gyro_Angle_Y = n.advertise<std_msgs::Float64>("KUDOS_Gyro_Angle",1);
      P_KUDOS_Accel_Angle_Y = n.advertise<std_msgs::Float64>("KUDOS_Accel_Angle",1);
      P_KUDOS_Complementary_Angle_Y = n.advertise<std_msgs::Float64>("KUDOS_Complementary_Angle",1);
      P_KUDOS_World_Angle_Y = n.advertise<std_msgs::Float64>("KUDOS_World_Angle",1);

      P_KUDOS_wheel1_angle = n.advertise<std_msgs::Float64>("KUDOS_wheel1_angle",1);
      P_KUDOS_wheel2_angle = n.advertise<std_msgs::Float64>("KUDOS_wheel2_angle",1);


      ros::Rate loop_rate(1000); // rqt에서 쏴주는 녀석의 시간

      this->model_ = _model;


// KUDOS_body
      this->KUDOS_body = this->model_->GetLink("KUDOS_body");


// KUDOS_wheel1
      //this->pid_KUDOS_wheel1.Init(1000, 1, 10, 100, -100, 1000, -1000);
      this->KUDOS_wheel1_joint = this->model_->GetJoint("KUDOS_wheel1shaft");


// KUDOS_wheel2
      //this->pid_KUDOS_wheel2.Init(1000, 1, 10, 100, -100, 1000, -1000);
      this->KUDOS_wheel2_joint = this->model_->GetJoint("KUDOS_wheel2shaft");

// KUDOS_IMU
      this->KUDOS_Sensor = sensors::get_sensor("KUDOS_IMU");
      this->KUDOS_IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(KUDOS_Sensor);
//      this->KUDOS_IMU.Init();


      this->last_update_time_ = this->model_->GetWorld()->GetSimTime();
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PIDJoints::UpdatePID, this));

    }

// ============= Callback function setting ==============
//  public: void CB(const std_msgs::Float64 &msg)
//  {
//     printf("p = %lf\n",msg.data);
//  }

//  public: void Hip_P_Gain_sub_f(const std_msgs::Float64Ptr &msg)
//  {
//  P_ = msg->data;
//  this->pid_hip.SetPGain(P_);
//  }



    void UpdatePID()
    {
      common::Time current_time = this->model_->GetWorld()->GetSimTime();
      
      dt = current_time.Double() - this->last_update_time_.Double();

        // code!!!↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

      time = time + dt;




//↓KUDOS_KUDOS_IMU↓


      //↓gyro↓
      KUDOS_AngularAcc_X = 0;//(KUDOS_AngularVelocity_X-this->KUDOS_IMU->AngularVelocity(false)[0])/dt;
      KUDOS_AngularAcc_Y = 0;//(KUDOS_AngularVelocity_Y-this->KUDOS_IMU->AngularVelocity(false)[1])/dt;
      KUDOS_AngularAcc_Z = 0;//(KUDOS_AngularVelocity_Z-this->KUDOS_IMU->AngularVelocity(false)[2])/dt;
      KUDOS_AngularVelocity_X = this->KUDOS_IMU->AngularVelocity(false)[0];
      KUDOS_AngularVelocity_Y = this->KUDOS_IMU->AngularVelocity(false)[1];
      KUDOS_AngularVelocity_Z = this->KUDOS_IMU->AngularVelocity(false)[2];
      KUDOS_Angle_X_gyro = KUDOS_Angle_X_gyro + KUDOS_AngularVelocity_X*dt;
      KUDOS_Angle_Y_gyro = KUDOS_Angle_Y_gyro + KUDOS_AngularVelocity_Y*dt;
      KUDOS_Angle_Z_gyro = KUDOS_Angle_Z_gyro + KUDOS_AngularVelocity_Z*dt;

      //↑gyro↑

      //↓Accel↓
      KUDOS_LinearAcceleration_X = KUDOS_LinearAcceleration_LP*KUDOS_LinearAcceleration_X + (1-KUDOS_LinearAcceleration_LP)*(this->KUDOS_IMU->LinearAcceleration(false)[0]);
      KUDOS_LinearAcceleration_Y = KUDOS_LinearAcceleration_LP*KUDOS_LinearAcceleration_Y + (1-KUDOS_LinearAcceleration_LP)*(this->KUDOS_IMU->LinearAcceleration(false)[1]);
      KUDOS_LinearAcceleration_Z = KUDOS_LinearAcceleration_LP*KUDOS_LinearAcceleration_Z + (1-KUDOS_LinearAcceleration_LP)*(this->KUDOS_IMU->LinearAcceleration(false)[2]);
      KUDOS_LinearAcceleration = sqrt(pow(KUDOS_LinearAcceleration_X,2)+pow(KUDOS_LinearAcceleration_Y,2)+pow(KUDOS_LinearAcceleration_Z,2));
      KUDOS_Angle_Y_Accel = -atan(KUDOS_LinearAcceleration_X/sqrt(pow(KUDOS_LinearAcceleration_Y,2)+pow(KUDOS_LinearAcceleration_Z,2)));


      if(math::isnan(KUDOS_Angle_Y_Accel))
      {
          KUDOS_Angle_Y_Accel=KUDOS_m_Angle_Y_Accel;
      }
      KUDOS_m_Angle_Y_Accel=KUDOS_Angle_Y;
      //↑Accel↑


      //↓complementary
      KUDOS_Angle_Y =  KUDOS_complementary_gain*KUDOS_m_Angle_Y + KUDOS_AngularVelocity_Y*dt + (1-KUDOS_complementary_gain)*KUDOS_Angle_Y_Accel;

      if(math::isnan(KUDOS_Angle_Y))
      {
          KUDOS_Angle_Y=KUDOS_m_Angle_Y;
      }
      KUDOS_m_Angle_Y=KUDOS_Angle_Y;
      //↑complementary↑

      //↓Euler↓
      KUDOS_World_Quaternion = this->KUDOS_body->GetWorldPose().rot;
      KUDOS_World_angle_Y=KUDOS_World_Quaternion.GetAsEuler().y;
      //↑Euler↑

      //printf("Time = %lf KUDOS_Angle_Y_gyro = %lf KUDOS_Angle_Y_Accel = %lf KUDOS_Angle_Y = %lf KUDOS_World_angle_Y = %lf\n",time,KUDOS_Angle_Y_gyro*180./PI,KUDOS_Angle_Y_Accel*180./PI,KUDOS_Angle_Y*180./PI,KUDOS_World_angle_Y*180./PI);

      KUDOS_wheel1_angle = -this->KUDOS_wheel1_joint->GetAngle(1).Degree()+KUDOS_Angle_Y_gyro*180./PI;
      KUDOS_wheel1_AngularVelocity = (KUDOS_wheel1_angle-KUDOS_pre_wheel1_angle)/dt;
      KUDOS_pre_wheel1_angle = KUDOS_wheel1_angle;
      KUDOS_wheel2_angle = this->KUDOS_wheel2_joint->GetAngle(1).Degree()+KUDOS_Angle_Y_gyro*180./PI;
      KUDOS_wheel2_AngularVelocity = (KUDOS_wheel2_angle-KUDOS_pre_wheel2_angle)/dt;
      KUDOS_pre_wheel2_angle = KUDOS_wheel2_angle;



      KUDOS_motor1_Torque = -(KUWAY_theta_Kp*(KUDOS_Angle_Y_gyro)+KUWAY_theta_Kd*(KUDOS_AngularVelocity_Y)+KUWAY_phi_Kp*(KUDOS_wheel1_angle*PI/180.)+KUWAY_phi_Kd*(PI/180.*KUDOS_wheel1_AngularVelocity));
      KUDOS_motor2_Torque = -(KUWAY_theta_Kp*(KUDOS_Angle_Y_gyro)+KUWAY_theta_Kd*(KUDOS_AngularVelocity_Y)+KUWAY_phi_Kp*(KUDOS_wheel2_angle*PI/180.)+KUWAY_phi_Kd*(PI/180.*KUDOS_wheel2_AngularVelocity));
      KUDOS_wheel1_Torque = KUDOS_motor1_Torque;
      KUDOS_wheel2_Torque = KUDOS_motor2_Torque+(1.0*(KUDOS_wheel1_angle-KUDOS_wheel2_angle)+0.05*(KUDOS_wheel1_AngularVelocity-KUDOS_wheel2_AngularVelocity));

      //printf("Time = %lf %lf %lf\n",time,KUDOS_wheel1_angle,KUDOS_Angle_Y);
// KUDOS_wheel1
//      ref_KUDOS_wheel1_Torque = this->KUDOS_wheel1_joint->GetAngle(0).Radian() - 100*3.14;
//      this->pid_KUDOS_wheel1.Update(ref_KUDOS_wheel1_Torque, dt);
      //this->KUDOS_wheel1_joint->SetForce(1, this->pid_KUDOS_wheel1.GetCmd());
      this->KUDOS_wheel1_joint->SetForce(1, -KUDOS_wheel1_Torque);


// KUDOS_wheel2
//      ref_KUDOS_wheel2_Torque = this->KUDOS_wheel2_joint->GetAngle(0).Radian() + 100*3.14;
//      this->pid_KUDOS_wheel2.Update(ref_KUDOS_wheel2_Torque, dt);
//      this->KUDOS_wheel2_joint->SetForce(1, this->pid_KUDOS_wheel2.GetCmd());
      this->KUDOS_wheel2_joint->SetForce(1, KUDOS_wheel2_Torque);



        // code!!↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

      this->last_update_time_ = current_time;

// ================= publish data setting ==================
      m_Times.data = time;
      m_KUDOS_Gyro_Angle_Y.data = KUDOS_Angle_Y_gyro*180./PI;
      m_KUDOS_Accel_Angle_Y.data = KUDOS_Angle_Y_Accel*180./PI;
      m_KUDOS_Complementary_Angle_Y.data = KUDOS_Angle_Y*180./PI;
      m_KUDOS_World_Angle_Y.data = KUDOS_World_angle_Y*180./PI;
      m_KUDOS_wheel1_torque.data = KUDOS_wheel1_Torque;
      m_KUDOS_wheel2_torque.data = KUDOS_wheel2_Torque;
      m_KUDOS_wheel1_angle.data = KUDOS_wheel1_angle;
      m_KUDOS_wheel2_angle.data = KUDOS_wheel2_angle;



      P_Times.publish(m_Times);
      P_KUDOS_Gyro_Angle_Y.publish(m_KUDOS_Gyro_Angle_Y);
      P_KUDOS_Accel_Angle_Y.publish(m_KUDOS_Accel_Angle_Y);
      P_KUDOS_Complementary_Angle_Y.publish(m_KUDOS_Complementary_Angle_Y);
      P_KUDOS_World_Angle_Y.publish(m_KUDOS_World_Angle_Y);
      P_KUDOS_wheel1_torque.publish(m_KUDOS_wheel1_torque);
      P_KUDOS_wheel2_torque.publish(m_KUDOS_wheel2_torque);
      P_KUDOS_wheel1_angle.publish(m_KUDOS_wheel1_angle);
      P_KUDOS_wheel2_angle.publish(m_KUDOS_wheel2_angle);
// ================= publish data setting ==================

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PIDJoints);
}
