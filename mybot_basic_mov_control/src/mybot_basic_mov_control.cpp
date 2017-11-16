    #include "ros/ros.h" 
    #include "std_msgs/Bool.h" 
    #include "std_msgs/String.h" 
    #include "std_msgs/Float64.h" 
    #include "geometry_msgs/Twist.h" 
    #include <unistd.h>
	//#include <cmath.h>


    #include "sensor_msgs/Joy.h"
    #include "sensor_msgs/Imu.h"
    #include "sensor_msgs/JointState.h"
    #include <iostream> 

  	//#include "mybot_msg/msgMybot_jointInterface.h"
    #include "mybot_msg/msgMybot_basicMovement.h"
    #include "mybot_msg/msgMybot_detailMovement.h"

	#include <control_toolbox/pid.h>
 

  double rosTimeToDouble(ros::Time RosTime) //ToDo implement in a library
  { 
    double a;
    double b;
    double c;
    a=RosTime.sec;
    b=RosTime.nsec;
    c=1.0*a + 1.0*b*10.0e-10;
    return c;
  } 

  class BasicMovementControl
  {

  public:

	  BasicMovementControl(){
   		ROS_INFO("in emty constructor");


   	}

	  ~BasicMovementControl(){}

	  void Reset(){}

	  void Load(ros::NodeHandle n){

		ROS_INFO("in Load");
		//init 
        x_ = 0.0; 
        y_ = 0.0; 
        zrot_ = 0.0;

        left_front_leg_ = 0;
      	left_back_leg_ = 0;
      	right_front_leg_ = 0;
      	right_back_leg_ = 0;

      	mec_left_front_ =  0;
        mec_left_back_ = 0;
        mec_right_front_ =   0;
        mec_right_back_ =  0;

        stabilisize_base_link_mode_ = true;
        stabilisize_ignore_front_ = false;
        stabilisize_ignore_back_ = true;
        last_time_ = ros::Time::now();
        stabilization_pid_.initPid(1.00, 0.30, 0.00, 0.1, -0.1);
        pos_desired1 = 0;
        pos_desired2 = 0;


        ChassisVelPublisher_ = n.advertise<geometry_msgs::Twist>("mybot/base_link_diffc/cmd_vel", 10);
        BaseWheelLeftPublisher_ = n.advertise<std_msgs::Float64>("/mybot/base_left_vc/command", 10);
        BaseWheelRichtPublisher_ = n.advertise<std_msgs::Float64>("/mybot/base_right_vc/command", 10);

        LeftFrontLegAnglePublisher_ = n.advertise<std_msgs::Float64>("/mybot/leg_left_front_pc/command", 10);
        LeftBackLegAnglePublisher_ = n.advertise<std_msgs::Float64>("/mybot/leg_left_back_pc/command", 10);
        RightFrontLegAnglePublisher_ = n.advertise<std_msgs::Float64>("/mybot/leg_right_front_pc/command", 10);
        RightBackLegAnglePublisher_ = n.advertise<std_msgs::Float64>("/mybot/leg_right_back_pc/command", 10);

        LeftFrontLegTrPublisher_ = n.advertise<std_msgs::Float64>("/mybot/tr_left_front_vc/command", 10);
        LeftBackLegTrPublisher_ = n.advertise<std_msgs::Float64>("/mybot/tr_left_back_vc/command", 10);
        RightFrontLegTrPublisher_ = n.advertise<std_msgs::Float64>("/mybot/tr_right_front_vc/command", 10);
        RightBackLegTrPublisher_ = n.advertise<std_msgs::Float64>("/mybot/tr_right_back_vc/command", 10);

        LeftFrontLegMecPublisher_ = n.advertise<std_msgs::Float64>("/mybot/mw_left_front_vc/command", 10);
        LeftBackLegMecPublisher_ = n.advertise<std_msgs::Float64>("/mybot/mw_left_back_vc/command", 10);
        RightFrontLegMecPublisher_ = n.advertise<std_msgs::Float64>("/mybot/mw_right_front_vc/command", 10);
        RightBackLegMecPublisher_ = n.advertise<std_msgs::Float64>("/mybot/mw_right_back_vc/command", 10);

        DetailMovementPublisher_ = n.advertise<mybot_msg::msgMybot_detailMovement>("mybot/detail/cmdMovement", 10);
		RobotBasicMovementSubscriber_ =  n.subscribe("mybot/robot/cmdBasicMovement",10, &BasicMovementControl::RobotcmdBasicMovementCallback ,this); 

		BaseLinkImuSubscriber_ =  n.subscribe("mybot/imu/chassis",10, &BasicMovementControl::BaseLinkImuCallback ,this); 
		JointStatesSubscriber_ =  n.subscribe("mybot/joint_states",10, &BasicMovementControl::JointStatesCallback ,this); 


	  }

    void loop()
    {
        ROS_INFO("in loop");
        //transChassisWheelSpeed();
        transLegAngle();
        wheelMovement();
        CmdDetailMovementPublish();
        
    }



  private:
  	//Modes:
  	bool leg_velocity_mode_;
    bool mecanum_movement_mode_;

    double x_; 
    double y_; 
    double zrot_;
    geometry_msgs::Twist chassisMovCmd_;

    double bw_left_;
    double bw_right_;

    double cmd_left_front_leg_;
    double cmd_left_back_leg_;
    double cmd_right_front_leg_;
    double cmd_right_back_leg_;
    double left_front_leg_;
    double left_back_leg_;
    double right_front_leg_;
    double right_back_leg_;

    double mec_left_front_;
    double mec_left_back_;
    double mec_right_front_;
    double mec_right_back_;

    double base_link_imu_acc_x_;
    double base_link_imu_acc_y_;
    double base_link_imu_acc_z_;

    double joint_leg_front_pos_;
    double joint_leg_front_vel_;
    double joint_leg_front_eff_;

    double joint_leg_back_pos_;
    double joint_leg_back_vel_;
    double joint_leg_back_eff_;

    std_msgs::Float64 fbw_left_;
    std_msgs::Float64 fbw_right_;

    std_msgs::Float64 fleft_front_leg_;
    std_msgs::Float64 fleft_back_leg_;
    std_msgs::Float64 fright_front_leg_;
    std_msgs::Float64 fright_back_leg_;

    std_msgs::Float64 fmec_left_front_;
    std_msgs::Float64 fmec_left_back_;
    std_msgs::Float64 fmec_right_front_;
    std_msgs::Float64 fmec_right_back_;

    ros::Publisher ChassisVelPublisher_;
    ros::Publisher BaseWheelLeftPublisher_;
    ros::Publisher BaseWheelRichtPublisher_;

    ros::Publisher LeftFrontLegAnglePublisher_;
    ros::Publisher LeftBackLegAnglePublisher_;
    ros::Publisher RightFrontLegAnglePublisher_;
    ros::Publisher RightBackLegAnglePublisher_;

    ros::Publisher LeftFrontLegTrPublisher_;
    ros::Publisher LeftBackLegTrPublisher_;
    ros::Publisher RightFrontLegTrPublisher_;
    ros::Publisher RightBackLegTrPublisher_;

    ros::Publisher LeftFrontLegMecPublisher_;
    ros::Publisher LeftBackLegMecPublisher_;
    ros::Publisher RightFrontLegMecPublisher_;
    ros::Publisher RightBackLegMecPublisher_;


    ros::Publisher DetailMovementPublisher_;
    ros::Subscriber RobotBasicMovementSubscriber_;

    ros::Subscriber BaseLinkImuSubscriber_;
    ros::Subscriber JointStatesSubscriber_;

    bool stabilisize_base_link_mode_;
    bool stabilisize_ignore_front_;
    bool stabilisize_ignore_back_;
    ros::Time last_time_;
    control_toolbox::Pid stabilization_pid_;
    double pos_desired1;
    double pos_desired2;

    
    //ToDo!!!
    double wheel_separation_; //ToDo in parameter?!!!

    mybot_msg::msgMybot_detailMovement cmdDetailMovement_; 
    
    

    //Subscriber / Callback Functions
    void RobotcmdBasicMovementCallback(const mybot_msg::msgMybot_basicMovement::ConstPtr& cmd_msg)
    {
      x_ = cmd_msg->robot_x ;
      y_ = cmd_msg->robot_y ;
      zrot_ = cmd_msg->robot_zrot ;


      cmd_left_front_leg_ = cmd_msg->left_front_leg;
      cmd_left_back_leg_ = cmd_msg->left_back_leg;
      cmd_right_front_leg_ = cmd_msg->right_front_leg;
      cmd_right_back_leg_ = cmd_msg->right_back_leg;

      leg_velocity_mode_ = cmd_msg->leg_velocity_mode;
      mecanum_movement_mode_ = cmd_msg->mecanum_movement_mode;

    }

    void BaseLinkImuCallback(const sensor_msgs::Imu::ConstPtr& cmd_msg)
    {
    	base_link_imu_acc_x_ = cmd_msg->linear_acceleration.x;
    	base_link_imu_acc_y_ = cmd_msg->linear_acceleration.y;
    	base_link_imu_acc_z_ = cmd_msg->linear_acceleration.z;

    }

//    void JointStatesCallback(const sensor_msgs::JointState & cmd_msg)
    void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& cmd_msg)
    {

    	if (cmd_msg->position.size() < 2 || cmd_msg->velocity.size() < 2 || cmd_msg->effort.size() < 2)
    	  {
    	   // ROS_ERROR("JointState command to PTU has wrong number of elements.");
    	    return;
    	}

    	//double* temp = cmd_msg->position;
    	//joint_leg_back_pos_ = temp;
		joint_leg_back_pos_ = cmd_msg->position[0];
    	//joint_leg_back_vel_ = cmd_msg->velocity[0];
    	//joint_leg_back_eff_ = cmd_msg->effort[0];
    	
    	joint_leg_front_pos_ = cmd_msg->position[1];
    	//joint_leg_front_vel_ = cmd_msg->velocity[1];
    	//joint_leg_front_eff_ = cmd_msg->effort[1];
		
    }


    //Process Functions
    void transChassisWheelSpeed()
    {
      double R  = 0.1; //radius of wheels
      double vr = x_;
      double va = zrot_;
      wheel_separation_ = 0.3; //ToDo in parameter!!!

      if((mecanum_movement_mode_ == false) ){

        cmdDetailMovement_.left_wheel = (vr + va * wheel_separation_ / 2.0)/R ;
        cmdDetailMovement_.right_wheel = (vr - va * wheel_separation_ / 2.0)/R;
      }

      chassisMovCmd_.linear.x = x_;
      chassisMovCmd_.angular.z = zrot_;

      ChassisVelPublisher_.publish(chassisMovCmd_);
      //ROS_INFO("BasicMovementController: obj: %i cmdWheelVelocity_.left: %f,cmdWheelVelocity_.right: %f, ", this, cmdWheelVelocity_.left,cmdWheelVelocity_.right);
      ROS_INFO("BasicMovementController: cmdDetailMovement_.left_wheel: %f,cmdDetailMovement_.right_wheel: %f, ", cmdDetailMovement_.left_wheel,cmdDetailMovement_.right_wheel);
    }


        //Publisher Functions
    void transLegAngle()
    {
      
      if((leg_velocity_mode_ == true) || (leg_velocity_mode_ == false) ){

        // do something here for differentiation of the modes.
      	left_front_leg_ = cmd_left_front_leg_;
      	left_back_leg_ = cmd_left_back_leg_;
      	right_front_leg_ = cmd_right_front_leg_;
      	right_back_leg_ = cmd_right_back_leg_;
      }

      double pos_new;
      double pos_current;
      double pos_desired;


      if(stabilisize_base_link_mode_ == true){

      	if(stabilisize_ignore_back_ == true){
      		pos_current = joint_leg_back_pos_;
      		pos_desired = (base_link_imu_acc_x_/10 + pos_desired1 + pos_desired2)/3;
      		//here reset pid;
      	}
      	else if(stabilisize_ignore_front_ == true){
      		pos_current = joint_leg_front_pos_;
      		//reset pid;
      	}

      	ros::Time time = ros::Time::now();
      	pos_new = stabilization_pid_.updatePid( pos_desired, time - last_time_);
      	last_time_ = time;

      	if(stabilisize_ignore_back_ == true){
      		left_back_leg_ = pos_new + pos_current;
      		right_back_leg_ = pos_new;
      	}
      	else if(stabilisize_ignore_front_ == true){
      		left_front_leg_ = pos_new;
      		right_front_leg_ = pos_new;
      	}
      	pos_desired1 = pos_desired; 
      	pos_desired2 = pos_desired1;

      }


      cmdDetailMovement_.base_to_left_front_leg = left_front_leg_;
      cmdDetailMovement_.base_to_left_back_leg = left_back_leg_;
      cmdDetailMovement_.base_to_right_front_leg = left_front_leg_;
      cmdDetailMovement_.base_to_right_back_leg = left_back_leg_;
//##########################
      cmdDetailMovement_.base_to_right_front_leg = base_link_imu_acc_x_;
      cmdDetailMovement_.base_to_right_back_leg = right_back_leg_;
//##########################     
      

      fleft_front_leg_.data = left_front_leg_;
      fleft_back_leg_.data = left_back_leg_;
      fright_front_leg_.data = right_front_leg_;
      fright_back_leg_.data = right_back_leg_;


      LeftFrontLegAnglePublisher_.publish(fleft_front_leg_);
      LeftBackLegAnglePublisher_.publish(fleft_back_leg_);
      RightFrontLegAnglePublisher_.publish(fright_front_leg_);
      RightBackLegAnglePublisher_.publish(fright_back_leg_);
    }

    void wheelMovement()
    {

      double fac = 1;

      if((mecanum_movement_mode_ == false) ){

        y_ = 0;
      }

      chassisMovCmd_.linear.x = x_;
      chassisMovCmd_.angular.z = zrot_;

      bw_left_ = fac * x_  + fac * zrot_;
      bw_right_ = fac * x_ -  fac * zrot_;
      mec_left_front_ =  fac * x_ -  fac * y_ + fac * zrot_;
      mec_left_back_ = fac * x_ +  fac * y_ +  fac * zrot_;
      mec_right_front_ =   fac * x_ +  fac * y_ -  fac * zrot_;
      mec_right_back_ =  fac * x_ -  fac * y_ -  fac * zrot_;


      cmdDetailMovement_.left_wheel = bw_left_;
      cmdDetailMovement_.right_wheel = bw_left_;
      cmdDetailMovement_.wheel_to_left_front_leg = mec_left_front_;
      cmdDetailMovement_.wheel_to_left_back_leg = mec_left_back_;
      cmdDetailMovement_.wheel_to_right_front_leg = mec_right_front_;
      cmdDetailMovement_.wheel_to_right_back_leg = mec_right_back_;

      fbw_left_.data = bw_left_;
      fbw_right_.data = bw_right_;
      fmec_left_front_.data = mec_left_front_;
      fmec_left_back_.data = mec_left_back_;
      fmec_right_front_.data = mec_right_front_;
      fmec_right_back_.data = mec_right_back_;

      ChassisVelPublisher_.publish(chassisMovCmd_);
      BaseWheelLeftPublisher_.publish(fbw_left_);
      BaseWheelRichtPublisher_.publish(fbw_right_);

      LeftFrontLegTrPublisher_.publish(fmec_left_front_);
      LeftBackLegTrPublisher_.publish(fmec_left_back_);
      RightFrontLegTrPublisher_.publish(fmec_right_front_);
      RightBackLegTrPublisher_.publish(fmec_right_back_);

      LeftFrontLegMecPublisher_.publish(fmec_left_front_);
      LeftBackLegMecPublisher_.publish(fmec_left_back_);
      RightFrontLegMecPublisher_.publish(fmec_right_front_);
      RightBackLegMecPublisher_.publish(fmec_right_back_);
    }


    void CmdDetailMovementPublish()
    {
      ros::Time current_time = ros::Time::now();
      cmdDetailMovement_.header.stamp = current_time;
      cmdDetailMovement_.header.frame_id = "command wheel velocity";

      DetailMovementPublisher_.publish(cmdDetailMovement_);
    }


  };


     
 
    int main(int argc, char** argv) 
    {
        ros::init(argc, argv,"basicMovementControl"); 
        double actualTime;
        double lastLoopTime;

        ROS_INFO("in main");
    
        ros::NodeHandle n;
      
        BasicMovementControl BasicMovementController;
        BasicMovementController.Load(n);

        //ros::spin(); 

      
        while (ros::ok())
        {
        actualTime = rosTimeToDouble( ros::Time::now());

        if((actualTime-lastLoopTime) >= 0.1 ){
          BasicMovementController.loop();
          lastLoopTime = actualTime;
        }
          
          ros::spinOnce();
          //usleep(100);
          //sleep(1);

        }

        ROS_INFO("in main end");

    } 