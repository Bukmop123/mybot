    #include "ros/ros.h" 
    #include "std_msgs/Bool.h" 
    #include "std_msgs/String.h" 
    #include "geometry_msgs/Twist.h" 

    #include "sensor_msgs/Joy.h"
    #include <iostream> 

	#include "mybot_msg/msgMybot_basicMovement.h"

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


   class OperatorCommands
   {
   public:

	OperatorCommands(ros::NodeHandle n ) 
	{

		inv_front_leg_ = -1;
		leg_left_front_up_vel_cmd_ = 0.0;
		leg_left_front_down_vel_cmd_ = 0.0;
		leg_left_back_up_vel_cmd_ = 0.0;
		leg_left_back_down_vel_cmd_ = 0.0;

		joystick_subscribed_ = false;


		JoystickSubscriber_ =  n.subscribe("/joy",10, &OperatorCommands::TrustJoyCallback ,this);
		BasicCmdPublisher_ = n.advertise<mybot_msg::msgMybot_basicMovement>("mybot/robot/cmdBasicMovement", 10);

	}

	
   
    void pub_function() 
    {
	
	ROS_INFO("in sub function");



	cmdBasicMovement_.mecanum_movement_mode = mecanum_movement_mode_;

	cmdBasicMovement_.robot_x = robot_x_;
    cmdBasicMovement_.robot_y = robot_y_;
    cmdBasicMovement_.robot_zrot = robot_zrot_;
    cmdBasicMovement_.left_front_leg = left_front_leg_; 	
    cmdBasicMovement_.left_back_leg = left_back_leg_;		
    cmdBasicMovement_.right_front_leg = right_front_leg_; 	
    cmdBasicMovement_.right_back_leg = right_back_leg_;		

    if(joystick_subscribed_ == true){
    	
    BasicCmdPublisher_.publish(cmdBasicMovement_);
    joystick_subscribed_ = false;
	}

    

    }

    void fakeLegVelocity ( ) 
		{ 

		double angle_vel = 1;

		if (true){


			fakeLegPosition( -1*leg_left_front_up_vel_cmd_, leg_left_back_up_vel_cmd_, angle_vel );
			fakeLegPosition( leg_left_front_down_vel_cmd_, -1*leg_left_back_down_vel_cmd_, angle_vel );
		}
	}


private:

	ros::Publisher BasicCmdPublisher_;
	ros::Subscriber JoystickSubscriber_;

	mybot_msg::msgMybot_basicMovement cmdBasicMovement_;

	bool joystick_subscribed_;

	bool mecanum_movement_mode_;

	double robot_x_;
	double robot_y_;
	double robot_zrot_;
	double left_front_leg_cmd_;
	double left_back_leg_cmd_;
	double right_front_leg_cmd_;
	double right_back_leg_cmd_;

	double leg_left_front_up_vel_cmd_;
	double leg_left_front_down_vel_cmd_;
	double leg_left_back_up_vel_cmd_;
	double leg_left_back_down_vel_cmd_;

	double left_front_leg_;
	double left_back_leg_;
	double right_front_leg_;
	double right_back_leg_;

	double inv_front_leg_;

	void fakeLegPosition( double cmd_left_front_leg, double cmd_left_back_leg, double angle_vel ) 
	{ 

		      if((cmd_left_front_leg == 1) || (cmd_left_front_leg == -1)){

		        left_front_leg_ = left_front_leg_ + angle_vel*3.14/360 * cmd_left_front_leg;

		      }
		      if((cmd_left_back_leg == 1) || (cmd_left_back_leg == -1)){

		        left_back_leg_ = left_back_leg_ + angle_vel*3.14/360 * cmd_left_back_leg;

		      }
	}


	void fakeLegPosition5( double cmd_left_front_leg, double cmd_left_back_leg ) 
	{ 
			double angle_vel = 5;

			fakeLegPosition( cmd_left_front_leg, cmd_left_back_leg, angle_vel );
	}

	void TrustJoyCallback(const sensor_msgs::Joy::ConstPtr& msg) 
    { 
    	double axes[] = {msg->axes[0],
    		msg->axes[1],
    		msg->axes[2],
    		msg->axes[3],
    		msg->axes[4],
    		msg->axes[5],
    		msg->axes[6],
    		msg->axes[7]};

    		int buttons[] = {msg->buttons[0],
			msg->buttons[1],
			msg->buttons[2],
			msg->buttons[3],
			msg->buttons[4],
			msg->buttons[5],
			msg->buttons[6],
			msg->buttons[7],
			msg->buttons[8],
			msg->buttons[9],
			msg->buttons[10],
			msg->buttons[11],
			msg->buttons[12],
			msg->buttons[13],
			msg->buttons[14]};


    robot_x_ = axes[1];
    robot_y_ = -axes[0];
    robot_zrot_ = axes[2];

    if(false){

    	left_front_leg_cmd_ = -axes[5]; 	//miguel
    	left_back_leg_cmd_ = axes[4];		//miguel
    	right_front_leg_cmd_ = -axes[5]; 	//miguel
    	right_back_leg_cmd_ = axes[4];		//miguel

    	leg_left_front_up_vel_cmd_ = buttons[5];
    	leg_left_front_down_vel_cmd_ = buttons[7];
    	leg_left_back_up_vel_cmd_ = buttons[4];
    	leg_left_back_down_vel_cmd_ = buttons[6];

    }else{
    	left_front_leg_cmd_ = inv_front_leg_*axes[7];		//viktor
    	left_back_leg_cmd_ = axes[6];		//viktor
    	right_front_leg_cmd_ = inv_front_leg_*axes[7];		//viktor
    	right_back_leg_cmd_ = axes[6];		//viktor

    	leg_left_front_up_vel_cmd_ = buttons[7];
    	leg_left_front_down_vel_cmd_ = buttons[9];
    	leg_left_back_up_vel_cmd_ = buttons[6];
    	leg_left_back_down_vel_cmd_ = buttons[8];

    }
    

    


    
    mecanum_movement_mode_ = buttons[0];

    if (buttons[1] == false){
    //if (false){

    	fakeLegPosition5( left_front_leg_cmd_, left_back_leg_cmd_ );					//miguel


    }else{
    	left_front_leg_ = 0;
    	left_back_leg_ = 0;
    	right_front_leg_ = 0;
		right_back_leg_ = 0;
    }

    joystick_subscribed_ = true;
	
    }

	
};


     
 
    int main(int argc, char** argv) 
    { 
      	ros::init(argc, argv,"operator_cmds_node"); 

 		ROS_INFO("in main");

		ros::NodeHandle n;

      	double actualTime;
      	double lastLoopTime;
     

		OperatorCommands OpCommand(n);


      	//ros::spin();
      	
      	while (ros::ok())
      	{
      	actualTime = rosTimeToDouble( ros::Time::now());

      	if((actualTime-lastLoopTime) >= 0.005 ){
      	  OpCommand.fakeLegVelocity ( );
      	  OpCommand.pub_function();
      	  lastLoopTime = actualTime;
      	}
      	  
      	  ros::spinOnce();
      	  //usleep(100);
      	  //sleep(1);

      	}
    } 
