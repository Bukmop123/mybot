    #include "ros/ros.h" 
    #include "std_msgs/Bool.h" 
    #include "std_msgs/String.h" 
    #include "geometry_msgs/Twist.h" 

    #include "sensor_msgs/Joy.h"
    #include <iostream> 

	#include "mybot_msg/msgMybot_basicMovement.h"

   class OperatorCommands
   {
   private:

	//std_msgs::String str_tmp_[] ={"front", "back", "right", "left", "stop"};

	std_msgs::String str_;
	geometry_msgs::Twist robot_move_order_;
	std_msgs::String str_mot1_;
	std_msgs::String str_mot2_;

	ros::Publisher pub_str_;
	ros::Publisher pub_twist_;
	ros::Publisher pub_mot1_;
	ros::Publisher pub_mot2_;
	ros::Publisher pub_trustCmd_;

	double left_front_leg_;
	double left_back_leg_;




	void fakeLegPosition( double cmd_left_front_leg, double cmd_left_back_leg ) 
	{ 
		
		fakeLegPosition( double cmd_left_front_leg, double cmd_left_back_leg ) 

	}

	void fakeLegPosition( double cmd_left_front_leg, double cmd_left_back_leg ) 
	{ 
			double angle_vel = 5;

		      if((cmd_left_front_leg == 1) || (cmd_left_front_leg == -1)){

		        left_front_leg_ = left_front_leg_ + angle_vel*3.14/360 * cmd_left_front_leg;

		      }
		      if((cmd_left_back_leg == 1) || (cmd_left_back_leg == -1)){

		        left_back_leg_ = left_back_leg_ + angle_vel*3.14/360 * cmd_left_back_leg;

		      }
	}

	

   public:
	OperatorCommands(const ros::Publisher pub_str, const ros::Publisher pub_twist, const ros::Publisher pub_mot1, const ros::Publisher pub_mot2, const ros::Publisher pub_trustCmd ) 
	{

	pub_str_ = pub_str;
	pub_twist_ = pub_twist;	
	pub_mot1_ = pub_mot1;
	pub_mot2_ = pub_mot2;
	pub_trustCmd_ = pub_trustCmd;

	left_front_leg_ = 0;
	left_back_leg_ = 0;

	}
	OperatorCommands(const ros::Publisher pub_twist, const ros::Publisher pub_trustCmd ) 
	{
		pub_twist_ = pub_twist;	
		pub_trustCmd_ = pub_trustCmd;
	}

	
    void MiguelJoyCallback(const sensor_msgs::Joy::ConstPtr& msg) 
    { 
	double axes[] = {msg->axes[0],msg->axes[1]};
	int buttons[] = {msg->buttons[0],
			msg->buttons[1],
			msg->buttons[2],
			msg->buttons[3],
			msg->buttons[4],
			msg->buttons[5],
			msg->buttons[6],
			msg->buttons[7],
			msg->buttons[8],
			msg->buttons[9]};
			
	
 	ROS_INFO("X: %f,Y: %f, ", axes[0],axes[1]);
  	ROS_INFO("B0: %d,B1: %d,B2: %d,B3: %d,B4: %d,B5: %d,B6: %d,B7: %d,B8: %d,B9: %d ",buttons[0],buttons[1],buttons[2],buttons[3],buttons[4],buttons[5],buttons[6],buttons[7],buttons[8],buttons[9]);
 
//	str_.data = "pub in jCB";


	robot_move_order_.linear.x =axes[1] * 0.4;
	robot_move_order_.angular.z =axes[0] * 0.2;


	//publish
	//pub_str_.publish(str_);
	pub_twist_.publish(robot_move_order_);
	//pub_mot1_.publish(str_mot1_);	
	//pub_mot2_.publish(str_mot2_);
	
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

	mybot_msg::msgMybot_basicMovement cmdBasicMovement;





    cmdBasicMovement.robot_x = axes[1];
    cmdBasicMovement.robot_y = -axes[0];
    cmdBasicMovement.robot_zrot = axes[2];
    cmdBasicMovement.left_front_leg = -axes[5]; 	//miguel
    cmdBasicMovement.left_back_leg = axes[4];		//miguel
    cmdBasicMovement.right_front_leg = -axes[5]; 	//miguel
    cmdBasicMovement.right_back_leg = axes[4];		//miguel

    //cmdBasicMovement.left_front_leg = axes[7];		//viktor
    //cmdBasicMovement.left_back_leg = axes[6];		//viktor
    //cmdBasicMovement.right_front_leg = axes[7];		//viktor
    //cmdBasicMovement.right_back_leg = axes[6];		//viktor


    
    cmdBasicMovement.mecanum_movement_mode = buttons[0];

    if (buttons[1] == false){
    //if (false){

    	fakeLegPosition( -axes[5], axes[4] );					//miguel
    	//fakeLegPosition( -axes[7], axes[6] );					//viktor
    	cmdBasicMovement.left_front_leg = left_front_leg_;		
    	cmdBasicMovement.left_back_leg = left_back_leg_;

    }else{
    	left_front_leg_ = 0;
    	left_back_leg_ = 0;
    }

    pub_trustCmd_.publish(cmdBasicMovement);
	
    }

//here goes the code for the publisher
//    void pub_function(const sensor_msgs::Joy::ConstPtr& msg) 
    void pub_function() 
    {
	
	ROS_INFO("in sub function");

    }

	
};


     
 
    int main(int argc, char** argv) 
    { 
      ros::init(argc, argv,"operator_cmds_node"); 

 	ROS_INFO("in main");

      ros::NodeHandle n;
     
      //Publishers init
	//ros::Publisher pub_str = n.advertise<std_msgs::String>("op_commands", 10);
	//ros::Publisher pub_mot1 = n.advertise<std_msgs::String>("iArd_mot_1", 10);
	//ros::Publisher pub_mot2 = n.advertise<std_msgs::String>("iArd_mot_2", 10);
	ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::Publisher pub_trustCmd = n.advertise<mybot_msg::msgMybot_basicMovement>("mybot/robot/cmdBasicMovement", 10);


	//OperatorCommands OpCommand(pub_str, pub_twist, pub_mot1, pub_mot2, pub_trustCmd);
	OperatorCommands OpCommand(pub_twist,pub_trustCmd);

      //Subscribers init
	OpCommand.pub_function();
	ros::Subscriber sub =  n.subscribe("/human_input/joy",10, &OperatorCommands::MiguelJoyCallback ,&OpCommand); 
	ros::Subscriber subtrust =  n.subscribe("/human_input/joy",10, &OperatorCommands::TrustJoyCallback ,&OpCommand); 

      ros::spin(); 
 
    } 
