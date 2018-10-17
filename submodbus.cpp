#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include "thread"
#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include <cmath>
#include <sstream>
#include <string>
#include <thread>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include "examples_common.h"
#include "ncurses.h"


int Arr[4];
int j=0, n;
void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array);
volatile bool keepLoopGoing = true;
franka::Robot robot("192.168.10.1");
franka::Gripper gripper("192.168.10.1");






void pauser()
{
 char input;
 char resume;
 
 
   while ( keepLoopGoing )
  {
   std::cin >> input;
   if ( input == 'p' )
	{
	  keepLoopGoing=false;
	  std::cout<<"paused"<<std::endl;
	  franka::Robot stop("192.168.10.1");
	  
	  std::cout<<"press s to resume and q to quit"<<std::endl;
          while (keepLoopGoing==false)
	  {
	  std::cin.get(resume);
	  std::cin.get(resume);
	  	if (resume=='s')
	  	{
	  	keepLoopGoing=true;
		std::cout<<"n = "<<n<<std::endl;
          	std::cout<<"resumed"<<std::endl;
		
	  	}
		else if (resume=='q')
		exit(1);
		else
		std::cout<<"invalid input"<<std::endl;

		sleep(1);
	  }
	}
 	sleep(1);
   }
} 

int main(int argc, char **argv)
{
	

	ros::init(argc, argv, "submodbus");

	ros::NodeHandle nh;	

	ros::Subscriber sub = nh.subscribe("modbus_wrapper/input", 1, arrayCallback);
	


    std::thread t1(pauser);
    t1.detach();

    
    robot.automaticErrorRecovery();
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    std::array<double, 7> lower_torque_thresholds_nominal{
        {0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {100, 100, 100, 100, 100, 100}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {10, 10, 10, 10, 10, 10, 10}};
    std::array<double, 6> lower_force_thresholds_nominal{{0, 0, 0, 0, 0, 0}};
    std::array<double, 6> upper_force_thresholds_nominal{{100, 100, 100, 100, 100, 100}};
    std::array<double, 6> lower_force_thresholds_acceleration{{0, 0, 0, 0, 0, 0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{100, 100, 100, 100, 100, 100}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

for(;;)
{
robot.automaticErrorRecovery();
while(keepLoopGoing)
{
if(n==0)
{
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{-0.484015,0.800438,-0.936059,-1.81119,0.781763,2.24503,0.539185}};
    MotionGenerator motion_generator(0.1, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    
    gripper.move(80,0.1);
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    n=1;
}

if (keepLoopGoing==false)
break;

if(n==1)
{
std::array<double, 7> q_goal1 = {{-0.65487,0.983066,-0.73714,-1.70459,0.889812,2.30222,0.497469}};
    MotionGenerator motion_generator1(0.1, q_goal1);
    robot.control(motion_generator1);
    n=2;
}

if (keepLoopGoing==false)
break;

if(n==2)
{
std::array<double, 7> q_goal2 = {{-1.75543,0.578447,-0.751952,-1.89364,0.49085,2.29285,-0.450558}};
    MotionGenerator motion_generator2(0.1, q_goal2);
    robot.control(motion_generator2);
    n=3;
}

if (keepLoopGoing==false)
break;

if(n==3)
{
std::array<double, 7> q_goal3 = {{-1.78325,0.814186,-0.685022,-1.84802,0.658692,2.38455,-0.510139}};
    MotionGenerator motion_generator3(0.1, q_goal3);
    robot.control(motion_generator3);
    n=4;
}

if (keepLoopGoing==false)
break;

if(n==4)
{
std::array<double, 7> q_goal4 = {{-1.7456,0.66103,-0.723,-1.94022,0.64484,2.37832,-0.541743}};
    MotionGenerator motion_generator4(0.1, q_goal4);
    robot.control(motion_generator4);
    n=5;
}

if (keepLoopGoing==false)
break;

if(n==5)
{
std::array<double, 7> q_goal5 = {{-2.13672,-0.126494,-1.14555,-2.99343,-0.285222,2.94873,-0.658379}};
    MotionGenerator motion_generator5(0.1, q_goal5);
    robot.control(motion_generator5);
    n=6;
}

if (keepLoopGoing==false)
break;

if(n==6)
{
std::array<double, 7> q_goal6 = {{-0.892728,-0.663742,-2.8469,-1.76396,-0.205672,2.31998,0.244744}};
    MotionGenerator motion_generator6(0.1, q_goal6);
    robot.control(motion_generator6);
    n=7;
}

if (keepLoopGoing==false)
break;

if(n==7)
{
std::array<double, 7> q_goal7 = {{-0.880211,-0.800673,-2.87072,-1.75957,-0.269548,2.48467,0.244566}};
    MotionGenerator motion_generator7(0.1, q_goal7);
    robot.control(motion_generator7);
    n=8;
}

if (keepLoopGoing==false)
break;

if(n==8)
{
std::array<double, 7> q_goal8 = {{-0.880372,-0.67594,-2.84847,-1.7739,-0.22646,2.36729,0.246863}};
    MotionGenerator motion_generator8(0.1, q_goal8);
    robot.control(motion_generator8);
    n=9;
}

if (keepLoopGoing==false)
break;

if(n==9)
{
std::array<double, 7> q_goal9 = {{-0.79903,-0.431982,-2.67334,-2.1768,-0.285868,2.55447,-0.970045}};
    MotionGenerator motion_generator9(0.1, q_goal9);
    robot.control(motion_generator9);
    n=10;
}

if (keepLoopGoing==false)
break;

if(n==10)
{
std::array<double, 7> q_goal10 = {{-0.660987,-0.579634,-2.83287,-2.14394,-0.401258,2.67604,-0.866201}};
    MotionGenerator motion_generator10(0.1, q_goal10);
    robot.control(motion_generator10);
    n=11;
}

if (keepLoopGoing==false)
break;

if(n==11)
{
std::array<double, 7> q_goal11 = {{-0.659869,-0.409068,-2.81636,-2.21729,-0.324867,2.60606,-0.867153}};
    MotionGenerator motion_generator11(0.1, q_goal11);
    robot.control(motion_generator11);
    n=12;
}

if (keepLoopGoing==false)
break;

if(n==12)
{
std::array<double, 7> q_goal12 = {{-0.296816,0.23312,-2.77823,-2.88431,0.104569,2.65269,-0.868797}};
    MotionGenerator motion_generator12(0.1, q_goal12);
    robot.control(motion_generator12);
    n=13;
}

if (keepLoopGoing==false)
break;

if(n==13)
{
std::array<double, 7> q_goal13 = {{0.321051,-0.647884,-2.78902,-1.86033,-0.376204,2.46138,0.0309417}};
    MotionGenerator motion_generator13(0.1, q_goal13);
    robot.control(motion_generator13);
    n=14;
}

if (keepLoopGoing==false)
break;

if(n==14)
{
std::array<double, 7> q_goal14 = {{0.335371,-0.74015,-2.81188,-1.85797,-0.388277,2.51637,0.0695565}};
    MotionGenerator motion_generator14(0.1, q_goal14);
    robot.control(motion_generator14);
    n=15;
}

if (keepLoopGoing==false)
break;

if(n==15)
{
std::array<double, 7> q_goal15 = {{0.307171,-0.626441,-2.77347,-1.91192,-0.329173,2.45776,0.0523894}};
    MotionGenerator motion_generator15(0.1, q_goal15);
    robot.control(motion_generator15);
    n=16;
}

if (keepLoopGoing==false)
break;

if(n==16)
{
std::array<double, 7> q_goal16 = {{-0.231663,-0.148822,-2.86248,-2.91162,-0.0832889,3.02969,-0.728508}};
    MotionGenerator motion_generator16(0.1, q_goal16);
    robot.control(motion_generator16);
    n=17;
}

if (keepLoopGoing==false)
break;

if(n==17)
{
std::array<double, 7> q_goal17 = {{-0.254738,-0.263179,-2.8802,-2.90613,-0.0839693,3.15819,-0.761067}};
    MotionGenerator motion_generator17(0.1, q_goal17);
    robot.control(motion_generator17);
    n=18;
}

if (keepLoopGoing==false)
break;

if(n==18)
{
std::array<double, 7> q_goal18 = {{-0.228413,-0.0434042,-2.84811,-2.97661,-0.08223,3.0348,-0.698573}};
    MotionGenerator motion_generator18(0.1, q_goal18);
    robot.control(motion_generator18);
    n=19;
}

if (keepLoopGoing==false)
break;

if(n==19)
{
std::array<double, 7> q_goal19 = {{-0.588128,-0.454242,-2.87243,-2.16721,-0.271432,2.61108,-0.898527}};
    MotionGenerator motion_generator19(0.1, q_goal19);
    robot.control(motion_generator19);
    n=20;
}

if (keepLoopGoing==false)
break;

if(n==20)
{
std::array<double, 7> q_goal20 = {{-0.592439,-0.599771,-2.88485,-2.13586,-0.279398,2.71547,-0.896359}};
    MotionGenerator motion_generator20(0.1, q_goal20);
    robot.control(motion_generator20);
    n=21;
}

if (keepLoopGoing==false)
break;

if(n==21)
{
std::array<double, 7> q_goal21 = {{-0.573168,-0.474281,-2.88741,-2.16872,-0.283658,2.636,-0.90573}};
    MotionGenerator motion_generator21(0.1, q_goal21);
    robot.control(motion_generator21);
    n=22;
}

if (keepLoopGoing==false)
break;

if(n==22)
{
std::array<double, 7> q_goal22 = {{-0.293575,-0.0862716,-2.78051,-2.92866,0.165128,2.97188,-0.971299}};
    MotionGenerator motion_generator22(0.1, q_goal22);
    robot.control(motion_generator22);
    n=23;
}

if (keepLoopGoing==false)
break;

if(n==23)
{
std::array<double, 7> q_goal23 = {{-0.224656,-0.174576,-2.84776,-2.90112,0.256256,3.02881,-1.04728}};
    MotionGenerator motion_generator23(0.1, q_goal23);
    robot.control(motion_generator23);
    n=24;
}

if (keepLoopGoing==false)
break;

if(n==24)
{
std::array<double, 7> q_goal24 = {{-1.82323,-0.347985,-2.87294,-2.42866,-0.344155,2.71302,-0.521478}};
    MotionGenerator motion_generator24(0.1, q_goal24);
    robot.control(motion_generator24);
    n=25;
}

if (keepLoopGoing==false)
break;

if(n==25)
{
 std::array<double, 7> q_goal25 = {{-0.484015,0.800438,-0.936059,-1.81119,0.781763,2.24503,0.539185}};
    MotionGenerator motion_generator25(0.1, q_goal25);
    robot.control(motion_generator25);
    n=26;
}
}
}
   


return 0;

}
void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
	for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}

	return;
}
