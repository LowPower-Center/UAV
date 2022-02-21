

/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2019.12.31
* Description: Autonomous circular trajectory in offboard mode

第一版由嘉创飞航修改
第二版由队长张大林修改
第三版由github作者sc修改
***************************************************************************************************************************/

#include <cstdlib>
#include <ros/ros.h>
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include "std_msgs/String.h"

using namespace std;

#define Xa      ( 2.0+0.15)
#define Ya      ( 0.0-0.03)
#define Xb      ( 0.0+0.0)
#define Yb      (-2.0+0.0)
#define Xc      ( 2.0+0.0)
#define Yc      (-2.0+0.0)
#define Xobs1     2.0
#define Yobs1    -1.0
#define Xobs2     1.0
#define Yobs2    -1.0

//const float spread_PREPARE_H = 40;//起飞速度
//const float spread_PREPARE = 500;//上升速度，横向移动速度
//const float spread_REST = 1000;//下降速度，C到H的移动速度

float desire_z = 1;                //期望高度
float desire_z_put = (0.01 - 0.16);    //投放高度
float desire_x = 0.5;              //期望高度
float desire_y = 2.3;              //飞跃障碍高度

Eigen::Vector3d temp_pos_drone;
Eigen::Vector3d temp_pos_target;

Eigen::Vector3d pos_target;//offboard模式下，发送给飞控的期望值
//float desire_z_obstacle= 2.3; //投放高度
float desire_Radius = 1;//期望圆轨迹半径
float MoveTimeCnt = 0;
float priod = 1000.0;   //减小数值可增大飞圆形的速度
bool  picture = 0;
mavros_msgs::SetMode mode_cmd;
ros::Publisher setpoint_raw_local_pub;
ros::Publisher chatter_pub;
ros::ServiceClient set_mode_client;
ros::ServiceClient client;
std_msgs::String msgpwm;


enum
{
    WAITING,		//等待offboard模式
	CHECKING,		//检查飞机状态
	PREPARE_H,		//起飞到第一个点    1m
	REST_H,			//达到H上方一稳定高度  1m
	PREPARE_A,		//飞到A点
	REST_A,			//休息一下	
	PREPARE_Ao1,    //A处升高，准备飞o1
	PREPARE_o1,     //飞至o1
    PREPARE_o1B,    //o1飞到B点
	REST_B,	
	PREPARE_Bo2,    //B处升高，准备飞o2
	PREPARE_o2,
    PREPARE_o2C,	//o2飞到C点
	REST_C,	
    PREPARE_CD,
	PREPARE_D,		//飞到C点
	REST_D,	
	LAND,		
	FLYOVER,		//结束		
}FlyState = WAITING;//初始状态WAITING

//接收来自飞控的当前飞机位置
Eigen::Vector3d pos_drone;                     
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    pos_drone = pos_drone_fcu_enu;
}

//接收来自飞控的当前飞机状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
	current_state = *msg;
}

//发送位置期望值至飞控（输入：期望xyz,期望yaw）
void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

//状态机更新
void FlyState_update(void)
{

	switch (FlyState)
	{
		case WAITING:
			if (current_state.mode != "OFFBOARD")//等待offboard模式
			{
				pos_target[0] = pos_drone[0];
				pos_target[1] = pos_drone[1];
				pos_target[2] = pos_drone[2];
				temp_pos_drone[0] = pos_drone[0];   //temp_pos_drone存储飞机从其他模式切换为OFFBOARD模式那一时刻的位置，作为自己的home位置
				temp_pos_drone[1] = pos_drone[1];
				temp_pos_drone[2] = pos_drone[2];
				send_pos_setpoint(pos_target, 0);
			}
			else
			{
				pos_target[0] = temp_pos_drone[0];  //如果程序运行时，飞机就已经被切换为了OFFBOARD模式，  temp_pos_drone可能默认存储飞机起飞点的位置。
				pos_target[1] = temp_pos_drone[1];
				pos_target[2] = temp_pos_drone[2];
				send_pos_setpoint(pos_target, 0);
				FlyState = CHECKING;
			}
			//cout << "WAITING" <<endl;
			break;
		case CHECKING:
			if (pos_drone[0] == 0 && pos_drone[1] == 0) 			//没有位置信息则执行降落模式
			{
				cout << "Check error, make sure have local location" << endl;
				mode_cmd.request.custom_mode = "AUTO.LAND";
				set_mode_client.call(mode_cmd);
				FlyState = WAITING;
			}
			else
			{
				FlyState = PREPARE_H;
				MoveTimeCnt = 0;
			}
			//cout << "CHECKING" <<endl;
			break;
		case PREPARE_H:								//起飞到H点上方desire_z高度处;
			//temp_pos_target[0] = temp_pos_drone[0] - desire_Radius;
			temp_pos_target[0] = temp_pos_drone[0];
			temp_pos_target[1] = temp_pos_drone[1];
			temp_pos_target[2] = desire_z;                    		  //设置期望高度 
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + (temp_pos_target[0] - temp_pos_drone[0]) * (MoveTimeCnt / 40);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + (temp_pos_target[1] - temp_pos_drone[1]) * (MoveTimeCnt / 40);
			pos_target[2] = temp_pos_drone[2] + (temp_pos_target[2] - temp_pos_drone[2]) * (MoveTimeCnt / 40);    //从当前高度抵达设定高度
			send_pos_setpoint(pos_target, 0);
			if (MoveTimeCnt >= 40)
			{
				FlyState = PREPARE_A;
				MoveTimeCnt = 0;
			}
		case PREPARE_A:								// 飞到A点上方desire_z高度处;
			temp_pos_target[0] = temp_pos_drone[0] + Xa;	  			 //前进2m
			temp_pos_target[1] = temp_pos_drone[1] + Ya;
			temp_pos_target[2] = desire_z;
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + (temp_pos_target[0] - temp_pos_drone[0]) * (MoveTimeCnt / 500);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + (temp_pos_target[1] - temp_pos_drone[1]) * (MoveTimeCnt / 500);
			pos_target[2] = desire_z + (temp_pos_target[2] - desire_z) * (MoveTimeCnt / 500);                                                    //!!!将上次的H处的最终高度 作为本次初始高度
			send_pos_setpoint(pos_target, 0);				//send_pos_setpoint(pos_target,  1.5708*(MoveTimeCnt/500)-1.5708);	   //从H飞往A时，扫描障碍

			if (MoveTimeCnt >= 500)
			{
				FlyState = REST_A;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;

		case REST_A:								// 飞到A点上方desire_z_put高度处,并进行投放;	
			temp_pos_target[2] = desire_z_put;
			MoveTimeCnt += 2;

			pos_target[2] = desire_z + (temp_pos_target[2] - desire_z) * (MoveTimeCnt / 800);
			send_pos_setpoint(pos_target, 0);			//A   往 B     顺时针转135度：-2.3562 *(MoveTimeCnt/500）               
			if (MoveTimeCnt >= 800)
			{

				std::stringstream ss;
				ss << "PUTA";
				msgpwm.data = ss.str();			//     ROS_INFO("%s", msg.data.c_str());
				chatter_pub.publish(msgpwm);

				FlyState = PREPARE_Ao1;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;
		case PREPARE_Ao1:								//起飞到B点上方desire_y高度处;		
			temp_pos_target[0] = temp_pos_drone[0] + Xa;
			temp_pos_target[1] = temp_pos_drone[1] + Ya;
			temp_pos_target[2] = desire_y;     //设置期望高度 
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + Xa + (temp_pos_target[0] - (temp_pos_drone[0] + Xa)) * (MoveTimeCnt / 500);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + Ya + (temp_pos_target[1] - (temp_pos_drone[1] + Ya)) * (MoveTimeCnt / 500);
			pos_target[2] = desire_z_put + (temp_pos_target[2] - desire_z_put) * (MoveTimeCnt / 500);    //从当前高度抵达设定高度
			send_pos_setpoint(pos_target, 0);
			if (MoveTimeCnt >= 500)
			{
				FlyState = PREPARE_o1;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;
		case PREPARE_o1:								// 飞到赛场中心点上方desire_z_obstacle高度处;
			temp_pos_target[0] = temp_pos_drone[0] + Xobs1;	   //前进2m
			temp_pos_target[1] = temp_pos_drone[1] + Yobs1;
			temp_pos_target[2] = desire_y;
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + Xa + (temp_pos_target[0] - (temp_pos_drone[0] + Xa)) * (MoveTimeCnt / 300);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + Ya + (temp_pos_target[1] - (temp_pos_drone[1] + Ya)) * (MoveTimeCnt / 300);
			pos_target[2] = desire_y + (temp_pos_target[2] - desire_y) * (MoveTimeCnt / 300);                              //!!!将A处投放的高度 作为本次初始高度
			send_pos_setpoint(pos_target, 0);				//send_pos_setpoint(pos_target, - 1.5708*(MoveTimeCnt/500)-1.5708);	   //从H飞往A时，右转90度扫描障碍

			if (MoveTimeCnt >= 300)
			{
				FlyState = PREPARE_o1B;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;
		case PREPARE_o1B:								// 飞到赛场中心点上方desire_z_obstacle高度处;
			temp_pos_target[0] = temp_pos_drone[0] + Xb;	   //前进2m
			temp_pos_target[1] = temp_pos_drone[1] + Yb;
			temp_pos_target[2] = desire_y;
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + Xobs1 + (temp_pos_target[0] - (temp_pos_drone[0] + Xobs1)) * (MoveTimeCnt / 300);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + Yobs1 + (temp_pos_target[1] - (temp_pos_drone[1] + Yobs1)) * (MoveTimeCnt / 300);
			pos_target[2] = desire_y + (temp_pos_target[2] - desire_y) * (MoveTimeCnt / 300);                              //!!!将A处投放的高度 作为本次初始高度
			send_pos_setpoint(pos_target, 0);				//send_pos_setpoint(pos_target, - 1.5708*(MoveTimeCnt/500)-1.5708);	   //从H飞往A时，右转90度扫描障碍

			if (MoveTimeCnt >= 300)
			{
				FlyState = REST_B;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;

		case REST_B:								// 飞到B点上方desire_z_put高度处 进行投放;	
			temp_pos_target[0] = temp_pos_drone[0] + Xb;	   //前进2m
			temp_pos_target[1] = temp_pos_drone[1] + Yb;
			temp_pos_target[2] = desire_z_put;
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + Xb + (temp_pos_target[0] - (temp_pos_drone[0] + Xb)) * (MoveTimeCnt / 1400);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + Yb + (temp_pos_target[1] - (temp_pos_drone[1] + Yb)) * (MoveTimeCnt / 1400);
			pos_target[2] = desire_y + (temp_pos_target[2] - desire_y) * (MoveTimeCnt / 1400);
			send_pos_setpoint(pos_target, 0);	//A   往 B     顺时针转135度：-2.3562 *(MoveTimeCnt/500）   
			if (MoveTimeCnt >= 1400)
			{
				std::stringstream ss;
				ss << "PUTB";
				msgpwm.data = ss.str();			//     ROS_INFO("%s", msg.data.c_str());
				chatter_pub.publish(msgpwm);

				FlyState = PREPARE_Bo2;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;
		case PREPARE_Bo2:								//起飞到B点上方desire_y高度处;
			temp_pos_target[0] = temp_pos_drone[0] + Xb;
			temp_pos_target[1] = temp_pos_drone[1] + Yb;
			temp_pos_target[2] = desire_y;                    		  //设置期望高度 
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + Xb + (temp_pos_target[0] - (temp_pos_drone[0] + Xb)) * (MoveTimeCnt / 500);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + Yb + (temp_pos_target[1] - (temp_pos_drone[1] + Yb)) * (MoveTimeCnt / 500);
			pos_target[2] = desire_z_put + (temp_pos_target[2] - desire_z_put) * (MoveTimeCnt / 500);    //从当前高度抵达设定高度
			send_pos_setpoint(pos_target, 0);
			if (MoveTimeCnt >= 500)
			{
				FlyState = PREPARE_o2;
				MoveTimeCnt = 0;
			}

			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;



		case PREPARE_o2:								// 飞到赛场中心点上方desire_z_obstacle高度处;
			temp_pos_target[0] = temp_pos_drone[0] + Xobs2;	   //前进2m
			temp_pos_target[1] = temp_pos_drone[1] + Yobs2;
			temp_pos_target[2] = desire_y;
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + Xb + (temp_pos_target[0] - (temp_pos_drone[0] + Xb)) * (MoveTimeCnt / 250);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + Yb + (temp_pos_target[1] - (temp_pos_drone[1] + Yb)) * (MoveTimeCnt / 250);
			pos_target[2] = desire_y + (temp_pos_target[2] - desire_y) * (MoveTimeCnt / 250);                              //!!!将A处投放的高度 作为本次初始高度
			send_pos_setpoint(pos_target, 0);				//send_pos_setpoint(pos_target, - 1.5708*(MoveTimeCnt/500)-1.5708);	   //从H飞往A时，右转90度扫描障碍

			if (MoveTimeCnt >= 250)
			{
				FlyState = PREPARE_o2C;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;

		case PREPARE_o2C:											//起飞到圆轨迹的第一个点, 起点在X负半轴（飞机后方向）
			temp_pos_target[0] = temp_pos_drone[0] + Xc;	   //前进2m
			temp_pos_target[1] = temp_pos_drone[1] + Yc;
			temp_pos_target[2] = desire_y;
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + Xobs2 + (temp_pos_target[0] - (temp_pos_drone[0] + Xobs2)) * (MoveTimeCnt / 250);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + Yobs2 + (temp_pos_target[1] - (temp_pos_drone[1] + Yobs2)) * (MoveTimeCnt / 250);
			pos_target[2] = desire_y + (temp_pos_target[2] - desire_y) * (MoveTimeCnt / 250);                              //!!!将A处投放的高度 作为本次初始高度
			send_pos_setpoint(pos_target, 0);				//send_pos_setpoint(pos_target, - 1.5708*(MoveTimeCnt/500)-1.5708);	   //从H飞往A时，右转90度扫描障碍


			if (MoveTimeCnt >= 250)
			{

				FlyState = REST_C;
				MoveTimeCnt = 0;
			}

			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;

		case REST_C:
			temp_pos_target[0] = temp_pos_drone[0] + Xc;
			temp_pos_target[1] = temp_pos_drone[1] + Yc;
			temp_pos_target[2] = desire_z_put;
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + Xc + (temp_pos_target[0] - (temp_pos_drone[0] + Xc)) * (MoveTimeCnt / 1400);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + Yc + (temp_pos_target[1] - (temp_pos_drone[1] + Yc)) * (MoveTimeCnt / 1400);
			pos_target[2] = desire_y + (temp_pos_target[2] - desire_y) * (MoveTimeCnt / 1400);
			send_pos_setpoint(pos_target, 0);	//A   往 B     顺时针转135度：-2.3562 *(MoveTimeCnt/500）               


			if (MoveTimeCnt >= 1400)
			{
				std::stringstream ss;
				ss << "PUTC";
				msgpwm.data = ss.str();			//     ROS_INFO("%s", msg.data.c_str());
				chatter_pub.publish(msgpwm);

				FlyState = PREPARE_CD;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;

		case PREPARE_CD:								//起飞到B点上方desire_y高度处;

			temp_pos_target[0] = temp_pos_drone[0] + Xc;
			temp_pos_target[1] = temp_pos_drone[1] + Yc;
			temp_pos_target[2] = desire_y;                    		  //设置期望高度 
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + Xc + (temp_pos_target[0] - (temp_pos_drone[0] + Xc)) * (MoveTimeCnt / 500);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + Yc + (temp_pos_target[1] - (temp_pos_drone[1] + Yc)) * (MoveTimeCnt / 500);
			pos_target[2] = desire_z_put + (temp_pos_target[2] - desire_z_put) * (MoveTimeCnt / 500);    //从当前高度抵达设定高度
			send_pos_setpoint(pos_target, 0);
			if (MoveTimeCnt >= 500)
			{
				FlyState = PREPARE_D;
				MoveTimeCnt = 0;
			}

			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;

		case PREPARE_D:							// 飞到赛场中心点上方desire_z_obstacle高度处;
			temp_pos_target[0] = temp_pos_drone[0];	   //前进2m
			temp_pos_target[1] = temp_pos_drone[1];
			temp_pos_target[2] = desire_y;
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + Xc + (temp_pos_target[0] - (temp_pos_drone[0] + Xc)) * (MoveTimeCnt / 700);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + Yc + (temp_pos_target[1] - (temp_pos_drone[1] + Yc)) * (MoveTimeCnt / 700);
			pos_target[2] = desire_y + (temp_pos_target[2] - desire_y) * (MoveTimeCnt / 700);                              //!!!将A处投放的高度 作为本次初始高度
			send_pos_setpoint(pos_target, 0);				//send_pos_setpoint(pos_target, - 1.5708*(MoveTimeCnt/500)-1.5708);	   //从H飞往A时，右转90度扫描障碍

			if (MoveTimeCnt >= 700)
			{
				FlyState = REST_D;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;

		case REST_D:
			temp_pos_target[0] = temp_pos_drone[0];
			temp_pos_target[1] = temp_pos_drone[1];
			temp_pos_target[2] = desire_x;
			MoveTimeCnt += 2;

			pos_target[0] = temp_pos_drone[0] + (temp_pos_target[0] - temp_pos_drone[0]) * (MoveTimeCnt / 500);    //离散当前点到目标点
			pos_target[1] = temp_pos_drone[1] + (temp_pos_target[1] - temp_pos_drone[1]) * (MoveTimeCnt / 500);
			pos_target[2] = desire_y + (temp_pos_target[2] - desire_y) * (MoveTimeCnt / 500);
			send_pos_setpoint(pos_target, 0);	//A   往 B     顺时针转135度：-2.3562 *(MoveTimeCnt/500）               
			if (MoveTimeCnt >= 500)
			{
				FlyState = LAND;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;

		case LAND:
			temp_pos_target[2] = 0;                                               //目的高度降为0
			MoveTimeCnt += 1;
			pos_target[2] = desire_x + (temp_pos_target[2] - desire_x) * (MoveTimeCnt / 72);
			send_pos_setpoint(pos_target, 0);	//A   往 B     顺时针转135度：-2.3562 *(MoveTimeCnt/500）               
			if (MoveTimeCnt >= 72)
			{
				FlyState = FLYOVER;
				MoveTimeCnt = 0;
			}
			if (current_state.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			//cout << "PREPARE" <<endl;
			break;

			/* 			case FLY:
						{
							float phase = 3.1415926;						//起点在X负半轴
							float Omega = 2.0*3.14159*MoveTimeCnt / priod;	//0~2pi
							MoveTimeCnt += 3;								//调此数值可改变飞圆形的速度
							if(MoveTimeCnt >=priod)							//走一个圆形周期
							{
								FlyState = FLYOVER;
							}
							//pos_target[0] = temp_pos_drone[0]+desire_Radius*cos(Omega+phase);
							// pos_target[1] = temp_pos_drone[1]+desire_Radius*sin(Omega+phase);
							pos_target[0] = temp_pos_drone[0]+desire_Radius*cos(Omega+phase);
							pos_target[1] = temp_pos_drone[1]+desire_Radius*sin(Omega+phase);
							pos_target[2] = desire_z;
							send_pos_setpoint(pos_target, 0);
							if(current_state.mode != "OFFBOARD")			//如果在飞圆形中途中切换到onboard，则跳到WAITING
							{
								FlyState = WAITING;
							}
						}
						//cout << "FLY" <<endl;
						break; */
		case FLYOVER:
		{
			mode_cmd.request.custom_mode = "AUTO.LAND";    // 模式汇总mt: http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack
			set_mode_client.call(mode_cmd);
			FlyState = WAITING;
		}
		//                        cout << "FLYOVER" <<endl;
		break;

		default:
			cout << "error" << endl;
	}
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular_offboard");
    ros::NodeHandle nh("~");
    // 频率 [20Hz]
    ros::Rate rate(20.0);
    ros::Rate delay(0.1);

    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
 
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);


    //ROS_INFO("1");
    // 【服务】修改系统模式
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    nh.param<float>("desire_z", desire_z, 1.0);
    nh.param<float>("desire_Radius", desire_Radius, 1.0);
     delay.sleep(); //延时10s
     delay.sleep(); //延时10s
     ROS_INFO("Put ready...");
    while(ros::ok())
    {

	FlyState_update();
 	ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


