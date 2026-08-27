#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <iostream>
#include <fstream>
#include "ins/ASENSING.h"
#include "ivsensorgps.h"
#include "ivsensorodom.h"
#include "ivsensorimu.h"
#include "robot/can_msg.h"
#include "frame_transform.h"
#include "ivmsglocpos.h"
using namespace std;
double vehicleSpeed;
void CanbusCallBack(const robot::can_msg::ConstPtr &msg){
	// rcv_flag = true;
	vehicleSpeed = msg->vehicleSpeed;
}
int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  std::string frame_id;
  bool received_message = false;
  ros::init(argc, argv, "INS");
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
 
  ros::NodeHandle nh;//("INS");
  ros::Subscriber can_bus_sub = nh.subscribe("can_msg",10,CanbusCallBack);
  ros::Publisher INS_pub = nh.advertise<ins::ASENSING>("ASENSING_INS", 500);
  ros::Publisher yw_gps_pub_ = nh.advertise<ivlocmsg::ivsensorgps>("/ivsensorgps", 1);
  ros::Publisher yw_imu_pub_ = nh.advertise<ivlocmsg::ivsensorimu>("/ivsensorimu", 1);
  ros::Publisher yw_odom_pub_ = nh.advertise<ivlocmsg::ivsensorodom>("/ivsensorodom", 1);
  ros::Publisher jjcc_gps_pub_ = nh.advertise<ivlocmsg::ivmsglocpos>("/localization", 1);

  ros::Rate r(100); 

  std::string input;
  std::string read;
  double long i=0;
  double gpswf;
  float latstd,lonstd,hstd,vnstd,vestd,vdstd,pitchstd,yawstd,rollstd,temperature,wheel_speed_status,numsv,position_type,heading_type;
  int Length=58;
 bool update = false;
 //int pos_type ;
 static int pos_type = 0;
 static int dir_type = 0;


  bool first_record_ = true;



   double last_x_,last_y_;

  double delta_x =0;
  double delta_y = 0;

  double last_x = 0;
  double  last_y = 0;
  while(ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        if(ser.available())
        {
          read = ser.read(ser.available());
          ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
          input += read;
          while (input.size() >= 3)
          {
              if  (((input[0] & 0xff) == 0xbd)&&((input[1] & 0xff) == 0xdb)&&((input[2] & 0xff) == 0x0b))
              {
                if (input.size() < 63) 
                {
                    break;
                }   
                char xorcheck58=0;
                for(int i=0;i<Length-1;i++)  
                {
                xorcheck58=xorcheck58^input[i];
                }
                if (input[Length-1]==xorcheck58 ) 
                {
                    // get RPY
                    short int roll = ((0xff &(char)input[4]) << 8) | (0xff &(char)input[3]);
                    short int pitch = ((0xff &(char)input[6]) << 8) | (0xff &(char)input[5]);
                    short int yaw = ((0xff &(char)input[8]) << 8) | (0xff &(char)input[7]);
                    // calculate RPY in deg
                    short int * temp = (short int*)& roll;
                    float rollf = (*temp)*(360.0/32768);
                    temp = (short int*)& pitch;
                    float pitchf = (*temp)*(360.0/32768);
                    temp = (short int*)& yaw;
                    float yawf = (*temp)*(360.0/32768);
                    
                    // get gyro values
                    short int gx = ((0xff &(char)input[10]) << 8) | (0xff &(char)input[9]);
                    short int gy = ((0xff &(char)input[12]) << 8) | (0xff &(char)input[11]);
                    short int gz = ((0xff &(char)input[14]) << 8) | (0xff &(char)input[13]);
                    // calculate gyro in deg/s
                    temp = (short int*)& gx;
                    float gxf = (*temp)*300.0/32768;
                    temp = (short int*)& gy;
                    float gyf = (*temp)*300.0/32768;
                    temp = (short int*)& gz;
                    float gzf = (*temp)*300.0/32768;
                
                    // get acelerometer values
                    short int ax = ((0xff &(char)input[16]) << 8) | (0xff &(char)input[15]);
                    short int ay = ((0xff &(char)input[18]) << 8) | (0xff &(char)input[17]);
                    short int az = ((0xff &(char)input[20]) << 8) | (0xff &(char)input[19]);   
                    // calculate acelerometer in g  
                    temp = (short int*)& ax;
                    float axf = *temp*12.0/32768;
                    temp = (short int*)& ay;
                    float ayf = *temp*12.0/32768;
                    temp = (short int*)& az;
                    float azf = *temp*12.0/32768;
                 
                    
                    // get gps values
                    int latitude = (((0xff &(char)input[24]) << 24) |((0xff &(char)input[23]) << 16) |((0xff &(char)input[22]) << 8) | 0xff &(char)input[21]);
                    int longitude = (((0xff &(char)input[28]) << 24) |((0xff &(char)input[27]) << 16) |((0xff &(char)input[26]) << 8) | 0xff &(char)input[25]);
                    int altitude = (((0xff &(char)input[32]) << 24) |((0xff &(char)input[31]) << 16) |((0xff &(char)input[30]) << 8) | 0xff &(char)input[29]);
                    // calculate lat、lon in deg(WGS84)
                    int* tempA = (int*)& latitude;
                    double latitudef = *tempA*1e-7L;
                    tempA = ( int*)& longitude;
                    double longitudef = *tempA*1e-7L;
                    // calculate lat、lon in m
                    tempA = ( int*)& altitude;
                    double altitudef = *tempA*1e-3L;


                    // get  NED vel values
                    short int Nvel = ((0xff &(char)input[34]) << 8) | (0xff &(char)input[33]);
                    short int Evel = ((0xff &(char)input[36]) << 8) | (0xff &(char)input[35]);
                    short int Dvel = ((0xff &(char)input[38]) << 8) | (0xff &(char)input[37]);
                    // calculate NED vel in m/s
                    temp = (short int*)& Nvel;
                    float Nvelf = (*temp)*(100.0/32768);
                    temp = (short int*)& Evel;
                    float Evelf = (*temp)*(100.0/32768);
                    temp = (short int*)& Dvel;
                    float Dvelf = (*temp)*(100.0/32768);

                
                    // ins_status values
                    short int ins_status = (0x0f &(char)input[39]);
                    temp = (short int*)& ins_status;
                    double ins_statusf = *temp*1;


                    // pdata_type
                    short int pdata_type = (0xff &(char)input[56]);

                    // data 1-3
                    short int data1 = ((0xff &(char)input[47]) << 8) | (0xff &(char)input[46]);
                    temp = (short int*)& data1;
                    float data1f = (*temp)*1;
                    short int data2 = ((0xff &(char)input[49]) << 8) | (0xff &(char)input[48]);
                    temp = (short int*)& data2;
                    float data2f = (*temp)*1;
                    short int data3 = ((0xff &(char)input[51]) << 8) | (0xff &(char)input[50]);
                    temp = (short int*)& data3;
                    float data3f = (*temp)*1;

                    switch (pdata_type)
                    {
                    case 0:
                    latstd = exp(data1f/100);
                    lonstd = exp(data2f/100);
                    hstd = exp(data3f/100);
                    break;

                    case 1:
                    vnstd = exp(data1f/100);
                    vestd = exp(data2f/100);
                    vdstd = exp(data3f/100);
                    break;

                    case 2:
                    rollstd = exp(data1f/100);
                    pitchstd = exp(data2f/100);
                    yawstd = exp(data3f/100);
                    break;

                    case 22:
                    temperature = (data1f)*200.0/32768;
                    break;

                    case 32:
                    position_type = data1f;
                    numsv = data2f;
                    heading_type = data3f;
                    if(data1f == 0) {pos_type = 0;}
                if(data1f == 16) {pos_type = 1;}
                if(data1f == 17) {pos_type = 2;}
                if(data1f == 50) {pos_type = 4;}
                if(data1f == 32 && data1f == 33 && data1f == 34) {pos_type = 5;}

                if(data3f == 0) {dir_type = 0;}
                if(data3f == 16) {dir_type = 1;}
                if(data3f == 17) {dir_type = 2;}
                if(data3f == 50) {dir_type = 4;}
                if(data3f == 32 && data3f == 33 && data3f == 34) {dir_type = 5;}
                    break;

                    case 33:
                    wheel_speed_status = data2f;
                    break;
                    }

                    // get gps time values
                    uint32_t gpst = (((0xff &(char)input[55]) << 24) |((0xff &(char)input[54]) << 16) |((0xff &(char)input[53]) << 8) | 0xff &(char)input[52]);
                    // calculate gps time in ms
                    double gpstf = gpst*0.25;
                    double gpss = gpst/4000.0;
                    
                    
                    char xorcheck63=0;
                    for(int i=0;i<62;i++)  
                    {
                    xorcheck63=xorcheck63^input[i];
                    }
                    if (input[62]==xorcheck63 ) 
                    {
                        // get gps week values
                        short int gpsw = (((0xff &(char)input[61]) << 24) |((0xff &(char)input[60]) << 16) |((0xff &(char)input[59]) << 8) | 0xff &(char)input[58]);
                        // calculate gps week
                        temp = (short int*)& gpsw;
                        gpswf = *temp*1;
                    }
                    received_message = true;
                    ins::ASENSING Asensing_msg;
                    Asensing_msg.latitude = latitudef;
                    Asensing_msg.longitude = longitudef;
                    Asensing_msg.altitude = altitudef;

                    Asensing_msg.north_velocity = Nvelf;
                    Asensing_msg.east_velocity = Evelf;
                    Asensing_msg.ground_velocity = Dvelf;

                    Asensing_msg.roll = rollf;
                    Asensing_msg.pitch = pitchf;
                    Asensing_msg.azimuth = yawf;

                    Asensing_msg.x_angular_velocity = gxf;
                    Asensing_msg.y_angular_velocity = gyf;
                    Asensing_msg.z_angular_velocity = gzf;

                    Asensing_msg.x_acc = axf;
                    Asensing_msg.y_acc = ayf;
                    Asensing_msg.z_acc = azf;

                    Asensing_msg.latitude_std = latstd;
                    Asensing_msg.longitude_std = lonstd;
                    Asensing_msg.altitude_std = hstd;

                    Asensing_msg.north_velocity_std = vnstd;
                    Asensing_msg.east_velocity_std = vestd;
                    Asensing_msg.ground_velocity_std = vdstd;

                    Asensing_msg.roll_std = rollstd;
                    Asensing_msg.pitch_std = pitchstd;
                    Asensing_msg.azimuth_std = yawstd;

                    Asensing_msg.sec_of_week = gpstf;
                    Asensing_msg.gps_week_number = gpswf;
                    
                    Asensing_msg.temperature = temperature;
                    Asensing_msg.wheel_speed_status = wheel_speed_status;
                    Asensing_msg.ins_status = ins_statusf;
                    Asensing_msg.position_type = position_type;
                    Asensing_msg.heading_type = heading_type;
                    Asensing_msg.numsv = numsv;
                  
                    
                    INS_pub.publish(Asensing_msg); 

    ivlocmsg::ivsensorimu yw_imu;
    yw_imu.header.stamp =  ros::Time::now();
    yw_imu.header.frame_id = "imu";
    yw_imu.update = true;
    yw_imu.gyro_x = gxf;
    yw_imu.gyro_y = gyf;
    yw_imu.gyro_z = gzf;

    yw_imu.acce_x = axf;
    yw_imu.acce_y =ayf;
    yw_imu.acce_z =azf;
    yw_imu_pub_.publish(yw_imu);
ivlocmsg::ivsensorgps yw_gps;
    //yw_gps.header.frame_id = ins.header.frame_id;
    yw_gps.header.stamp = ros::Time::now();

    yw_gps.lat = latitudef;
    yw_gps.lon = longitudef;
    yw_gps.height = altitudef;
    yw_gps.heading = yawf;

    yw_gps.utctime = gpss;
    yw_gps.status = pos_type;
    yw_gps.status_yaw = dir_type;
    

    yw_gps.heading_std = yawstd;

    yw_gps.velocity = sqrt(Nvelf * Nvelf + Evelf * Evelf);
    yw_gps.satenum = numsv;
    yw_gps.track_angle = 0;
    yw_gps.hdop = 1;
    yw_gps.diff_age = 0;
    yw_gps.base_length = 1;
    yw_gps.is_heading_valid = 1;
    update = true;
    yw_gps.update = update;
    yw_gps_pub_.publish(yw_gps);



      ivlocmsg::ivsensorodom odom;

      odom.header.stamp = ros::Time::now();
      odom.velocity_rl = vehicleSpeed;
      odom.velocity_rr = vehicleSpeed;
      odom.velocity = (odom.velocity_rl +odom.velocity_rr) / 2;
        // odom.velocity_rl = sqrt(Nvelf * Nvelf + Evelf * Evelf);
        // odom.velocity_rr =sqrt(Nvelf * Nvelf + Evelf * Evelf);

        // odom.velocity = (odom.velocity_rl +odom.velocity_rr) / 2;				
        // if (vel_wheel.Gear_fd == 1)
        // {
        // odom.velocity_rl = -vel_wheel.left_velocity / 3.6;
        // odom.velocity_rr = -vel_wheel.right_velocity / 3.6;			
        // odom.velocity = (odom.velocity_rl +odom.velocity_rr) / 2;			
        // }

         yw_odom_pub_.publish(odom);


ivlocmsg::ivmsglocpos jjcc_gps;

UTMCoor xy;
LatlonToUtmXY(longitudef*3.141592654/180,latitudef*3.141592654/180,&xy);



jjcc_gps.lat= latitudef;
jjcc_gps.lon=longitudef;
jjcc_gps.height=altitudef;
jjcc_gps.velocity=sqrt(Nvelf * Nvelf + Evelf * Evelf);
jjcc_gps.heading=yawf;
jjcc_gps.xg=xy.x-384337.16989732475;
jjcc_gps.yg=xy.y -3449370.17366875;
std::cout<<jjcc_gps.xg<<std::endl;
std::cout<<jjcc_gps.yg<<std::endl;
jjcc_gps.zg=altitudef - 3.539;
double    an1 = -jjcc_gps.heading+90;
  if (an1 < -180) an1 += 360;
  if (an1 > 180) an1 -= 360;
jjcc_gps.angle =an1;
jjcc_gps.localization_mode = 1;
if(pos_type == 4){
jjcc_gps.localization_status = 0;}
else{jjcc_gps.localization_status = 2;};
jjcc_gps.header.stamp = ros::Time::now();


    if (1){
    
    std::ofstream file_;
    file_.open("/home/nvidia/qingwei-L4-No2/path.csv",std::ios::app);
    double x = jjcc_gps.xg;
    double y = jjcc_gps.yg;
    float head_temp = jjcc_gps.heading;
    if(head_temp < 0)
		  head_temp += 360;
    if(first_record_){
        last_x_ = x;
        last_y_ = y;
        first_record_ = false;
        file_<<std::fixed<<std::setprecision(3)<<x<<","<<y<<","<<head_temp<<"\n";
    }else{
        double dx = x - last_x_;
        double dy = y - last_y_;
        double d = sqrt(dx*dx + dy*dy);
        // std::cout<<"d ="<<d<<std::endl;
        if(d> 0.2){
        file_<<std::fixed<<std::setprecision(3)<<x<<","<<y<<","<<head_temp<<"\n";
        last_x_ = x;
        last_y_ = y;
        }    
    }
    file_.close();

    }



jjcc_gps_pub_.publish(jjcc_gps);




                }
                input.erase(0, Length); 
            }
             else 
            {
                input.erase(0,1);
            }
          }
        }
      }


      else
      {
        try
        {
          ser.setPort(port);
          ser.setBaudrate(230400);//115200  230400
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if(ser.isOpen())
        {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
        }
      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }
}
















