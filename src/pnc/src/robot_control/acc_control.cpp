/**
 * @file  : 
 * @brief : 
 * @author: 
 * @date  : 
 */

#include "control_comply.h"

double ControlComply::LongitudinalFeedforwardControl(robot::acc &pub)
{
    //--objects filter--
    robot::perception objects;

    for(auto i : LidarObject.objs)  {
        bool l = 0;
        bool w = 0;
	bool d = 0;

	double path_dist = 100.0;

        for(auto j : mPathList)  {
            double pdx = i.x - j.x_axis;
            double pdy = i.y - j.y_axis;
            double pdist = hypot(pdx, pdy);

            if(pdist < path_dist) path_dist = pdist;
        }

	if(path_dist < 3.0) d = 1;
        if(i.dx > 0.0 && i.dx < 5.0) l = 1;
        if(i.dy > 1.0 && i.dy < 5.0) w = 1;

        if(l == 1 && w == 1 && d == 1) objects.objs.push_back(i);
    }

    //--search nearest object ahead--
    robot::object object;
    bool object_detect = false;
    double distance = 100.0;

    pub.object = objects.objs.size();

    if(objects.objs.size() > 0)  {
        object_detect = true;

        for(auto i : objects.objs)  {
            double path_dist = 100.0;

            for(auto j : mPathList)  {
                double pdx = i.x - j.x_axis;
                double pdy = i.y - j.y_axis;
                double pdist = hypot(pdx, pdy);    

                if(pdist < path_dist) path_dist = pdist;
            }

	    if(path_dist < distance) object = i;
        }

	double dx = object.x - mNavData.xAxis;
	double dy = object.y - mNavData.yAxis;

        distance = hypot(dx, dy);

	pub.active = 1;
    }else  {
        object_detect = false;
	pub.active = 0;
    }

    //--ACC calculate--
    static double Aaeb = 0.0;
    double safe_inter_vehicle_distance = 15.0;
    double time_gap = 0.5;
    double D0 = safe_inter_vehicle_distance;
    double Tg = time_gap;
    double Vf = mNavData.gpsSpeed;
    double Ddes = Tg * Vf + D0;
    double Ades = 0.0;
    double Aout = 0.0;;
    double A = 0.5;
    double B = 1.0;
    double C = 0.5;

    if(object_detect)  {
        double D = distance;
        double derta_v = Vf - hypot(object.vx, object.vy);
        double derta_d = D - Ddes;
        
        Ades = A * derta_v + B * derta_d;

        if(Ades > 1.0) Ades = 1.0;
        if(Ades <-1.0) Ades =-1.0;

        double time = ros::Time::now().toSec();
        //double time = current_gps_time;
        double time_err = time - LidarObject.header.stamp.toSec();
    }else  {
        Ades = 100.0;
        Aaeb = 100.0;
    }

    //--AEB calculate--
    if(object_detect)  {
        if(distance < 10.0 && mNavData.gpsSpeed < 4.0)  {
            Aaeb -= 0.1;
        }else  {
            Aaeb = Ades;
        }

        if(Aaeb < -5.0) Aaeb = -5.0; 
    }

    double tarspd = mControlData.desireSpeed;
    
    tarspd = tarspd > 1.0 ? 1.0 : tarspd;

    //--target speed from trajectory--
    Aout = C * (mControlData.desireSpeed - mNavData.gpsSpeed);

    Aout = Aout > Aaeb ? Aaeb : Aout;    

    pub.ds = distance;
    pub.dv = Vf - hypot(object.vx, object.vy);
    pub.tarspd = tarspd;
    pub.curspd = mNavData.gpsSpeed;
    pub.objspd = hypot(object.vx, object.vy);
    pub.Ades = Ades;
    pub.Av = A * (Vf - hypot(object.vx, object.vy));
    pub.Bd = B * (distance - Ddes);
    pub.Aaeb = Aaeb;
    pub.Aout = Aout;

    return Aout;
}

double ControlComply::LongitudinalFeedbackControl()
{
    double aim_acc = 0.0;
/* 
    if(current_pitch >= 0.0)  {
        aim_acc = 5.0 * sin(current_pitch * M_PI/180.0);
    }else  {
        aim_acc = 3.0 * sin(current_pitch * M_PI/180.0);
    }
*/
    return aim_acc;
}

double ControlComply::LongitudinalControlOutput(robot::acc &pub)
{
    double a_feedforward_control = LongitudinalFeedforwardControl(pub);
    double a_feedback_control = LongitudinalFeedbackControl();

    if(a_feedforward_control > 1.0) a_feedforward_control = 1.0;
    if(a_feedback_control > 0.5) a_feedback_control = 0.5;

    double output = a_feedforward_control + a_feedback_control;
    
    if(mControlData.desireSpeed < 0.8 && mNavData.gpsSpeed < 1.0) 
        output = -1.0;

    if(output > 1.0) output = 1.0;

    return output;
}

