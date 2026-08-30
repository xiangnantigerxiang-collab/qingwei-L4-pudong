#include "pubalgor.h"

PubAlgor::PubAlgor(){
	
}

float PubAlgor::FuzzyDataProcess(const float *data_array,const float *limit_array,const int size,const float data){
	int i_temp = 0;
	float para1,para2;
	float data_value = data;

	if(data_value < data_array[0])	
		data_value = data_array[0];
	else if(data_value > data_array[size - 1])
		data_value = data_array[size - 1];

	for(i_temp = 0;i_temp < size;i_temp++){
		if(data_value <= data_array[i_temp])
			break;
	}
	if(i_temp < 1)
		i_temp = 1;
	else if(i_temp > size - 1)
		i_temp = size - 1;
	para1 = (data_array[i_temp] - data_value)/(data_array[i_temp] - data_array[i_temp-1]);
	para2 = (data_value - data_array[i_temp - 1])/(data_array[i_temp] - data_array[i_temp-1]);

	return para1*limit_array[i_temp - 1] + para2*limit_array[i_temp];
}

float PubAlgor::CalculatePoint2PointAngleByDelta_P(float detaX,float detaY){
    float angle_temp = 0;
    
	if(detaX > -0.0001 && detaX < 0.0001)
    {
        if(detaY > 0)
            angle_temp = 0;
        else
            angle_temp = 180;
    }else{
        angle_temp = atan((double)detaY / detaX) / M_PI * 180;
        if (detaX > 0)
            angle_temp = 90 - angle_temp;
        else
            angle_temp = 270 - angle_temp;
    }
    return angle_temp;
}

float PubAlgor::CalculatePoint2PointDistance_P(float x1_coor,float y1_coor,float z1_coor,
		float x2_coor,float y2_coor,float z2_coor)
{
	float disn = 0;
	float deta_x,deta_y,deta_z;

	deta_x = x1_coor - x2_coor;
	deta_y = y1_coor - y2_coor;
	deta_z = z1_coor - z2_coor;
	disn = sqrt(deta_x*deta_x + deta_y*deta_y + deta_z*deta_z);

	return disn;
}

float PubAlgor::CalculatePoint2PointAngle_P(float start_x,float start_y,float stop_x,float stop_y){
    float detaX,detaY,angle_temp;
    detaX = stop_x - start_x;
    detaY = stop_y - start_y;
    if(detaX > -0.0001 && detaX < 0.0001)
    {
        if(detaY > 0)
            angle_temp = 0;
        else
            angle_temp = 180;
    }else{
        angle_temp = atan((float)detaY / detaX) / M_PI * 180;
        if (detaX > 0)
            angle_temp = 90 - angle_temp;
        else
            angle_temp = 270 - angle_temp;
    }
    return angle_temp;
}

int PubAlgor::FindKeyPointByTargetPoint_P(vector<XYZ_COOR_S> xyz_list,const float x_coor,const float y_coor){
	
	int road_point_size = xyz_list.size();
    XYZ_COOR_S xyz_temp;
    float x_temp;
    float y_temp;
    float x_cur = x_coor;
    float y_cur = y_coor;
    float dis = 0;
    float min = 100.0;
    int key = 0;

    for (int j = 0; j < road_point_size; j++)//modify j = 0
    {
        xyz_temp = xyz_list.at(j);

        x_temp = xyz_temp.x_axis;
        y_temp = xyz_temp.y_axis;

        dis = sqrt( (x_temp - x_cur)*(x_temp - x_cur) + (y_temp - y_cur)*(y_temp - y_cur) );

        if (min > dis)
        {
            min = dis;
            key = j;
        }
    }
    return(key);
}

int PubAlgor::FindKeyPointByTargetPoint_P(vector<XYZ_COOR_S> xyz_list,const float x_coor,const float y_coor,\
						const int tSrc,const int tDes){
	
	int road_point_size = xyz_list.size();
    XYZ_COOR_S xyz_temp;
    float x_temp;
    float y_temp;
    float x_cur = x_coor;
    float y_cur = y_coor;
    float dis = 0;
    float min = 100.0;
    int key = 0;

	if( tSrc >= tDes )	
		return 0;
	
    for (int j = tSrc; j <= tDes; j++)//modify j = 0
    {
        xyz_temp = xyz_list.at(j);

        x_temp = xyz_temp.x_axis;
        y_temp = xyz_temp.y_axis;

        dis = sqrt( (x_temp - x_cur)*(x_temp - x_cur) + (y_temp - y_cur)*(y_temp - y_cur) );

        if (min > dis)
        {
            min = dis;
            key = j;
        }
    }
    return(key);
}

XYZ_COOR_S PubAlgor::CalculNextPointByAngle_P(float x_coor,float y_coor,float heading,float distance){
	XYZ_COOR_S xyz_temp; 
    float slope; 
    float dx,dy; 
    
    slope = tan((90 - heading)/180*3.141592);
    if(heading >= 0.5 && heading <= 179.5){
        dx = distance/sqrt(1 + slope*slope);
        dy = dx*slope;
    }else if(heading >= 180.5 && heading <= 359.5){
        dx = -1.0*distance/sqrt(1 + slope*slope);
        dy = dx*slope;
    }else if(heading > 359.5 || heading < 0.5){
        dx = 0;
        dy = distance;
    }else if(heading > 179.5 && heading < 180.5){
        dx = 0;
        dy = -1.0*distance;
    }

    xyz_temp.x_axis = dx + x_coor;
    xyz_temp.y_axis = dy + y_coor;
	xyz_temp.heading = heading;
	
	return xyz_temp;
}

float PubAlgor::CalculateMinDistance_P(vector<XYZ_COOR_S> road_list,float x_axis,float y_axis)
{
    int road_point_size = road_list.size();
    XYZ_COOR_S xyz_temp;
    float x_temp;
    float y_temp;
    float x_cur = x_axis;//4.562;//PubApi::gX_Axis;
    float y_cur = y_axis;//50.061;//PubApi::gY_Axis;
    float dis = 0;
    float min = 100.0;

    for (int j = 0; j < road_point_size; j++)
    {
        xyz_temp = road_list[j];

        x_temp = xyz_temp.x_axis;
        y_temp = xyz_temp.y_axis;

        dis = sqrt( (x_temp - x_cur)*(x_temp - x_cur) + (y_temp - y_cur)*(y_temp - y_cur) );

        if (min > dis)
        {
            min = dis;
        }
    }
    return min;
}

XYZ_COOR_S PubAlgor::CaculateCrossPoint_P(float xPoint1,float yPoint1,float angle1,float xPoint2,float yPoint2,float angle2){
    XYZ_COOR_S xy_temp;
    float slope1 = 0,slope2 = 0;
	float angle_temp1 = 0,angle_temp2 = 0;

	angle_temp1 = angle1;
	angle_temp2 = angle2;
    /*if(MULTIPLE90(angle1) && !MULTIPLE90(angle2)){
        slope2 = tan((90 - angle2)/180*3.141592);
        xy_temp.x_axis = (yPoint1 - yPoint2)/slope2 + xPoint2;
        xy_temp.y_axis = yPoint1;
    }else if(!MULTIPLE90(angle1) && MULTIPLE90(angle2)){
        slope1 = tan((90 - angle1)/180*3.141592);
        xy_temp.x_axis = (yPoint2 - yPoint1)/slope1 + xPoint1;
        xy_temp.y_axis = yPoint2;
    }else if(!MULTIPLE90(angle1) && !MULTIPLE90(angle2)){
        slope1 = tan((90 - angle1)/180*3.141592);
        slope2 = tan((90 - angle2)/180*3.141592);
        xy_temp.x_axis = (yPoint2 - yPoint1 + slope1*xPoint1 - slope2*xPoint2)/(slope1 - slope2);
        xy_temp.y_axis = ((xPoint2 - xPoint1)*slope1*slope2 - slope1*yPoint2 + slope2*yPoint1)/(slope2 - slope1);

    }else{
        xy_temp.x_axis = 0;
        xy_temp.y_axis = 0;      
    }*/
	if(MULTIPLE90(angle_temp1)){
		angle_temp1 += 5;
		if(angle_temp1 >= 360)
			angle_temp1 -= 360;
	}
	if(MULTIPLE90(angle_temp2)){
		angle_temp2 += 5;
		if(angle_temp2 >= 360)
			angle_temp2 -= 360;
	}
	slope1 = tan((90 - angle_temp1)/180*3.141592);
	slope2 = tan((90 - angle_temp2)/180*3.141592);
	xy_temp.x_axis = (yPoint2 - yPoint1 + slope1*xPoint1 - slope2*xPoint2)/(slope1 - slope2);
	xy_temp.y_axis = ((xPoint2 - xPoint1)*slope1*slope2 - slope1*yPoint2 + slope2*yPoint1)/(slope2 - slope1);	

    return xy_temp;
}

vector<string> PubAlgor::SplitString(string str,string flag){

    vector<string> v ;
    string::size_type pos1,pos2;
    pos2 = str.find(flag);
    pos1 = 0;

    while(string::npos != pos2){
        v.push_back(str.substr(pos1,pos2 - pos1));
        pos1 = pos2 + 1;
        pos2 = str.find(flag,pos1);
    }
    if( pos1 != str.length() )
        v.push_back(str.substr( pos1 ));
    return v;
}

vector<string> PubAlgor::Split(string str,char del){
    stringstream ss(str);
    string tok;
    vector<string> ret;

    while(getline(ss,tok,del)){
        ret.push_back(tok);
    }
    return ret;
}

int16_t PubAlgor::Uchar2Int16(const uint8_t *p_ch){
    uint16_t temp_uint16 = ((*(p_ch+1) & 0xff)<<8)|(*p_ch & 0xff);
    int16_t temp_int16 = (*(int16_t*)&temp_uint16);

    return temp_int16;
}

int16_t PubAlgor::Uchar2Int16_r(const uint8_t *p_ch){
    uint16_t temp_uint16 = ((*(p_ch) & 0xff)<<8)|(*(p_ch + 1) & 0xff);
    int16_t temp_int16 = (*(int16_t*)&temp_uint16);
    
    return temp_int16;
}

int32_t PubAlgor::Uchar2Int32(const uint8_t *p_ch){
    uint32_t temp_uint32 = ((*(p_ch+3) & 0xff)<<24) |((*(p_ch+2) & 0xff)<<16) |((*(p_ch+1) & 0xff)<<8)|(*p_ch & 0xff);
    int32_t temp_int32 = (*(int32_t*)&temp_uint32);

    return temp_int32;
}

int32_t PubAlgor::Uchar2Int32_r(const uint8_t *p_ch){
    uint32_t temp_uint32 = ((*(p_ch) & 0xff)<<24) |((*(p_ch+1) & 0xff)<<16) |((*(p_ch+2) & 0xff)<<8)|(*(p_ch+3) & 0xff);
    int32_t temp_int32 = (*(int32_t*)&temp_uint32);

    return temp_int32;
}


vector<XYZ_COOR_S> PubAlgor::GenerateBezierPath_P(const float start_x,const float start_y,const float start_angle,
	const float stop_x,const float stop_y,const float stop_angle){
	float x[4],y[4];
	float angle_temp = 0;
	float distance = 0;
	int size = 0;
	float detaX,detaY;
	XYZ_COOR_S xyz_temp;
	vector<XYZ_COOR_S> xyz_list;

	x[0] = start_x;
	y[0] = start_y;
	x[3] = stop_x;
	y[3] = stop_y;

	distance = CalculatePoint2PointDistance_P(start_x,start_y,0,stop_x,stop_y,0);

	xyz_temp = CalculNextPointByAngle_P(start_x,start_y,start_angle,distance*0.4);
	x[1] = xyz_temp.x_axis;
	y[1] = xyz_temp.y_axis;
	angle_temp = stop_angle + 180;
	if(angle_temp > 360)
		angle_temp -= 360;
	xyz_temp = CalculNextPointByAngle_P(stop_x,stop_y,angle_temp,distance*0.4);
	x[2] = xyz_temp.x_axis;
	y[2] = xyz_temp.y_axis;

	size = (int)(distance/0.05);
	//qDebug()<<"size:"<<size<<distance<<0.1/distance;

	xyz_temp.x_axis = start_x;
	xyz_temp.y_axis = start_y;
	xyz_temp.z_axis = 38.79;
	xyz_temp.heading = start_angle;//PubApi::gHead;
	xyz_temp.velocity = 5.0;
	xyz_temp.p2pDistance = 0.05;
	xyz_list.push_back(xyz_temp);

	for(int i = 1;i < size;i++){
		float t = 0.05/distance;
		xyz_temp.x_axis = x[0]*(1 - i*t)*(1 - i*t)*(1 - i*t) + \
						  3*x[1]*(i*t)*(1 - i*t)*(1 - i*t) + \
						  3*x[2]*(i*t)*(i*t)*(1 - i*t) + x[3]*(i*t)*(i*t)*(i*t);
		xyz_temp.y_axis = y[0]*(1 - i*t)*(1 - i*t)*(1 - i*t) + \
						  3*y[1]*(i*t)*(1 - i*t)*(1 - i*t) + \
						  3*y[2]*(i*t)*(i*t)*(1 - i*t) + y[3]*(i*t)*(i*t)*(i*t);
		xyz_temp.z_axis = 38.79;
		detaX = xyz_temp.x_axis - xyz_list.at(xyz_list.size() - 1).x_axis;
		detaY = xyz_temp.y_axis - xyz_list.at(xyz_list.size() - 1).y_axis;
		if(detaX > -0.0001 && detaX < 0.0001)
		{
			if(detaY > 0)
				angle_temp = 0;
			else
				angle_temp = 180;
		}else{
			angle_temp = atan((float)detaY / detaX) / M_PI * 180;
			if (detaX > 0)
				angle_temp = 90 - angle_temp;
			else
				angle_temp = 270 - angle_temp;
		}
		xyz_temp.heading = angle_temp;
		xyz_temp.velocity = 5.0;
		xyz_temp.p2pDistance = CalculatePoint2PointDistance_P(xyz_temp.x_axis,xyz_temp.y_axis,0,\
				xyz_list.at(xyz_list.size() - 1).x_axis,xyz_list.at(xyz_list.size() - 1).y_axis,0);
		xyz_list.push_back(xyz_temp);
	}

	xyz_temp.x_axis = stop_x;
	xyz_temp.y_axis = stop_y;
	xyz_temp.z_axis = 38.79;
	xyz_temp.heading = stop_angle;//PubApi::gHead;
	xyz_temp.velocity = 5.0;
	xyz_temp.p2pDistance = 0;//pubapi.GetXYZDistance(xyz_temp.x_axis,xyz_temp.y_axis,0,\
	xyz_list.at(xyz_list.size() - 1).x_axis,xyz_list.at(xyz_list.size() - 1).y_axis,0);
	xyz_list.push_back(xyz_temp);

	return xyz_list;
}

vector<XYZ_COOR_S> PubAlgor::GenerateBezierPathBy4Point_P(vector<float> point_x,vector<float> point_y){
	float x[4],y[4];
	float angle_temp = 0;
	float distance = 0;
	int size = 0;
	float detaX,detaY;
	XYZ_COOR_S xyz_temp;
	vector<XYZ_COOR_S> xyz_list;

	for(int i = 0;i < 4;++i){
		x[i] = point_x[i];
		y[i] = point_y[i];
	}

	distance = CalculatePoint2PointDistance_P( x[0],y[0],0,x[3],y[3],0);

	size = (int)(distance/0.05);
	
	xyz_temp.x_axis = x[0];
	xyz_temp.y_axis = y[0];
	xyz_temp.heading = CalculatePoint2PointAngle_P(x[0],y[0],x[1],y[1]);
	xyz_list.push_back(xyz_temp);
	
	for(int i = 1;i < size;i++){
		float t = 0.05/distance;
		xyz_temp.x_axis = x[0]*(1 - i*t)*(1 - i*t)*(1 - i*t) + \
						  3*x[1]*(i*t)*(1 - i*t)*(1 - i*t) + \
						  3*x[2]*(i*t)*(i*t)*(1 - i*t) + x[3]*(i*t)*(i*t)*(i*t);
		xyz_temp.y_axis = y[0]*(1 - i*t)*(1 - i*t)*(1 - i*t) + \
						  3*y[1]*(i*t)*(1 - i*t)*(1 - i*t) + \
						  3*y[2]*(i*t)*(i*t)*(1 - i*t) + y[3]*(i*t)*(i*t)*(i*t);
		xyz_temp.z_axis = 38.79;
		detaX = xyz_temp.x_axis - xyz_list.at(xyz_list.size() - 1).x_axis;
		detaY = xyz_temp.y_axis - xyz_list.at(xyz_list.size() - 1).y_axis;
		if(detaX > -0.0001 && detaX < 0.0001)
		{
			if(detaY > 0)
				angle_temp = 0;
			else
				angle_temp = 180;
		}else{
			angle_temp = atan((float)detaY / detaX) / M_PI * 180;
			if (detaX > 0)
				angle_temp = 90 - angle_temp;
			else
				angle_temp = 270 - angle_temp;
		}
		xyz_temp.heading = angle_temp;
		xyz_temp.velocity = 5.0;
		xyz_temp.p2pDistance = CalculatePoint2PointDistance_P(xyz_temp.x_axis,xyz_temp.y_axis,0,\
				xyz_list.at(xyz_list.size() - 1).x_axis,xyz_list.at(xyz_list.size() - 1).y_axis,0);
		if(xyz_temp.p2pDistance < 0.05)
			continue;
		xyz_list.push_back(xyz_temp);
	}

	return xyz_list;
}

vector<XYZ_COOR_S> PubAlgor::GeneratePoint2PointPath(const float start_x,const float start_y,\
								const float stop_x,const float stop_y){
	float angle_temp = 0;
	float distance_temp = 0;
	XYZ_COOR_S xyz_temp;
	int size = 0;
	vector<XYZ_COOR_S> xyz_list;

	angle_temp = CalculatePoint2PointAngle_P(start_x,start_y,stop_x,stop_y);
	distance_temp = CalculatePoint2PointDistance_P(start_x,start_y,0,stop_x,stop_y,0);
	size = (int)(distance_temp/0.05);
	for(int i = 1;i < size;++i){
		xyz_temp = CalculNextPointByAngle_P(start_x,start_y,angle_temp,(float)(i*0.05));
		xyz_temp.z_axis = 38.79;
		xyz_temp.heading = angle_temp;
		xyz_temp.velocity = 5.0;
		xyz_temp.p2pDistance = 0.05;
		xyz_list.push_back(xyz_temp);
	}
	return xyz_list;
}

vector<XYZ_COOR_S> PubAlgor::PanPath_P(const vector<XYZ_COOR_S> converPathList,float detaX,float detaY){
    XYZ_COOR_S xyz_temp;
    float x_temp,y_temp;
    int size = converPathList.size();
    vector<XYZ_COOR_S> rtnPathList;
    rtnPathList.clear();

    for(int i =0 ;i < size - 1;i++){
        x_temp = converPathList[i].x_axis;
        y_temp = converPathList[i].y_axis;
        xyz_temp = converPathList[i];
        xyz_temp.x_axis = x_temp + detaX;
        xyz_temp.y_axis = y_temp + detaY;
        rtnPathList.push_back(xyz_temp);
    }

    return rtnPathList;
}

vector<XYZ_COOR_S> PubAlgor::GenerateExtendedPath(float tStartX,float tStartY,float tStartAngle,float tExtendDis){
	vector<XYZ_COOR_S> rtn_list;
	XYZ_COOR_S xyz_temp;
	float dis_sum = 0;
	rtn_list.clear();

	while(1){
		dis_sum += 0.1;
		xyz_temp = CalculNextPointByAngle_P(tStartX,tStartY,tStartAngle,dis_sum);
		xyz_temp.z_axis = 0;
		xyz_temp.heading = tStartAngle;
		xyz_temp.velocity = 5.0;
		xyz_temp.p2pDistance = 0.1;
		rtn_list.push_back(xyz_temp);

		if(dis_sum > tExtendDis)
			break;
	}
	return rtn_list;
}








