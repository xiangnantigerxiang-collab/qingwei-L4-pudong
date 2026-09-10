#include "canbus_comply.h"
#include <ros/package.h>

ros::Publisher can_msg_pub;
ros::Publisher ehb_msg_pub;
ros::Publisher can_send_pub;

CanbusComply canbusComply;

int hook_position_min = 182;
int hook_position_max = 254;
int pallet_position_min = 213;
int pallet_position_max = 254;

//load 4 position limit keys from config.cfg; a pair falls back to the
//built-in defaults when missing or degenerate (0<=min<max<=255, window>=20)
static int LoadPositionConfig(const char *path)
{
    FILE *fp = fopen(path, "r");
    if(fp == NULL) {
        printf("config.cfg not found at %s, using built-in defaults\n", path);
        return -1;
    }

    const char *keys[4] = {"hook_position_min", "hook_position_max",
                           "pallet_position_min", "pallet_position_max"};
    int vals[4] = {hook_position_min, hook_position_max,
                   pallet_position_min, pallet_position_max};
    int seen[4] = {0, 0, 0, 0};

    char line[256];
    while(fgets(line, sizeof(line), fp)) {
        char *p = line;
        while(*p == ' ' || *p == '\t') p++;
        for(int i = 0; i < 4; i++) {
            int len = strlen(keys[i]);
            if(strncmp(p, keys[i], len) != 0) continue;
            char *eq = p + len;
            while(*eq == ' ' || *eq == '\t') eq++;
            if(*eq != '=') continue;
            vals[i] = atoi(eq + 1);
            seen[i] = 1;
        }
    }
    fclose(fp);

    int rejects = 0;
    if(seen[0] && seen[1] && vals[0] >= 0 && vals[1] > vals[0] &&
       vals[1] <= 255 && vals[1] - vals[0] >= 20) {
        hook_position_min = vals[0];
        hook_position_max = vals[1];
    } else {
        printf("config.cfg: hook pair invalid or missing, using defaults %d/%d\n",
               hook_position_min, hook_position_max);
        rejects++;
    }
    if(seen[2] && seen[3] && vals[2] >= 0 && vals[3] > vals[2] &&
       vals[3] <= 255 && vals[3] - vals[2] >= 20) {
        pallet_position_min = vals[2];
        pallet_position_max = vals[3];
    } else {
        printf("config.cfg: pallet pair invalid or missing, using defaults %d/%d\n",
               pallet_position_min, pallet_position_max);
        rejects++;
    }
    return rejects;
}

void CanbusCallBack(const can_msgs::Frame &msg)
{
    canbusComply.RecvCanData(msg);
}

void T1Callback(const ros::TimerEvent &real)
{
    int sensorstate = 0;
    ros::param::get("/planning/sensorstate", sensorstate);

    int network_down = 0;
    ros::param::get("/robot/planning/netcheck", network_down);

    int planning_alive = 0;
    ros::param::get("/planning/alive", planning_alive);

    canbusComply.EmergencyStop = 0;
    if(sensorstate != 0) canbusComply.EmergencyStop = 1;  //nonzero = sensor fault
    if(network_down != 0) canbusComply.EmergencyStop = 1; //nonzero = network down
    if(!planning_alive) canbusComply.EmergencyStop = 1;

    double dtm = canbusComply.sysTime.now - canbusComply.sysTime.msgCanComm;
    if(dtm > 1.0) canbusComply.EmergencyStop = 1;

    uint8_t can[8];
    can[0] = 0x0E;
    can[1] = 0x00;
    can[2] = 0x05;
    can[3] = 0x00;
    can[4] = 0x00;
    can[5] = 0x00;
    can[6] = 0x00;
    can[7] = 0x00;

    if ((sensorstate & 0x02) == 2) {
        //can[0] = 0x06; // lidar
        //canbusComply.SendVehicleControlCmd(0x201, can);
    }

    if ((sensorstate & 0x04) == 4) {
        //can[0] = 0x08; // camera
        //canbusComply.SendVehicleControlCmd(0x201, can);
    }

    if ((sensorstate & 0x08) == 8) {
        //can[0] = 0x07; // software dead
        //can[0] = 0x04; // gnss
	//can[0] = 0x01; // ready
        //canbusComply.SendVehicleControlCmd(0x201, can);
    }

    int fencealarm = 0;
    ros::param::get("alarmcmd", fencealarm);
    if (fencealarm == 1) {
        //can[0] = 0x0D; // fence alarm
        //canbusComply.SendVehicleControlCmd(0x201, can);
        //ros::param::set("alarmcmd", 0);
    }

    //hook status check
    static int hook_max_count = 0;
    static int hook_min_count = 0;
    static int pallet_max_count = 0;
    static int pallet_min_count = 0;
    static int hookmid = 0;
    static int hookmid_count = 0;

    int hookpos = canbusComply.mCanMsg.hookPos;
    int palletpos = canbusComply.mCanMsg.palletPos;
    int hookmax = hook_position_max;
    int hookmin = hook_position_min;
    int palletmax = pallet_position_max;
    int palletmin = pallet_position_min;
    
    canbusComply.mCanMsg.hookStatus = 0;
    canbusComply.mCanMsg.palletStatus = 0;

    if(abs(hookpos-hookmin) < 5) hook_min_count += 1;
    else hook_min_count = 0;

    if(hook_min_count > 10) {
        canbusComply.mCanMsg.hookStatus = 4;//up end
        if(hook_min_count > 100) hook_min_count = 100;
    }

    if(abs(hookpos-hookmax) < 5) hook_max_count += 1;
    else hook_max_count = 0;

    if(hook_max_count > 10) {
        canbusComply.mCanMsg.hookStatus = 3;//down end
        if(hook_max_count > 100) hook_max_count = 100;
    }

    if(abs(palletpos-palletmin) < 5) pallet_min_count += 1;
    else pallet_min_count = 0;

    if(pallet_min_count > 10) {
        canbusComply.mCanMsg.palletStatus = 4;//up end
        if(pallet_min_count > 100) pallet_min_count = 100;
    }

    if(abs(palletpos-palletmax) < 5) pallet_max_count += 1;
    else pallet_max_count = 0;

    if(pallet_max_count > 10) {
        canbusComply.mCanMsg.palletStatus = 3;//down end
        if(pallet_max_count > 100) pallet_max_count = 100;
    }

    if(hookpos >= hookmin + 5 && hookpos <= hookmax - 5) {
        if(abs(hookpos-hookmid) > 3) {
            hookmid = hookpos;
            hookmid_count = 0;
        }else hookmid_count += 1;

	if(hookmid_count > 10) {
            canbusComply.mCanMsg.hookStatus = 1;//block
            if(hookmid_count > 100) hookmid_count = 100;
        }
    }else hookmid_count = 0;

    ehb_msg_pub.publish(canbusComply.mEHBMsg);
}

void T2Callback(const ros::TimerEvent &real)
{
    canbusComply.VehicleComm();
    //canbusComply.checkCamera();
    can_msg_pub.publish(canbusComply.mCanMsg);
}

void CanCommCallBack(const canbus::can_comm_msg &msg)
{
    canbusComply.can_comm_cmd = msg;
    canbusComply.sysTime.msgCanComm = canbusComply.sysTime.now;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_node");
    ros::NodeHandle nh;

    //load position limits from config.cfg, then publish to rosparam for pnc
    std::string cfgFile = ros::package::getPath("canbus") + "/config.cfg";
    LoadPositionConfig(cfgFile.c_str());

    ros::param::set("/canbus/hookposition/min", hook_position_min);
    ros::param::set("/canbus/hookposition/max", hook_position_max);
    ros::param::set("/canbus/palletposition/min", pallet_position_min);
    ros::param::set("/canbus/palletposition/max", pallet_position_max);

    ros::Subscriber can_comm_sub = nh.subscribe(
        "/can_comm_msg", 1, CanCommCallBack,
        ros::TransportHints().tcpNoDelay());
    ros::Subscriber can_msg_sub = nh.subscribe(
        "/can_recv", 100, CanbusCallBack,
        ros::TransportHints().tcpNoDelay());

    can_msg_pub = nh.advertise<canbus::can_msg>("/can_msg", 1);
    ehb_msg_pub = nh.advertise<canbus::ehb_msg>("/ehb_msg", 1);
    can_send_pub = nh.advertise<can_msgs::Frame>("/can_send", 1);

    ros::Timer T1 = nh.createTimer(ros::Duration(0.1), T1Callback);
    ros::Timer T2 = nh.createTimer(ros::Duration(0.05), T2Callback);

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        canbusComply.sysTime.now = ros::Time::now().toSec();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
