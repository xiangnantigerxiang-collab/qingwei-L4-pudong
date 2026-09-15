// 纵向速度控制 SpeedControl 实现：7x7 模糊表、PID 加速度计算、
// SpeedTrack 油门/刹车分配
#include "longitudinal_speed_control.h"

const float SpeedControlDeltaKp[7][7] = {
    {2.43, 1.94, 1.77, 1.25, 0.69, 0.4, 0.006},
    {1.94, 1.94, 1.35, 0.69, 0.26, 0.003, -0.6},
    {1.77, 1.35, 0.909, 0.69, -0.005, -0.26, -1.09},
    {1.25, 0.8, 0.26, 0.1, -0.1, -0.69, -1.25},
    {1.09, 0.26, -0.004, -0.26, -0.69, -0.909, -1.77},
    {0.596, -0.1, -0.26, -0.8, -1.35, -1.36, -1.94},
    {-0.006, -0.5, -0.8, -1.67, -1.77, -1.94, -2.43}};

const float SpeedControlDeltaKi[7][7] = {
    {-2.43, -1.94, -1.77, -1.25, -0.69, -0.4, -0.006},
    {-2.43, -1.48, -1.35, -0.69, -0.26, -0.003, 0.4},
    {-1.94, -0.909, -0.8, -0.26, 0.1, 0.26, 0.69},
    {-1.77, -0.8, -0.1, 0.003, 0.26, 0.909, 1.35},
    {-0.8, -0.26, 0.005, 0.69, 0.8, 1.35, 1.94},
    {-0.5, -0.1, 0.26, 0.693, 1.35, 1.48, 2.43},
    {0.006, 0.4, 0.69, 1.25, 1.77, 1.94, 2.43}};

const float SpeedControlDeltaKd[7][7] = {
    {1, -0.371, -1.48, -2.43, -1.94, -0.444, 0.97},
    {0.6, -0.371, -1.48, -1.35, -1.35, -0.381, 0.4},
    {0.4, -0.371, -1.35, -0.909, -0.69, -0.69, -0.012},
    {-0.012, -0.69, -0.69, -0.69, -0.5, -0.6, -0.0119},
    {0.562, 0.218, 0, 0, 0, 0.218, 0.562},
    {0.972, 0.0715, 0.262, 0.693, 0.69, 0.8, 0.972},
    {2.43, 1.77, 1.67, 1.25, 1.09, 1.2, 2.43}};

SpeedControl::SpeedControl() {
    m_kp = 0;
    m_ki = 0;
    m_kd = 0;
    m_min_acc = -4.5;
    m_max_acc = 2.0;

    m_speed_dev = 0;
    m_speed_dev_last = 0;
    m_speed_add = 0;
    m_speed_dif = 0;
    for(int i = 0; i < sizeof(m_speed_array) / sizeof(m_speed_array[0]); i++)
        m_speed_array[i] = 0;

    m_throttle_kp = 0;
    m_throttle_ki = 0;
    m_throttle_kd = 0;
}

void SpeedControl::SpeedFuzzyPIDControl(float tBiaV, float tBiaV_d, float& tKp,
                                        float& tKi, float& tKd) {
    float Up = 10;
    float Ui = 50;
    float Ud = 20;  // 输出量清晰化因子
    float E = 0;
    float EC = 0;

    uint indexE = 0;
    uint indexEC = 0;
    // BiaV_temp；BiaV;BiaV_d;

    E = tBiaV;
    EC = tBiaV_d;

    if(E <= -1)
        indexE = 0;
    if(E > -1 && E <= -0.5)
        indexE = 1;
    if(E > -0.5 && E <= -0.2)
        indexE = 2;
    if(E > -0.2 && E <= 0.2)
        indexE = 3;
    if(E > 0.2 && E <= 0.5)
        indexE = 4;
    if(E > 0.5 && E <= 1)
        indexE = 5;
    if(E > 1)
        indexE = 6;

    if(EC <= -0.3)
        indexEC = 0;
    if(EC > -0.3 && EC <= -0.2)
        indexEC = 1;
    if(EC > -0.2 && EC <= -0.1)
        indexEC = 2;
    if(EC > -0.1 && EC <= 0.1)
        indexEC = 3;
    if(EC > 0.1 && EC <= 0.2)
        indexEC = 4;
    if(EC > 0.2 && EC <= 0.3)
        indexEC = 5;
    if(EC > 0.3)
        indexE = 6;
    tKp = SpeedControlDeltaKp[indexE][indexEC] / Up;
    tKi = SpeedControlDeltaKi[indexE][indexEC] / Ui;
    tKd = SpeedControlDeltaKd[indexE][indexEC] / Ud;
}

float SpeedControl::AccelerationCalculateBySpeed_P(float tDesiredSpeed,
                                                   float tCurSpeed) {
    float acc_temp = 0;
    float speed_dev_temp = 0;
    float speed_temp = 0;
    float acc_max = 0, acc_min = 0;
    float delta_kp = 0, delta_ki = 0, delta_kd = 0;

    speed_dev_temp = tDesiredSpeed - tCurSpeed;
    m_speed_dev_last = m_speed_dev;
    speed_temp = m_speed_dev_last + m_speed_dif / 4;
    m_speed_dev = 0.7 * speed_temp + 0.3 * speed_dev_temp;

    for(int i = 0; i < 5; i++)
        m_speed_array[i] = m_speed_array[i + 1];
    m_speed_array[5] = m_speed_dev;

    m_speed_dif = m_speed_array[5] + m_speed_array[4] - m_speed_array[3] -
                  m_speed_array[2];
    if(m_speed_dev > 0.15)
        m_speed_add = 0.95 * m_speed_add + 0.15;
    else if(m_speed_dev < -0.15)
        m_speed_add = 0.95 * m_speed_add - 0.15;
    else
        m_speed_add = 0.95 * m_speed_add + m_speed_dev;

    if(m_speed_dev > 3.5)
        speed_dev_temp = 3.5;
    else if(m_speed_dev < -4.5)
        speed_dev_temp = -4.5;
    else
        speed_dev_temp = m_speed_dev;
    // 模糊
    SpeedFuzzyPIDControl(m_speed_dev, m_speed_dif, delta_kp, delta_ki, delta_kd);
    acc_temp = (m_kp + delta_kp) * speed_dev_temp +
               (m_ki + delta_ki) * m_speed_add + (m_kd + delta_kd) * m_speed_dif * 2;

    if((acc_temp - m_acc_last) > 0.1)
        acc_temp = m_acc_last + 0.1;
    else if((acc_temp - m_acc_last) < -1)
        acc_temp = m_acc_last - 1;

    if(acc_temp > m_max_acc)
        acc_temp = m_max_acc;
    else if(acc_temp < m_min_acc)
        acc_temp = m_min_acc;
    acc_temp = m_acc_last;

    return acc_temp;
}

float SpeedControl::SpeedIncreasePID(const float delta_error) {
    static float delta_error_last_s = 0;
    static float delta_error_previous_s = 0;
    float kp = m_throttle_kp;
    float ki = m_throttle_ki;
    float kd = m_throttle_kd;
    float rtn_value = 0;
    // read from parameter server
    rtn_value = kp * (delta_error - delta_error_last_s) +
                ki * delta_error_last_s +
                kd * (delta_error - 2 * delta_error_last_s + delta_error_previous_s);

    return rtn_value;
}

int SpeedControl::FuzzyQuantization(const float tTemp) {
    float fuzzy_array[14] = {-100, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5, 0.5, 1.5,
                             2.5, 3.5, 4.5, 5.5, 100};

    for(int i = 0; i < 13; i++) {
        if(tTemp > fuzzy_array[i] && tTemp <= fuzzy_array[i + 1])
            return i;
    }
    return 0;
}

int SpeedControl::SpeedFuzzyControl(const float delta_error) {
    static float delta_error_last_s = 0;
    float delta_error_dev = 0;
    float kfe = 0.5;
    float kfec = 2;
    float fuk = 0;
    int fuzzy_table[13][13] = {
        {-4, -4, -4, -4, -3, -3, -2, -2, -2, -2, -1, -1, 0},
        {-4, -4, -4, -3, -3, -2, -2, -2, -2, -1, -1, 0, 0},
        {-4, -4, -3, -3, -2, -2, -2, -2, -1, -1, 0, 0, 0},
        {-4, -3, -3, -2, -2, -2, -2, -1, -1, 0, 1, 1, 1},
        {-3, -3, -2, -2, -2, -1, -1, -1, 0, 1, 1, 2, 2},
        {-3, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2},
        {-2, -2, -1, -1, 0, 0, 0, 1, 1, 2, 2, 3, 3},
        {-1, -1, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 4},
        {-1, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4},
        {0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4},
        {0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 4},
        {0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 4, 4, 4},
        {0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 4, 4, 4},
    };
    delta_error_dev = delta_error - delta_error_last_s;
    fuk = fuzzy_table[FuzzyQuantization(kfe * delta_error)]
                     [FuzzyQuantization(kfec * delta_error_dev)];
    return fuk;
}

void SpeedControl::SpeedTrack(float tDesireSpeed, float tCurSpeed, float tAcc,
                              uint8_t& tThrottle, uint8_t& tBrake) {
    float speed_increase = 0;
    float throttle_percent = (float)tThrottle;
    float delta_error = 0;
    int fuk = 0;

    delta_error = tDesireSpeed - tCurSpeed;
    // throttle calculate
    speed_increase += SpeedIncreasePID(delta_error);
    throttle_percent += speed_increase;
    if(throttle_percent > 100)
        throttle_percent = 100;
    else if(throttle_percent < 0)
        throttle_percent = 0;
    // brake calculate
    fuk = SpeedFuzzyControl(delta_error);
    if(delta_error >= -1.8) {  // e >= -0.5m/s
        tThrottle = throttle_percent;
        tBrake = 0;
    } else {
        if(throttle_percent < 0 && fuk < 0) {
            tThrottle = 0;
            switch(fuk) {
                case -1:
                    tBrake = 20;
                    break;
                case -2:
                    tBrake = 40;
                    break;
                case -3:
                    tBrake = 60;
                    break;
                case -4:
                    tBrake = 80;
                    break;
                default:
                    tBrake = 0;
            }
        } else {
            tThrottle = throttle_percent;
            tBrake = 0;
        }
    }
}
