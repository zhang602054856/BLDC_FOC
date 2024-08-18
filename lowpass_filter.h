//

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

/**
  * 低通滤波器类
  */

class LowPassFilter
{
public:
    /**
     * @Tf - 低通滤波时间常数
     */
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float process(float x);

protected:
    unsigned long timestamp_prev;  //!< 最后执行时间戳
    float Tf; //!< 低通滤波时间常数
    float value;
};


// class velocity
// {
//     public:
//         velocity();

//         ~velocity() = default;

//         void update(float x);

//         float getVelocity() { return radian_vel; }
//         float getRpm() { return rpm; }

//     private:
//         LowPassFilter fliter;
//         float radian_vel; //unit: rad/s
//         float rpm;
// };




#endif
