#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO08x.h>
#include <Arduino.h>

#include "systemParams.h"
#include "config.h"
#include "servo2040.h"

struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

enum IMU_MODE {
    BNO08x, BNO055, NO_IMU
};

class imuHandler {
    public:
        imuHandler();
        bool begin();

        void setReports(sh2_SensorId_t reportType, long report_interval);
        void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
        void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false);
        void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false);

        void process_bno08x(euler_t* ypr);

    private:
            
        Adafruit_BNO08x _bno08x = Adafruit_BNO08x(-1);
        HardwareSerial _imuSerial = HardwareSerial(1);

        sh2_SensorId_t _reportType = SH2_ARVR_STABILIZED_RV;
        long _reportIntervalUs = 10 * 1000;  

        
};

#endif