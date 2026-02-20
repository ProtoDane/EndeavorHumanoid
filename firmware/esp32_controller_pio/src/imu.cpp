#include "imu.h"

imuHandler::imuHandler() {
    // Constructor method
}

bool imuHandler::begin() {


    _imuSerial.begin(115200, SERIAL_8N1, 18, 19);

    if (!_bno08x.begin_UART(&_imuSerial)) {
        Serial.println("Failed to find BNO08x chip");
        return false;
    }

    Serial.println("Found BNO08x chip!");

    setReports(_reportType, _reportIntervalUs);

}

void imuHandler::setReports(sh2_SensorId_t reportType, long report_interval) {
    Serial.println("Setting desired reports");
    if (! _bno08x.enableReport(reportType, report_interval)) {
       Serial.println("Could not enable stabilized remote vector");
    }    
}

void imuHandler::quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
        ypr->yaw *= RAD_TO_DEG;
        ypr->pitch *= RAD_TO_DEG;
        ypr->roll *= RAD_TO_DEG;
    }
}

void imuHandler::quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees) {
    quaternionToEuler(rotational_vector->i, rotational_vector->j, rotational_vector->k, rotational_vector->real, ypr, degrees);
}

void imuHandler::quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees) {
    quaternionToEuler(rotational_vector->i, rotational_vector->j, rotational_vector->k, rotational_vector->real, ypr, degrees);
}

void imuHandler::process_bno08x(euler_t* ypr) {
    
    sh2_SensorValue_t sensorValue;
    // double filteredPitch = 0.0;

    if (_bno08x.wasReset()) {
        Serial.println("sensor was reset, reconfiguring reports...");
        setReports(_reportType, _reportIntervalUs);
    }

    if (_bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ARVR_STABILIZED_RV:
                quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, ypr, true);
            case SH2_GYRO_INTEGRATED_RV:
                // faster (more noise?)
                quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, ypr, true);
                break;
        }
    }
}