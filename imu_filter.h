#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <Arduino.h>
#include <math.h>

#ifndef IMU_FILTER_BETA
#define IMU_FILTER_BETA 0.1f
#endif

#ifndef GRAVITY_MS2
#define GRAVITY_MS2 9.80665f
#endif

// Non-static so .ino + com_rx.h share one quaternion state
inline float &imu_q0() { static float v = 1.0f; return v; }
inline float &imu_q1() { static float v = 0.0f; return v; }
inline float &imu_q2() { static float v = 0.0f; return v; }
inline float &imu_q3() { static float v = 0.0f; return v; }

static inline float imuInvSqrt(float x)
{
    if (!isfinite(x) || x <= 0.0f) {
        return 0.0f;
    }
    float r = 1.0f / sqrtf(x);
    if (!isfinite(r)) {
        return 0.0f;
    }
    return r;
}

static inline void imuQuaternionNormalize()
{
    float magSq = imu_q0() * imu_q0() + imu_q1() * imu_q1()
                + imu_q2() * imu_q2() + imu_q3() * imu_q3();
    if (!isfinite(magSq) || magSq <= 0.0f) {
        imu_q0() = 1.0f;
        imu_q1() = 0.0f;
        imu_q2() = 0.0f;
        imu_q3() = 0.0f;
        return;
    }
    float recipNorm = imuInvSqrt(magSq);
    if (recipNorm <= 0.0f) {
        imu_q0() = 1.0f;
        imu_q1() = 0.0f;
        imu_q2() = 0.0f;
        imu_q3() = 0.0f;
        return;
    }
    imu_q0() *= recipNorm;
    imu_q1() *= recipNorm;
    imu_q2() *= recipNorm;
    imu_q3() *= recipNorm;
}

inline void imuFilterReset()
{
    imu_q0() = 1.0f;
    imu_q1() = 0.0f;
    imu_q2() = 0.0f;
    imu_q3() = 0.0f;
}

inline bool imuFilterQuaternionValid()
{
    return isfinite(imu_q0()) && isfinite(imu_q1())
        && isfinite(imu_q2()) && isfinite(imu_q3());
}

// Madgwick 6DOF IMU fusion. Accel in m/s^2, gyro in rad/s.
inline void imuFilterUpdate(float ax, float ay, float az, float gx, float gy, float gz, float dtSec)
{
    if (!(dtSec > 0.0f) || !isfinite(dtSec)) {
        return;
    }
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        !isfinite(gx) || !isfinite(gy) || !isfinite(gz)) {
        imuFilterReset();
        return;
    }
    if (!imuFilterQuaternionValid()) {
        imuFilterReset();
    }

    float q0 = imu_q0();
    float q1 = imu_q1();
    float q2 = imu_q2();
    float q3 = imu_q3();

    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    float accelMagSq = ax * ax + ay * ay + az * az;
    // Use accel only when magnitude is roughly 0.5g..2.0g ((m/s^2)^2 ≈ 24..384)
    if (accelMagSq > 24.0f && accelMagSq < 400.0f) {
        float recipNorm = imuInvSqrt(accelMagSq);
        if (recipNorm > 0.0f) {
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            float _2q0 = 2.0f * q0;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _4q0 = 4.0f * q0;
            float _4q1 = 4.0f * q1;
            float _4q2 = 4.0f * q2;
            float _8q1 = 8.0f * q1;
            float _8q2 = 8.0f * q2;
            float q0q0 = q0 * q0;
            float q1q1 = q1 * q1;
            float q2q2 = q2 * q2;
            float q3q3 = q3 * q3;

            float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1
                     + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
                     + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

            recipNorm = imuInvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            if (recipNorm > 0.0f) {
                s0 *= recipNorm;
                s1 *= recipNorm;
                s2 *= recipNorm;
                s3 *= recipNorm;

                qDot1 -= IMU_FILTER_BETA * s0;
                qDot2 -= IMU_FILTER_BETA * s1;
                qDot3 -= IMU_FILTER_BETA * s2;
                qDot4 -= IMU_FILTER_BETA * s3;
            }
        }
    }

    q0 += qDot1 * dtSec;
    q1 += qDot2 * dtSec;
    q2 += qDot3 * dtSec;
    q3 += qDot4 * dtSec;

    imu_q0() = q0;
    imu_q1() = q1;
    imu_q2() = q2;
    imu_q3() = q3;

    imuQuaternionNormalize();

    if (!imuFilterQuaternionValid()) {
        imuFilterReset();
    }
}

inline void imuFilterGetQuaternion(float &w, float &x, float &y, float &z)
{
    if (!imuFilterQuaternionValid()) {
        imuFilterReset();
    }
    w = imu_q0();
    x = imu_q1();
    y = imu_q2();
    z = imu_q3();
}

// Remove gravity using estimated orientation. Inputs and outputs in m/s^2.
inline void imuFilterGetLinearAccel(float ax, float ay, float az, float &linAx, float &linAy, float &linAz)
{
    if (!imuFilterQuaternionValid()) {
        imuFilterReset();
    }
    float q0 = imu_q0();
    float q1 = imu_q1();
    float q2 = imu_q2();
    float q3 = imu_q3();

    float gx = 2.0f * (q1 * q3 - q0 * q2);
    float gy = 2.0f * (q2 * q3 + q0 * q1);
    float gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    linAx = ax - gx * GRAVITY_MS2;
    linAy = ay - gy * GRAVITY_MS2;
    linAz = az - gz * GRAVITY_MS2;

    if (!isfinite(linAx) || !isfinite(linAy) || !isfinite(linAz)) {
        linAx = 0.0f;
        linAy = 0.0f;
        linAz = 0.0f;
    }
}

#endif
