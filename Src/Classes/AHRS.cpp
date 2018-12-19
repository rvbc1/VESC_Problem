/*
 * AHRS.cpp
 *
 *  Created on: 21.06.2018
 *      Author: mice
 */

#include <AHRS.h>

#include "Allshit.h"
#include "Gyro.h"
#include "Futaba.h"
#include "Tools.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "Mathematics.h"
#include "axis.h"

AHRS ahrs;

AHRS::AHRS() {
	// TODO Auto-generated constructor stub

}

AHRS::~AHRS() {
	// TODO Auto-generated destructor stub
}
// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf

#define SPIN_RATE_LIMIT 20

#define ATTITUDE_RESET_QUIET_TIME 250000   // 250ms - gyro quiet period after disarm before attitude reset
#define ATTITUDE_RESET_GYRO_LIMIT 15       // 15 deg/sec - gyro limit for quiet period
#define ATTITUDE_RESET_KP_GAIN    25.0     // dcmKpGain value to use during attitude reset
#define ATTITUDE_RESET_ACTIVE_TIME 500000  // 500ms - Time to wait for attitude to converge at high gain
#define GPS_COG_MIN_GROUNDSPEED 500        // 500cm/s minimum groundspeed for a gps heading to be considered valid

void AHRS::Init(void)
{
	tools.Init();
    canUseGPSHeading = false;
    ComputeRotationMatrix();
}
void AHRS::Process(void) {
	osEvent evt = osSignalWait(Signal_AHRS, osWaitForever);
	if (evt.status == osEventSignal) {
		UpdateAttitude();
	}
}
void AHRS::TaskNotify(void){
	osSignalSet(AHRSTaskHandle, Signal_AHRS);
}
void AHRS::ResetAccelerationSum(void)
{
    accSum[0] = 0.f;
    accSum[1] = 0.f;
    accSum[2] = 0.f;
    accSumCount = 0;
    accTimeSum = 0;
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void AHRS::MahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag, float mx, float my, float mz,
                                bool useCOG, float courseOverGround, const float dcmKpGain)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

    // Calculate general spin rate (rad/s)
    const float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

    // Use raw heading error (from GPS or whatever else)
    float ex = 0, ey = 0, ez = 0;
    if (useCOG) {
        while (courseOverGround >  M_PI_FLOAT) {
            courseOverGround -= (2.0f * M_PI_FLOAT);
        }

        while (courseOverGround < -M_PI_FLOAT) {
            courseOverGround += (2.0f * M_PI_FLOAT);
        }

        const float ez_ef = (- sin_approx(courseOverGround) * rMat[0][0] - cos_approx(courseOverGround) * rMat[1][0]);

        ex = rMat[2][0] * ez_ef;
        ey = rMat[2][1] * ez_ef;
        ez = rMat[2][2] * ez_ef;
    }

#ifdef USE_MAG
    // Use measured magnetic field vector
    float recipMagNorm = sq(mx) + sq(my) + sq(mz);
    if (useMag && recipMagNorm > 0.01f) {
        // Normalise magnetometer measurement
        recipMagNorm = invSqrt(recipMagNorm);
        mx *= recipMagNorm;
        my *= recipMagNorm;
        mz *= recipMagNorm;

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles

        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
        // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
        const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
        const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
        const float bx = sqrtf(hx * hx + hy * hy);

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        const float ez_ef = -(hy * bx);

        // Rotate mag error vector back to BF and accumulate
        ex += rMat[2][0] * ez_ef;
        ey += rMat[2][1] * ez_ef;
        ez += rMat[2][2] * ez_ef;
    }
#else
    UNUSED(useMag);
    UNUSED(mx);
    UNUSED(my);
    UNUSED(mz);
#endif

    // Use measured acceleration vector
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipAccNorm > 0.01f) {
        // Normalise accelerometer measurement
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if (dcm_ki > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = dcm_ki;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    quaternion buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    ComputeRotationMatrix();
}

void AHRS::UpdateEulerAngles(void)
{
	attitude.values.roll = atan2_approx(rMat[2][1], rMat[2][2]) * (180.0f / M_PI_FLOAT);
	attitude.values.pitch = ((0.5f * M_PI_FLOAT) - acos_approx(-rMat[2][0])) * (180.0f / M_PI_FLOAT);
	attitude.values.yaw = -atan2_approx(rMat[1][0], rMat[0][0]) * (180.0f / M_PI_FLOAT);

	if (attitude.values.yaw < 0.f)
		attitude.values.yaw += 360.f;
}

bool AHRS::IsAccelerometerHealthy(float *accAverage)
{
    float accMagnitude = sq(accAverage[X]) + sq(accAverage[Y]) + sq(accAverage[Z]);

    // Accept accel readings only in range 0.90g - 1.10g
    return (0.81f < accMagnitude) && (accMagnitude < 1.21f);
}

// Calculate the dcmKpGain to use. When armed, the gain is imuRuntimeConfig.dcm_kp * 1.0 scaling.
// When disarmed after initial boot, the scaling is set to 10.0 for the first 20 seconds to speed up initial convergence.
// After disarming we want to quickly reestablish convergence to deal with the attitude estimation being incorrect due to a crash.
//   - wait for a 250ms period of low gyro activity to ensure the craft is not moving
//   - use a large dcmKpGain value for 500ms to allow the attitude estimate to quickly converge
//   - reset the gain back to the standard setting
float AHRS::CalcKpGain(uint32_t currentTimeUs, bool useAcc, float *gyroAverage)
{
    static bool lastArmState = false;
    static uint32_t gyroQuietPeriodTimeEnd = 0;
    static uint32_t attitudeResetTimeEnd = 0;
    static bool attitudeResetCompleted = false;
    float ret;
    bool attitudeResetActive = false;

    if (futaba.SwitchA < SWITCH_DOWN) {
        if (lastArmState) {   // Just disarmed; start the gyro quiet period
            gyroQuietPeriodTimeEnd = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
            attitudeResetTimeEnd = 0;
            attitudeResetCompleted = false;
        }

        // If gyro activity exceeds the threshold then restart the quiet period.
        // Also, if the attitude reset has been complete and there is subsequent gyro activity then
        // start the reset cycle again. This addresses the case where the pilot rights the craft after a crash.
        if ((attitudeResetTimeEnd > 0) || (gyroQuietPeriodTimeEnd > 0) || attitudeResetCompleted) {
            if ((fabsf(gyroAverage[X]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (fabsf(gyroAverage[Y]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (fabsf(gyroAverage[Z]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (!useAcc)) {

                gyroQuietPeriodTimeEnd = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
                attitudeResetTimeEnd = 0;
            }
        }
        if (attitudeResetTimeEnd > 0) {        // Resetting the attitude estimation
            if (currentTimeUs >= attitudeResetTimeEnd) {
                gyroQuietPeriodTimeEnd = 0;
                attitudeResetTimeEnd = 0;
                attitudeResetCompleted = true;
            } else {
                attitudeResetActive = true;
            }
        } else if ((gyroQuietPeriodTimeEnd > 0) && (currentTimeUs >= gyroQuietPeriodTimeEnd)) {
            // Start the high gain period to bring the estimation into convergence
            attitudeResetTimeEnd = currentTimeUs + ATTITUDE_RESET_ACTIVE_TIME;
            gyroQuietPeriodTimeEnd = 0;
        }
    }
    lastArmState = futaba.SwitchA;

    if (attitudeResetActive) {
        ret = ATTITUDE_RESET_KP_GAIN;
    } else {
       ret = dcm_kp;
       if (futaba.SwitchA < SWITCH_DOWN) {
          ret = ret * 10.0f; // Scale the kP to generally converge faster when disarmed.
       }
    }

    return ret;
}

void AHRS::CalculateEstimatedAttitude(uint32_t currentTimeUs)
{
    static uint32_t previousIMUUpdateTime;
    bool useAcc = false;
    bool useMag = false;
    bool useCOG = false; // Whether or not correct yaw via imuMahonyAHRSupdate from our ground course
    float courseOverGround = 0; // To be used when useCOG is true.  Stored in Radians

    const int32_t deltaT = currentTimeUs - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTimeUs;

#ifdef USE_MAG
    if (sensors(SENSOR_MAG) && compassIsHealthy()) {
        useMag = true;
    }
#endif

    float gyroAverage[XYZ_AXIS_COUNT];
    gyro.GetGyroAccumulationAverage(gyroAverage);
    float accAverage[XYZ_AXIS_COUNT];
    if (gyro.GetAccAccumulationAverage(accAverage)) {
        useAcc = IsAccelerometerHealthy(accAverage);
    }

    MahonyAHRSupdate(deltaT * 1e-6f,
                        DEGREES_TO_RADIANS(gyroAverage[X]), DEGREES_TO_RADIANS(gyroAverage[Y]), DEGREES_TO_RADIANS(gyroAverage[Z]),
                        useAcc, accAverage[X], accAverage[Y], accAverage[Z],
                        useMag, 0.f, 0.f, 0.f,
                        useCOG, courseOverGround,  CalcKpGain(currentTimeUs, useAcc, gyroAverage));

    UpdateEulerAngles();
}

void AHRS::UpdateAttitude(void)
{
	uint32_t currentTimeUs = tools.GetMicros();

    if (gyro.isAccelUpdatedAtLeastOnce()) {
        CalculateEstimatedAttitude(currentTimeUs);
    } else {
//        acc.accADC[X] = 0;
//        acc.accADC[Y] = 0;
//        acc.accADC[Z] = 0;
    }
}

bool AHRS::shouldInitializeGPSHeading()
{
    static bool initialized = false;

    if (!initialized) {
        initialized = true;

        return true;
    }

    return false;
}
void AHRS::ComputeRotationMatrix(void){
    QuaternionComputeProducts(&q, &qP);

    rMat[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
    rMat[0][1] = 2.0f * (qP.xy + -qP.wz);
    rMat[0][2] = 2.0f * (qP.xz - -qP.wy);

    rMat[1][0] = 2.0f * (qP.xy - -qP.wz);
    rMat[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
    rMat[1][2] = 2.0f * (qP.yz + -qP.wx);

    rMat[2][0] = 2.0f * (qP.xz + -qP.wy);
    rMat[2][1] = 2.0f * (qP.yz - -qP.wx);
    rMat[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;
}

float AHRS::getCosTiltAngle(void)
{
    return rMat[2][2];
}

void AHRS::getQuaternion(quaternion *quat)
{
   quat->w = q.w;
   quat->x = q.x;
   quat->y = q.y;
   quat->z = q.z;
}

void AHRS::ComputeQuaternionFromRPY(quaternionProducts *quatProd, float initialRoll, float initialPitch, float initialYaw)
{
    if (initialRoll > 180.f) {
        initialRoll -= 360.f;
    }

    if (initialPitch > 180.f) {
        initialPitch -= 360.f;
    }

    if (initialYaw > 180.f) {
        initialYaw -= 360.f;
    }

    const float cosRoll = cos_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);
    const float sinRoll = sin_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);

    const float cosPitch = cos_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);
    const float sinPitch = sin_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);

    const float cosYaw = cos_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);
    const float sinYaw = sin_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);

    const float q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    const float q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    const float q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    const float q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

    quatProd->xx = sq(q1);
    quatProd->yy = sq(q2);
    quatProd->zz = sq(q3);

    quatProd->xy = q1 * q2;
    quatProd->xz = q1 * q3;
    quatProd->yz = q2 * q3;

    quatProd->wx = q0 * q1;
    quatProd->wy = q0 * q2;
    quatProd->wz = q0 * q3;

    ComputeRotationMatrix();
}

void AHRS::QuaternionComputeProducts(quaternion *quat, quaternionProducts *quatProd)
{
    quatProd->ww = quat->w * quat->w;
    quatProd->wx = quat->w * quat->x;
    quatProd->wy = quat->w * quat->y;
    quatProd->wz = quat->w * quat->z;
    quatProd->xx = quat->x * quat->x;
    quatProd->xy = quat->x * quat->y;
    quatProd->xz = quat->x * quat->z;
    quatProd->yy = quat->y * quat->y;
    quatProd->yz = quat->y * quat->z;
    quatProd->zz = quat->z * quat->z;
}

bool AHRS::QuaternionHeadfreeOffsetSet(void)
{
    if ((ABS(attitude.values.roll) < 45.f)  && (ABS(attitude.values.pitch) < 45.f)) {
        const float yaw = -atan2_approx((+2.0f * (qP.wz + qP.xy)), (+1.0f - 2.0f * (qP.yy + qP.zz)));

        offset.w = cos_approx(yaw/2);
        offset.x = 0;
        offset.y = 0;
        offset.z = sin_approx(yaw/2);

        return true;
    } else {
        return false;
    }
}

void AHRS::QuaternionMultiplication(quaternion *q1, quaternion *q2, quaternion *result)
{
    const float A = (q1->w + q1->x) * (q2->w + q2->x);
    const float B = (q1->z - q1->y) * (q2->y - q2->z);
    const float C = (q1->w - q1->x) * (q2->y + q2->z);
    const float D = (q1->y + q1->z) * (q2->w - q2->x);
    const float E = (q1->x + q1->z) * (q2->x + q2->y);
    const float F = (q1->x - q1->z) * (q2->x - q2->y);
    const float G = (q1->w + q1->y) * (q2->w - q2->z);
    const float H = (q1->w - q1->y) * (q2->w + q2->z);

    result->w = B + (- E - F + G + H) / 2.0f;
    result->x = A - (+ E + F + G + H) / 2.0f;
    result->y = C + (+ E - F + G - H) / 2.0f;
    result->z = D + (+ E - F - G + H) / 2.0f;
}

void AHRS::QuaternionHeadfreeTransformVectorEarthToBody(t_fp_vector_def *v)
{
    quaternionProducts buffer;

    QuaternionMultiplication(&offset, &q, &headfree);
    QuaternionComputeProducts(&headfree, &buffer);

    const float x = (buffer.ww + buffer.xx - buffer.yy - buffer.zz) * v->X + 2.0f * (buffer.xy + buffer.wz) * v->Y + 2.0f * (buffer.xz - buffer.wy) * v->Z;
    const float y = 2.0f * (buffer.xy - buffer.wz) * v->X + (buffer.ww - buffer.xx + buffer.yy - buffer.zz) * v->Y + 2.0f * (buffer.yz + buffer.wx) * v->Z;
    const float z = 2.0f * (buffer.xz + buffer.wy) * v->X + 2.0f * (buffer.yz - buffer.wx) * v->Y + (buffer.ww - buffer.xx - buffer.yy + buffer.zz) * v->Z;

    v->X = x;
    v->Y = y;
    v->Z = z;
}
void AHRS::OrientationReset(void){
	q.w = 1; q.x = 0; q.y = 0; q.z = 0;
	qP.ww = 1; qP.wx = 0; qP.wy = 0; qP.wz = 0; qP.xx = 0; qP.xy = 0; qP.xz = 0; qP.yy = 0; qP.yz = 0; qP.zz = 0;
    ComputeRotationMatrix();
	ResetAccelerationSum();
}
