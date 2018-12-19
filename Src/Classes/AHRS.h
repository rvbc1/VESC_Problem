/*
 * AHRS.h
 *
 *  Created on: 21.06.2018
 *      Author: mice
 */

#ifndef _AHRS_H_
#define _AHRS_H_

#include "stdint.h"
#include "axis.h"
#include "Mathematics.h"

typedef struct {
    float w,x,y,z;
} quaternion;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

typedef struct {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

typedef union {
	float raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination
        float roll;
        float pitch;
        float yaw;
    } values;
} attitudeEulerAngles_t;
#define EULER_INITIALIZE  { { 0.f, 0.f, 0.f } }


typedef struct accDeadband_s {
    uint8_t xy;                 // set the acc deadband for xy-Axis
    uint8_t z;                  // set the acc deadband for z-Axis, this ignores small accelerations
} accDeadband_t;


class AHRS {
	int32_t Signal_AHRS = 1 << 0;

	float dcm_kp = 0.2500f;
	float dcm_ki = 0.0f;

	// Exported symbols
	int32_t accSum[XYZ_AXIS_COUNT];

	uint32_t accTimeSum = 0;        // keep track for integration of acc
	int accSumCount = 0;
	float accVelScale;

	bool canUseGPSHeading = true;

	float rMat[3][3];

	// quaternion of sensor frame relative to earth frame
	quaternion q = QUATERNION_INITIALIZE;
	quaternionProducts qP = QUATERNION_PRODUCTS_INITIALIZE;
	// headfree quaternions
	quaternion headfree = QUATERNION_INITIALIZE;
	quaternion offset = QUATERNION_INITIALIZE;


	void UpdateAttitude(void);

	void ComputeRotationMatrix(void);
	void MahonyAHRSupdate(	float dt, 	float gx, float gy, float gz,
							bool useAcc, float ax, float ay, float az,
							bool useMag, float mx, float my, float mz,
							bool useCOG, float courseOverGround, const float dcmKpGain);
	void CalculateEstimatedAttitude(uint32_t currentTimeUs);
	void UpdateEulerAngles(void);
	void ResetAccelerationSum(void);

	bool IsAccelerometerHealthy(float *accAverage);
	float CalcKpGain(uint32_t currentTimeUs, bool useAcc, float *gyroAverage);

	float getCosTiltAngle(void);

	void QuaternionMultiplication(quaternion *q1, quaternion *q2, quaternion *result);
	void QuaternionComputeProducts(quaternion *quat, quaternionProducts *quatProd);
	bool QuaternionHeadfreeOffsetSet(void);
	void QuaternionHeadfreeTransformVectorEarthToBody(t_fp_vector_def * v);
	void ComputeQuaternionFromRPY(quaternionProducts *qP, float initialRoll, float initialPitch, float initialYaw);
	bool shouldInitializeGPSHeading(void);
public:
	// absolute angle inclination
	attitudeEulerAngles_t attitude = EULER_INITIALIZE;;

	void Init(void);
	void Process(void);
	void TaskNotify(void);

	void getQuaternion(quaternion * q);
	void OrientationReset(void);
	AHRS();
	virtual ~AHRS();
};
extern AHRS ahrs;
#endif /* _AHRS_H_ */
