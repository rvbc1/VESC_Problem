/*
 * Mathematics.h
 *
 *  Created on: 14.01.2018
 *      Author: mice
 */

/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef COMMON_MATHEMATICS_H_
#define COMMON_MATHEMATICS_H_

#ifdef __cplusplus
 extern "C" {
#endif

//#define VERY_FAST_MATH

#ifndef sq
#define sq(x) ((x)*(x))
#endif

#define M_PI_FLOAT 3.14159265358979323846f

#define RAD    (M_PI_FLOAT / 180.0f)
#define RADIANS_TO_DEGREES(angle) ((angle) * 180.f / M_PI_FLOAT)
#define DEGREES_TO_DECIDEGREES(angle) ((angle) * 10)
#define DECIDEGREES_TO_DEGREES(angle) ((angle) / 10)
#define DECIDEGREES_TO_RADIANS(angle) ((angle) / 10.0f * 0.0174532925f)
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#define ABS(x) ((x) > 0 ? (x) : -(x))

#define SIGNF(x) ((x > 0.f) ? 1.f : ((x < 0.f) ? -1.f : 0.f))

#define Q12 (1 << 12)

typedef int32_t fix12_t;

typedef struct stdev_s
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

// Floating point 3 vector.
typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union u_fp_vector {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

// Floating point Euler angles.
// Be carefull, could be either of degrees or radians.
typedef struct fp_angles {
    float roll;
    float pitch;
    float yaw;
} fp_angles_def;

typedef union {
    float raw[3];
    fp_angles_def angles;
} fp_angles_t;

float powerf(float base, int exp);
int32_t applyDeadband(int32_t value, int32_t deadband);

void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devVariance(stdev_t *dev);
float devStandardDeviation(stdev_t *dev);
float degreesToRadians(int16_t degrees);

int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax);

void normalizeV(struct fp_vector *src, struct fp_vector *dest);

void rotateV(struct fp_vector *v, fp_angles_t *delta);
void buildRotationMatrix(fp_angles_t *delta, float matrix[3][3]);

int32_t quickMedianFilter3(int32_t * v);
int32_t quickMedianFilter5(int32_t * v);
int32_t quickMedianFilter7(int32_t * v);
int32_t quickMedianFilter9(int32_t * v);

float quickMedianFilter3f(float * v);
float quickMedianFilter5f(float * v);
float quickMedianFilter7f(float * v);
float quickMedianFilter9f(float * v);

#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
float sin_approx(float x);
float cos_approx(float x);
float atan2_approx(float y, float x);
float acos_approx(float x);
#define tan_approx(x)       (sin_approx(x) / cos_approx(x))
#else
#define sin_approx(x)   sinf(x)
#define cos_approx(x)   cosf(x)
#define atan2_approx(y,x)   atan2f(y,x)
#define acos_approx(x)      acosf(x)
#define tan_approx(x)       tanf(x)
#endif

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count);

int16_t qPercent(fix12_t q);
int16_t qMultiply(fix12_t q, int16_t input);
fix12_t qConstruct(int16_t num, int16_t den);

static inline int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

static inline float constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}
uint16_t crc16_ccitt(uint16_t crc, unsigned char a);
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);

#ifdef __cplusplus
 }
#endif


#endif /* COMMON_MATHEMATICS_H_ */