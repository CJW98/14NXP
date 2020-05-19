#ifndef _FASTMATH_H
#define _FASTMATH_H


#define RADTODEG(x) ((x) * 57.295779513082320876798154814105f)
#define DEGTORAD(x) ((x) * 0.01745329251994329576923690768489f)

//translate from the DSP instruction of a DSP Library.
#ifndef PI
#define PI (3.1415926535897932384626433832795f)
#endif
#define PI_2 (1.5707963267948966192313216916398f)
#define PI_3 (1.0471975511965977461542144610932f)
#define PI_4 (0.78539816339744830961566084581988f)
#define PI_6 (0.52359877559829887307710723054658f)
#define TWO_MINUS_ROOT3 (0.26794919243112270647255365849413f)
#define SQRT3_MINUS_1 (0.73205080756887729352744634150587f)
#define SQRT3 (1.7320508075688772935274463415059f)
#define EPS_FLOAT (+3.452669830012e-4f)

float FastSqrtI(float x);
float FastSqrt(float x);
float FastSin(float x);
float FastCos(float x);
float FastAsin(float x);
float FastAtan2(float y, float x);
//////////////////////////////////////////////////////////////////////////
#define X_MAX (+9.099024257348e3f)
#define INV_PI_2 ( 0.63661977236758134307553505349006f)
#define PI_2_C1             ( 1.5703125f)
#define PI_2_C2             ( 4.84466552734375e-4f)
#define PI_2_C3 (-6.39757837755768678308360248557e-7f)

#define TANP_COEF1    (-1.113614403566e-1f)
#define TANP_COEF2    (+1.075154738488e-3f)
#define TANQ_COEF0    (+1.000000000000f)
#define TANQ_COEF1    (-4.446947720281e-1f)
#define TANQ_COEF2    (+1.597339213300e-2f)
float FastTan(float x);
//////////////////////////////////////////////////////////////////////////
//Coefficients used for atan/atan2
#define ATANP_COEF0 (-1.44008344874f)
#define ATANP_COEF1 (-7.20026848898e-1f)
#define ATANQ_COEF0 (+4.32025038919f)
#define ATANQ_COEF1 (+4.75222584599f)
//Coefficients used for asin/acos
#define ASINP_COEF1 (-2.7516555290596f)
#define ASINP_COEF2 (+2.9058762374859f)
#define ASINP_COEF3 (-5.9450144193246e-1f)
#define ASINQ_COEF0 (-1.6509933202424e+1f)
#define ASINQ_COEF1 (+2.4864728969164e+1f)
#define ASINQ_COEF2 (-1.0333867072113e+1f)

//FastAbs与abs都占用非常少的时间
inline float FastAbs(float x){
	union { unsigned int i; float f;} y;
	y.f = x;
	y.i = y.i & 0x7FFFFFFF;
	return (float)y.f;
}

#endif



