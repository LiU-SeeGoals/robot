#ifndef STATE_ESTIMATOR2_H
#define STATE_ESTIMATOR2_H

#include "arm_math.h"
#include "imu.h"
#include "lag_element.h"

/*
	// EKF state vector: [ px py pw vx vy ]
	// input vector: [ gyr_w acc_x acc_y ]
	// measurement vector: [ px py pw ]
  
  Input enters through the prediction update and
  measurements enter via measurement update.

  GLOBAL FRAME COORDINATES (Football field): px, py, pw
  ROBOT FRAME COORDINATES (Center of robot seen from above): vx, vy
  IMU FRAME COORDINATES: (Center of imu seen from above) gyr_z, acc_x, acc_y
  // NOTE: IMU might not be perfectly aligned with the robot,
  // estimating the orientation of IMU will likely improve estimates

*/

#define EKF_SIZE_A(f)		(f*f)
#define EKF_SIZE_C(h,f)		(h*f)
#define EKF_SIZE_EX(f)		(f*f)
#define EKF_SIZE_EZ(h)		(h*h)
#define EKF_SIZE_X(f)		(f)
#define EKF_SIZE_SIGMA(f)	(f*f)
#define EKF_SIZE_K(f,h)		(f*h)
#define EKF_SIZE_U(g)		(g)
#define EKF_SIZE_Z(h)		(h)
#define EKF_SIZE_MAX(max)	(max*max)

// state vector (x) rows: f
// control vector (u) rows: g
// sensor vector (z) rows: h
#define EKF_DATA_SIZE(f,g,h,max) \
	(EKF_SIZE_A(f)+EKF_SIZE_C(h,f)+EKF_SIZE_EX(f) \
	+EKF_SIZE_EZ(h)+EKF_SIZE_X(f)+EKF_SIZE_SIGMA(f)+EKF_SIZE_K(f,h)+EKF_SIZE_U(g) \
	+EKF_SIZE_Z(h)+EKF_SIZE_MAX(max)*3)

#define MAT_ELEMENT(mat,r,c) ((mat).pData[(mat).numCols*(r)+(c)])

typedef void(*EKFStateFunc)(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU);
typedef void(*EKFStateJacobianFunc)(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF);
typedef void(*EKFMeasFunc)(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY);
typedef void(*EKFMeasJacobianFunc)(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH);

// This EKF struct is stolen so dont change it
typedef struct _EKF

{
	uint16_t f;
	uint16_t g;
	uint16_t h;

	// state and measurement functions
	EKFStateFunc pState;
	EKFStateJacobianFunc pStateJacobian;
	EKFMeasFunc pMeas;
	EKFMeasJacobianFunc pMeasJacobian;

	// user matrices
	arm_matrix_instance_f32 A;		// (f x f)
	arm_matrix_instance_f32 C;		// (h x f)
	arm_matrix_instance_f32 Ex;		// (f x f)
	arm_matrix_instance_f32 Ez;		// (h x h)

	// internal matrices
	arm_matrix_instance_f32 x;		// state (f x 1)
	arm_matrix_instance_f32 Sigma;	// uncertainty (f x f)
	arm_matrix_instance_f32 K;		// Kalman gain (f x h)

	// command input
	arm_matrix_instance_f32 u;		// (g x 1)

	// sensor input
	arm_matrix_instance_f32 z;		// (h x 1)

	// temporary calculation matrices
	arm_matrix_instance_f32 tmp1;
	arm_matrix_instance_f32 tmp2;
	arm_matrix_instance_f32 tmp3;
} EKF;

typedef struct _FusionEKF
{
	// EKF state vector: [ px py pw vx vy ]
	// input vector: [ gyr_w acc_x acc_y ]
	// measurement vector: [ px py pw ]
	EKF ekf;
	float ekfData[EKF_DATA_SIZE(5, 3, 3, 5)];
	/*float imu_dt;*/

  struct
  {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
    uint16_t is_calibrated;

  } bias;

	/*float encGyrPos[3];*/

	struct
	{
		uint16_t online;
		/*uint32_t timeLastValidSample;*/
		/*int32_t turns;*/
		/*float lastOrient;*/
		/*uint32_t numLateMeasurements;*/
	} vision;

	/*FusionEKFConfig* pConfig;*/

	/*FusionEKFTimeSlot timeSlots[FUSION_EKF_MAX_DELAY];*/
	/*uint32_t timeSlotNow;*/

	/*ModelEnc modelEnc;*/

	LagElementPT1 lagAccel[2];

	/*struct*/
	/*{*/
	/*	LagElementPT1 lagCurrent;*/
	/**/
	/*	uint8_t isStrongDribbling;*/
	/*	uint32_t strongTicks;*/
	/**/
	/*	FusionEKFDribblerIdleCurTable idleCurrentTable;*/
	/*	FlashFile* pIdleCurrentFile;*/
	/*} dribbler;*/

	/*float ballPosGlobal[2];*/
	/*float ballVelGlobal[2];*/
	/*uint32_t ballLastDetectedTimestamp;*/
} FusionEKF;


// Public functions
void STATE_Init();
void STATE_Test();
void STATE_FusionEKFVisionUpdate(float posx, float posy, float posw);
void STATE_FusionEKFIntertialUpdate(IMU_AccelVec3 acc, IMU_GyroVec3 gyr);
void STATE_calibrate_imu_gyr();
uint16_t STATE_is_calibrated();
float STATE_get_robot_angle();
float STATE_get_posx();
float STATE_get_posy();
float STATE_get_vx();
float STATE_get_vy();
void STATE_log_states();

// Private functions
static void ekfStateJacobianFunc(const arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU, arm_matrix_instance_f32* pF);
static void ekfMeasFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pY);
static void ekfStateFunc(arm_matrix_instance_f32* pX, const arm_matrix_instance_f32* pU);
static void ekfMeasJacobianFunc(const arm_matrix_instance_f32* pX, arm_matrix_instance_f32* pH);
static void EKFInit(EKF* pKF, uint16_t numStates, uint16_t numCtrl, uint16_t numSensors, float* pData);
static void initEKF();

#endif
