/*
 *  BOSCH REXROTH
 *
 *  Copyright (c) Bosch Rexroth AG 2017
 *  Internet: http://www.boschrexroth.com
 *
 *  Product Name:   NYCe4000
 *  Component Name: Examples
 */

/**
 *  @file
 *  @brief  Defines the arguments for starting my_udsx.c.
 */

#ifndef _MY_UDSX_INTERFACE_H_
#define _MY_UDSX_INTERFACE_H_

#include <n4k_basictypes.h>
#include <nycedefs.h>
#include <nhivariables.h>

/* Helper macros */
#define USR_ERROR(x)    (NYCE_STATUS)(NYCE_ERROR_MASK | (N4K_SS_USR << NYCE_SUBSYS_SHIFT) | (x))
#define USR_WARNING(x)  (NYCE_STATUS)(NYCE_OK_MASK    | (N4K_SS_USR << NYCE_SUBSYS_SHIFT) | (x))


/* My udsx user errors */

/* My UDSX UdsxInitialize detected an invalid argument */
#define MY_UDSX_ERR_INVALID_ARG             USR_ERROR(100)

/* My UDSX cannot run: analog input value too low */
#define MY_UDSX_ERR_AN_IN_LEVEL_TOO_LOW     USR_ERROR(101)

/* My UDSX runs, but the analog input value is low */
#define MY_UDSX_WRN_AN_IN_LEVEL_LOW         USR_WARNING(201)


/**
 * @brief   Argument for UdsxInitialize in my_udsx.c
 *          This is a shared interface between the UDSX and the application which starts the UDSX.
 */
typedef struct my_udsx_args
{
    NHI_VAR_ID          anInputvarId;           /**< Selected analog input, for example NHI_VAR_AN_IN0_VALUE_SLOT0.
                                                 *   If this variable does not define an analog input, starting the UDSX
                                                 *   returns MY_UDSX_ERR_INVALID_ARG.
                                                 */

    double              anInErrorThreshold;     /**< If the analog input is lower than this value,
                                                 *   starting the UDSX returns MY_UDSX_ERR_AN_IN_LEVEL_TOO_LOW.
                                                 */

    double              anInWarningThreshold;   /**< If the analog input is betweeen anInErrorThreshold and this value,
                                                 *   starting the UDSX returns the warning MY_UDSX_WRN_AN_IN_LEVEL_LOW.
                                                 *   Must not be lower than anInErrorThreshold, otherwise starting the UDSX
                                                 *   returns MY_UDSX_ERR_INVALID_ARG.
                                                 */

} MY_UDSX_ARGS;

typedef struct axis_setting{
	int					Shared_AxisType[10];
	char				Shared_AxisName0[20];
	char				Shared_AxisName1[20];
	char				Shared_AxisName2[20];
	char				Shared_AxisName3[20];
	char				Shared_AxisName4[20];
	char				Shared_AxisName5[20];
	char				Shared_AxisName6[20];
	char				Shared_AxisName7[20];
	char				Shared_AxisName8[20];
	char				Shared_AxisName9[20];
}AXIS_SETTING;


#define		SHMEM_AREA		2

typedef struct shmem_data
{
	unsigned int		Shared_StatFlag[10];
	float				Shared_CtrFlag[80];
	double				Shared_SetPointPos[20];
	int					Shared_AxisType[10];
	char				Shared_AxisName0[20];
	char				Shared_AxisName1[20];
	char				Shared_AxisName2[20];
	char				Shared_AxisName3[20];
	char				Shared_AxisName4[20];
	char				Shared_AxisName5[20];
	char				Shared_AxisName6[20];
	char				Shared_AxisName7[20];
	char				Shared_AxisName8[20];
	char				Shared_AxisName9[20];
	unsigned int		STAT_FLG[10];
	float 				VC_POS[20];
	float 				FORCE_LIMIT[10];
	float 				NET_CURRENT[10];
} SHMEM_DATA;

struct ThreadArgs {
	int clntSock;
};



float F_CMD_FLG_1[10];
float F_CTR_FLG_1[80];
char  F_AXS1_NAM_0[20];
char  F_AXS1_NAM_1[20];
char  F_AXS1_NAM_2[20];
char  F_AXS1_NAM_3[20];
char  F_AXS1_NAM_4[20];
char  F_AXS1_NAM_5[20];
char  F_AXS1_NAM_6[20];
char  F_AXS1_NAM_7[20];
char  F_AXS1_NAM_8[20];
char  F_AXS1_NAM_9[20];
int   F_AXS_TYPE_1[10];

char  TempName0[20];
char  TempName1[20];
char  TempName2[20];
char  TempName3[20];
char  TempName4[20];
char  TempName5[20];
char  TempName6[20];
char  TempName7[20];
char  TempName8[20];
char  TempName9[20];
int   TempAxType;

float LAST_VC_POS[20];
unsigned int LAST_STAT_FLG[10];

#define CMD_FLG     F_CMD_FLG_1
#define CTR_FLG		F_CTR_FLG_1
#define AXS_NAM0	F_AXS1_NAM_0
#define AXS_NAM1	F_AXS1_NAM_1
#define AXS_NAM2	F_AXS1_NAM_2
#define AXS_NAM3	F_AXS1_NAM_3
#define AXS_NAM4	F_AXS1_NAM_4
#define AXS_NAM5	F_AXS1_NAM_5
#define AXS_NAM6	F_AXS1_NAM_6
#define AXS_NAM7	F_AXS1_NAM_7
#define AXS_NAM8	F_AXS1_NAM_8
#define AXS_NAM9	F_AXS1_NAM_9
#define AXS_TYPE	F_AXS_TYPE_1



enum{
	E_NO_CMD,
	E_CMD_FLG,
	E_CTR_FLG,
	E_AXS_NAM0,
	E_AXS_NAM1,
	E_AXS_NAM2,
	E_AXS_NAM3,
	E_AXS_NAM4,
	E_AXS_NAM5,
	E_AXS_NAM6,
	E_AXS_NAM7,
	E_AXS_NAM8,
	E_AXS_NAM9,
	E_AXS_TYPE,

	E_FORCE_LIMIT,
	E_NET_CURRENT,
	E_STAT_FLG,
	E_VC_POS,

	E_NYCE_INIT,
	E_NYCE_STOP,

	E_PING = 4114,

};

#define MAXPENDING 5                   /* Maximum outstanding connection requests */
#define PORT 6666
#define MAX_BUFFER_SIZE 2000


/// on the force control state
#define TURRET		0
#define VC_PUSHER	1
#define STD_ABS		2
#define STD_REL		3
#define NA			9

#define Tweak_Compensation	0
#define Current_Drive		1
#define Setpoint_Pos		2
#define Axis_Pos			3
#define Axis_State			4
#define	Axis_SPG_State		5

SAC_AXIS sacAxis[10];
UDSX_RW_HANDLE hndRWData[10][5];

int Axis_Type[10];
char Axis_Name[10][20];
int ReqParaInit[10];
int SacConnected[10];
int ScanForceCtr[10];
int ScanForceRate;
double dSatLevelCheck;
double dSatLevelType;

int ax;
int DEBUG_MODE;
int CUR_LIM_MODE[10];
int CUR_STAB_MODE[10];
int StabilizeCycle[10];
int StabilizeCnt;
int ParaInit[10];

int OpenLoopStab[10];
double InterpolationPos[10];
double oldturretpos;
int dPosOffset;

double DRIVE_CURRENT[10];
double SAT_LEVEL[10];
double OLD_SAT_LEVEL[10];
double MAX_DYN_ERR[10];
double dOrgSatLevel[10];
//double dOrgDynError[10];
double dCurStabilize[10];
double dTweakValue[10];

double Flag270Deg;
double Flag110Deg;

float FORCE_THRESHOLD;
float DP_THRESHOLD;
float LINEAR_THRESHOLD;
float STANDBY_POS[10];

NYCE_STATUS StatusSConnect[10];
NYCE_STATUS StatusReadForceOri[10];
NYCE_STATUS StatusReadDynOri[10];
NYCE_STATUS StatusWriteForce[10];
NYCE_STATUS StatusWriteDyn[10];
NYCE_STATUS StatusWriteForceOri[10];
NYCE_STATUS StatusWriteDynOri[10];
NYCE_STATUS StatusSDisconnect[10];

NYCE_STATUS StatusReadTwe[10];
NYCE_STATUS StatusReadCur[10];
NYCE_STATUS StatusReadSet[10];
NYCE_STATUS StatusReadPos[10];
NYCE_STATUS StatusReadStat[10];

NYCE_STATUS StatusOpenDTwe[10];
NYCE_STATUS StatusOpenDCur[10];
NYCE_STATUS StatusOpenDSet[10];
NYCE_STATUS StatusOpenDPos[10];
NYCE_STATUS StatusOpenDSta[10];
NYCE_STATUS StatusOpenDSpg[10];

NYCE_STATUS StatusCloseDTwe[10];
NYCE_STATUS StatusCloseDCur[10];
NYCE_STATUS StatusCloseDPos[10];
NYCE_STATUS StatusCloseDSet[10];
NYCE_STATUS StatusCloseDSta[10];
NYCE_STATUS StatusCloseDSpg[10];

SAC_STATE AXIS_STATE[10];
SAC_STATE OLD_AXIS_STATE[10];
SAC_SPG_STATE AXIS_SPG_STATE[10];

double axis_state;
double axis_spg_state;

double dCurrentDrive;
double dPos;
double dSetpointPos;

void ChkTurStat (int AxisID);
void ChkVCStat (int AxisID);
void ChkStdStat (int AxisID);

int NyceDisconnectAxis(void);


#endif
