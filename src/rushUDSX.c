/*
 *  Exis Tech
 *
 *	Company name : Exis Tech Sdn. bhd.
 *
 *	Author : Muhammad Rushdi bin Mohd Rasid
 *  Product Name:   NYCe4000
 *  Component Name: rushNodeSeq and rushNodeSeq.so
 */

/**
 *  @file
 *  @brief  establish comm server, manage and execute commands at the NYCE4114 and return status
 *
 *  To compile the udsx
 *  compile with:
 *      arm-rexroth-linux-gnueabihf-gcc -DSO -c rushUDSX.c -o rushUDSX.so.o
 *  link with:
 *      arm-rexroth-linux-gnueabihf-gcc -o librushUDSX.so rushUDSX.so.o -ludsx -lsac -lpthread -lrt -fPIC -shared
 *
 *  All output files should be put at /home/user/ at the nyce sequencer
 *  please put .SO at the output file if compiling the udsx
 *  .SO is a position independent code -fPIC, shared library -shared
 */

#define SO

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>           /* For O_* constants */
#include <sactypes.h>
#include "sysapi.h"
#include "udsxapi.h"


/* Include MY_UDSX_ARGS type and USR error codes */
#include "rushUDSX.h"

/*
 * Name of the shared memory file which is located in /dev/shm.
 */
#define SHM_NAME "mycounter"

#define UNUSED(x) (void)(x)

/*
 * User defined errors.
 */
#define USR_ERR_FAILED_TO_CREATE_SHM        ((NYCE_STATUS)((NYCE_ERROR_MASK)|((N4K_SS_USR<<NYCE_SUBSYS_SHIFT)|0)))
#define USR_ERR_FAILED_TO_RESIZE_SHM        ((NYCE_STATUS)((NYCE_ERROR_MASK)|((N4K_SS_USR<<NYCE_SUBSYS_SHIFT)|1)))
#define USR_ERR_FAILED_TO_MAP_SHM           ((NYCE_STATUS)((NYCE_ERROR_MASK)|((N4K_SS_USR<<NYCE_SUBSYS_SHIFT)|2)))
#if defined(SO)
#define USR_ERR_NOT_INITIALIZED             ((NYCE_STATUS)((NYCE_ERROR_MASK)|((N4K_SS_USR<<NYCE_SUBSYS_SHIFT)|3)))
#endif

/*
 * Administration of the shared memory.
 */
int         g_sharedMemoryDescriptor = -1;  // Descriptor to the file.
SHMEM_DATA* pShmem_data = NULL;              // Pointer to the shared memory data.
BOOL        g_created = FALSE;              // Flag to remember whether or not the shared memory was created by this process.

/**
 *  @brief  Initialize the shared memory.
 *
 *  @param[in]  create       Whether or not the application may create the shared memory file if it does not exist.
 *  @return     Status of the success of creating the shared memory.
 */
static NYCE_STATUS Initialize(BOOL create);

/**
 *  @brief  Terminate the shared memory.
 */
static void Terminate(void);

#if defined(SO)
#include <udsxapi.h>

NYCE_STATUS g_retVal = NYCE_OK;//= USR_ERR_NOT_INITIALIZED;

NYCE_STATUS UdsxInitialize(const void* argument, uint32_t argumentSize)
{

	AXIS_SETTING* axisSetting = (AXIS_SETTING*)argument; // not using this
		//-------------------------
		//		Axis Naming
		//-------------------------
				strcpy(Axis_Name[0],(char*)&axisSetting->Shared_AxisName0);
				strcpy(Axis_Name[1],(char*)&axisSetting->Shared_AxisName1);
				strcpy(Axis_Name[2],(char*)&axisSetting->Shared_AxisName2);
				strcpy(Axis_Name[3],(char*)&axisSetting->Shared_AxisName3);
				strcpy(Axis_Name[4],(char*)&axisSetting->Shared_AxisName4);
				strcpy(Axis_Name[5],(char*)&axisSetting->Shared_AxisName5);
				strcpy(Axis_Name[6],(char*)&axisSetting->Shared_AxisName6);
				strcpy(Axis_Name[7],(char*)&axisSetting->Shared_AxisName7);
				strcpy(Axis_Name[8],(char*)&axisSetting->Shared_AxisName8);
				strcpy(Axis_Name[9],(char*)&axisSetting->Shared_AxisName9);


				strcpy((char*)&pShmem_data->Shared_AxisName0,(char*)&axisSetting->Shared_AxisName0);
				strcpy((char*)&pShmem_data->Shared_AxisName1,(char*)&axisSetting->Shared_AxisName1);
				strcpy((char*)&pShmem_data->Shared_AxisName2,(char*)&axisSetting->Shared_AxisName2);
				strcpy((char*)&pShmem_data->Shared_AxisName3,(char*)&axisSetting->Shared_AxisName3);
				strcpy((char*)&pShmem_data->Shared_AxisName4,(char*)&axisSetting->Shared_AxisName4);
				strcpy((char*)&pShmem_data->Shared_AxisName5,(char*)&axisSetting->Shared_AxisName5);
				strcpy((char*)&pShmem_data->Shared_AxisName6,(char*)&axisSetting->Shared_AxisName6);
				strcpy((char*)&pShmem_data->Shared_AxisName7,(char*)&axisSetting->Shared_AxisName7);
				strcpy((char*)&pShmem_data->Shared_AxisName8,(char*)&axisSetting->Shared_AxisName8);
				strcpy((char*)&pShmem_data->Shared_AxisName9,(char*)&axisSetting->Shared_AxisName9);

	 	//-------------------------
	 	//		Axis Type
	 	//-------------------------
	    for ( ax = 0; ax < 10; ax++ )
	    {
			Axis_Type[ax] = axisSetting->Shared_AxisType[ax];
			pShmem_data->Shared_AxisType[ax] = axisSetting->Shared_AxisType[ax];
		}

		ax = 0;
		DEBUG_MODE = 0;
		ScanForceRate = 10;

		FORCE_THRESHOLD = 3000;
		DP_THRESHOLD = 150;
		LINEAR_THRESHOLD = 800;

	    for ( ax = 0; ax < 10; ax++ )
	    {
			STANDBY_POS[ax] = 1000;
		}

		StabilizeCnt = 40;
		//-----------------
	    // Connect the axes
	    //----------------
	    for ( ax = 0; ax < 10; ax++ )
	    {
			if (Axis_Type[ax] != NA)
			{
				StatusSConnect[ax] = SacConnect(Axis_Name[ax], &sacAxis[ax]);

				if (StatusSConnect[ax] == 0)
				{
					SacConnected[ax] = 255;
				}
			}
		}

	    // ------------------
	    // Open data channels
	    // ------------------
	    for ( ax = 0; ax < 10; ax++ )
	    {
	    	pShmem_data->FORCE_LIMIT[ax] = 0.38;
	    	DRIVE_CURRENT[ax] = 0;
			ScanForceCtr[ax] = 0;

			pShmem_data->STAT_FLG[ax] = 0x02;

			pShmem_data->VC_POS[ax] = 0;
			pShmem_data->VC_POS[ax + 10] = 0;

			dOrgSatLevel[ax] = 8.5;
			//dOrgDynError[ax] = 0;
			ParaInit[ax] = 0;

			if (SacConnected[ax] == 255)
			{
				StatusOpenDTwe[ax] = UdsxOpenAxisData(sacAxis[ax], SAC_VAR_TWEAK_COMPENSATION, &hndRWData[ax][Tweak_Compensation]);
				StatusOpenDSet[ax] = UdsxOpenAxisData(sacAxis[ax], SAC_VAR_SETPOINT_POS, &hndRWData[ax][Setpoint_Pos]);
				StatusOpenDCur[ax] = UdsxOpenAxisData(sacAxis[ax], SAC_VAR_DIRECT_CURRENT, &hndRWData[ax][Current_Drive]);
				StatusOpenDPos[ax] = UdsxOpenAxisData(sacAxis[ax], SAC_VAR_AXIS_POS, &hndRWData[ax][Axis_Pos]);
				StatusOpenDSta[ax] = UdsxOpenAxisData(sacAxis[ax], SAC_VAR_STATE, &hndRWData[ax][Axis_State]);
				StatusOpenDSpg[ax] = UdsxOpenAxisData(sacAxis[ax], SAC_VAR_SPG_STATE, &hndRWData[ax][Axis_SPG_State]);
			}
	    }

    return NYCE_OK;
}

void UdsxExecuteAtSampleStart(void)
{
    /*
     * The UdsxExecuteAtSampleStart is not needed for this example,
     * but an empty function is used to prevent a warning from NhiUdsxStart.
     */
}
void UdsxExecuteAtSampleEnd(void)
{

if (pShmem_data)
{
	pShmem_data->udsx_enter = 1;
	DP_THRESHOLD = pShmem_data->Shared_CtrFlag[11];

	if (pShmem_data->Shared_CtrFlag[10] == 1.00)
	{
		Flag110Deg = 1700;
	}
	else
	{
		Flag110Deg = pShmem_data->Shared_CtrFlag[12];
	}

	LINEAR_THRESHOLD = Flag110Deg;
	FORCE_THRESHOLD = pShmem_data->Shared_CtrFlag[14];
	ScanForceRate = pShmem_data->Shared_CtrFlag[15];

	for ( ax = 0; ax < 10; ax++)
	{
		STANDBY_POS[ax] = pShmem_data->Shared_CtrFlag[ax + 60];
	}

	for ( ax = 0; ax < 10; ax++)
	{
		if (SacConnected[ax] == 255)
		{
			//StatusReadPos[ax] = SqcReadData( &hndRWData[ax][Axis_Pos], &dPos);
			//StatusReadSet[ax] = SqcReadData( &hndRWData[ax][Setpoint_Pos], &dSetpointPos);

			//StatusReadStat[ax] = SacReadState(sacAxis[ax],&AXIS_STATE[ax],&AXIS_SPG_STATE[ax]);
			StatusReadStat[ax] = UdsxReadData(&hndRWData[ax][Axis_State], &axis_state);
			StatusReadStat[ax] &= UdsxReadData(&hndRWData[ax][Axis_SPG_State], &axis_spg_state);

			AXIS_STATE[ax] = (SAC_STATE)axis_state; // convert to type
			AXIS_SPG_STATE[ax] = (SAC_STATE)axis_spg_state; // convert to type


			//pShmem_data->Shared_SetPointPos[ax] = dSetpointPos;		//Pass setpoint pos back to NodeSeq for motion profile calculation
			//pShmem_data->VC_POS[(ax * 2) + 1] = dSetpointPos;
			//pShmem_data->VC_POS[ax * 2] = dPos;

			//--------------------------------
			// Read encoder and setpoint
			// -------------------------------

			StatusReadPos[ax] = UdsxReadData( &hndRWData[ax][Axis_Pos], &dPos);
			StatusReadSet[ax] = UdsxReadData( &hndRWData[ax][Setpoint_Pos], &dSetpointPos);


			pShmem_data->Shared_SetPointPos[ax] = dSetpointPos;		//Pass setpoint pos back to NodeSeq for motion profile calculation
			pShmem_data->VC_POS[(ax * 2) + 1] = dSetpointPos;
			pShmem_data->VC_POS[ax * 2] = dPos;


			switch(Axis_Type[ax])
			{
			case TURRET:
				ChkTurStat(ax);
				break;

			case VC_PUSHER:
				if (AXIS_STATE[ax] == SAC_IDLE)
				{
					ParaInit[ax] = 0;
				}
				else
				{
					if (ParaInit[ax] != 255)
					{
						ParaInit[ax] = 255;
						//StatusReadForceOri[ax] = SacReadParameter(sacAxis[ax], SAC_PAR_SAT_LEVEL, &dOrgSatLevel[ax]);
					}
				}
				ChkVCStat(ax);
				break;

			case STD_ABS:
				ChkStdStat(ax);
				break;

			case STD_REL:
				ChkStdStat(ax);
				break;
			}

			pShmem_data->STAT_FLG[ax] = pShmem_data->Shared_StatFlag[ax];
		}
	}

	for ( ax = 0; ax < 10; ax++)
	{
		//if ((SacConnected[ax] == 255) && (Axis_Type[ax] == VC_PUSHER) && (ScanForceCtr[ax] == 0))
		if ((SacConnected[ax] == 255) && (Axis_Type[ax] == VC_PUSHER))
		{
			if (pShmem_data->FORCE_LIMIT[ax] > 0)
			{
				//if ((pShmem_data->VC_POS[(ax * 2) + 1] > FORCE_THRESHOLD) || (AXIS_SPG_STATE[ax] == SAC_SPG_DISABLED))
				if ((pShmem_data->VC_POS[(ax * 2) + 1] > (pShmem_data->Shared_CtrFlag[ax] - FORCE_THRESHOLD)) || (AXIS_SPG_STATE[ax] == SAC_SPG_DISABLED))
				{
					CUR_LIM_MODE[ax] = 1;

					//-------------------------
					// Limit controller output
					// ------------------------
					//StatusReadTwe[ax] = SqcReadData(&hndRWData[ax][Tweak_Compensation], &dTweakValue[ax]);

					if (ScanForceRate != 0)
					{
						StatusReadTwe[ax] = UdsxReadData(&hndRWData[ax][Tweak_Compensation], &dTweakValue[ax]);
					}
					else
					{
						dTweakValue[ax] = 1;
					}

					if (CUR_STAB_MODE[ax] != 255)
					{
						if ((AXIS_SPG_STATE[ax] == SAC_SPG_IDLE) && (pShmem_data->VC_POS[ax * 2] <= (pShmem_data->VC_POS[(ax * 2) + 1] - 20)))	//If Spg state is idle AND VC Pos is greater equal than setpoint  - 20
						{
							CUR_STAB_MODE[ax]++;

							if (CUR_STAB_MODE[ax] >= 3)
							{
								CUR_STAB_MODE[ax] = 255;
							}
						}
						else
						{
							CUR_STAB_MODE[ax] = 0;
						}
					}
					else
					{
						StabilizeCycle[ax]++;

						if (StabilizeCycle[ax] < StabilizeCnt + 1)
						{
							dCurStabilize[ax] = 0.2;
						}
						else
						{
							dCurStabilize[ax] = 0;
						}
					}

					if (dTweakValue[ax] + pShmem_data->FORCE_LIMIT[ax] + dCurStabilize[ax] <= dOrgSatLevel[ax])
					{
						SAT_LEVEL[ax] = dTweakValue[ax] + pShmem_data->FORCE_LIMIT[ax] + dCurStabilize[ax];
					}
					else
					{
						SAT_LEVEL[ax] = dOrgSatLevel[ax];
					}

					if (OLD_SAT_LEVEL[ax] != SAT_LEVEL[ax])
					{
						OLD_SAT_LEVEL[ax] = SAT_LEVEL[ax];
						StatusWriteForce[ax] = SacWriteParameter(sacAxis[ax], SAC_PAR_SAT_LEVEL, SAT_LEVEL[ax]);
					}

					//--------------------------------
					// Disable max dynamic pos error
					// -------------------------------
					//if (MAX_DYN_ERR[ax] != 0)
					//{
					//	MAX_DYN_ERR[ax] = 0;
					//	StatusWriteDyn[ax] = SacWriteParameter(sacAxis[ax], SAC_PAR_MAX_DYN_POS_ERROR,MAX_DYN_ERR[ax]);
					//}
				}
				else
				{
					CUR_LIM_MODE[ax] = 0;
					CUR_STAB_MODE[ax] = 0;
					StabilizeCycle[ax] = 0;
					dCurStabilize[ax] = 0;

					//-----------------------------------
					// Restore original controller output
					// ----------------------------------
					SAT_LEVEL[ax] = dOrgSatLevel[ax];

					if (OLD_SAT_LEVEL[ax] != SAT_LEVEL[ax])
					{
						OLD_SAT_LEVEL[ax] = SAT_LEVEL[ax];
						StatusWriteForceOri[ax] = SacWriteParameter(sacAxis[ax], SAC_PAR_SAT_LEVEL,SAT_LEVEL[ax]);
					}

					//--------------------------------
					// Restore max dynamic pos error
					// -------------------------------
					//if (MAX_DYN_ERR[ax] != dOrgDynError[ax])
					//{
					//	MAX_DYN_ERR[ax] = dOrgDynError[ax];
					//	StatusWriteDynOri[ax] = SacWriteParameter(sacAxis[ax], SAC_PAR_MAX_DYN_POS_ERROR,MAX_DYN_ERR[ax]);
					//}

				}
			}
			else
			{
				CUR_LIM_MODE[ax] = 2;
				CUR_STAB_MODE[ax] = 0;
				StabilizeCycle[ax] = 0;
				dCurStabilize[ax] = 0;

				SAT_LEVEL[ax] = -1 * pShmem_data->FORCE_LIMIT[ax];

				if (SAT_LEVEL[ax] > dOrgSatLevel[ax])
				{
					SAT_LEVEL[ax] = dOrgSatLevel[ax];
				}

				//-----------------------------------
				// Limit controller output
				// ----------------------------------
				if (OLD_SAT_LEVEL[ax] != SAT_LEVEL[ax])
				{
					OLD_SAT_LEVEL[ax] = SAT_LEVEL[ax];
					StatusWriteForce[ax] = SacWriteParameter(sacAxis[ax], SAC_PAR_SAT_LEVEL, SAT_LEVEL[ax]);
				}

				//--------------------------------
				// Restore max dynamic pos error
				// -------------------------------
				//if (MAX_DYN_ERR[ax] != dOrgDynError[ax])
				//{
				//	MAX_DYN_ERR[ax] = dOrgDynError[ax];
				//	StatusWriteDynOri[ax] = SacWriteParameter(sacAxis[ax], SAC_PAR_MAX_DYN_POS_ERROR,MAX_DYN_ERR[ax]);
				//}
			}

			//--------------------------------
			// Read drive current
			// -------------------------------
			//StatusReadCur[ax] = SqcReadData(&hndRWData[ax][Current_Drive], &dCurrentDrive);
			//DRIVE_CURRENT[ax] = dCurrentDrive;
			//NET_CURRENT[ax] = dCurrentDrive - dTweakValue[ax];

		}

		//ScanForceCtr[ax]++;
		//if (ScanForceCtr[ax] >= ScanForceRate)
		//{
		//	ScanForceCtr[ax] = 0;
		//}
	}
	pShmem_data->udsx_exit = 1;
	}
}

void UdsxTerminate(void)
{
    int ax;

    for ( ax = 0; ax < 10; ax++ )
    {
        if (SacConnected[ax] == 255)
        {
            StatusCloseDTwe[ax] = UdsxCloseData(&hndRWData[ax][Tweak_Compensation]);
            StatusCloseDCur[ax] = UdsxCloseData(&hndRWData[ax][Current_Drive]);
            StatusCloseDSet[ax] = UdsxCloseData(&hndRWData[ax][Setpoint_Pos]);
            StatusCloseDPos[ax] = UdsxCloseData(&hndRWData[ax][Axis_Pos]);
            StatusCloseDSta[ax] = UdsxCloseData(&hndRWData[ax][Axis_State]);
            StatusCloseDSpg[ax] = UdsxCloseData(&hndRWData[ax][Axis_SPG_State]);

            StatusSDisconnect[ax] = SacDisconnect(sacAxis[ax]);
        }
    }

}

/**
 *  @brief  On load function that is executed when the shared object is loaded
 *
 *  Because of the constructor attribute that is set for this function, this
 *  function is executed when the shared object is loaded, which is done within
 *  the linux domain.
 */
__attribute__((constructor)) void OnLoad(void)
{
    g_retVal = Initialize(TRUE);
}

/**
 *  @brief  On unload function that is executed when the shared object is unloaded
 *
 *  Because of the destructor attribute that is set for this function, this
 *  function is executed when the shared object is unloaded, which is done within
 *  the linux domain.
 */
__attribute__((destructor)) void OnUnload(void)
{
    Terminate();
}

#endif

static NYCE_STATUS Initialize(BOOL create)
{
    NYCE_STATUS retVal = NYCE_OK;

    /*
     * Create shared memory object.
     */
    g_sharedMemoryDescriptor = shm_open(SHM_NAME, O_RDWR | (create ? O_CREAT : 0), S_IRUSR | S_IWUSR);
    if (g_sharedMemoryDescriptor != -1)
    {
        g_created = create;

        /*
         *  Set its size to the size of the data.
         */
        if (!create ||
            (ftruncate(g_sharedMemoryDescriptor, sizeof(*pShmem_data)) == 0))
        {
            /*
             * Map shared memory object.
             */
        	pShmem_data = mmap(NULL, sizeof(*pShmem_data),
                              PROT_READ | PROT_WRITE, MAP_SHARED,
                              g_sharedMemoryDescriptor, 0);
            if (pShmem_data == MAP_FAILED)
            {
                retVal = USR_ERR_FAILED_TO_MAP_SHM;
                pShmem_data = NULL;
            }
            else
            {
                if (create)
                {
                    memset(pShmem_data, 0, sizeof(*pShmem_data));
                }
            }
        }
        else
        {
            retVal = USR_ERR_FAILED_TO_RESIZE_SHM;
        }

        if (NyceError(retVal))
        {
            /*
             * If an error occurred we must close the descriptor again.
             */
            Terminate();
        }
    }
    else
    {
        retVal = USR_ERR_FAILED_TO_CREATE_SHM;
    }

    return retVal;
}

static void Terminate(void)
{
    if (pShmem_data)
    {
        /*
         * If the counter is mapped we must unmap it.
         */
        (void)munmap(pShmem_data, sizeof(*pShmem_data));
        pShmem_data = NULL;
    }

    if (g_sharedMemoryDescriptor != -1)
    {
        /*
         * When the descriptor is still open we need to close it.
         */
        (void)close(g_sharedMemoryDescriptor);
        g_sharedMemoryDescriptor = -1;

        if (g_created)
        {
            /*
             * Finally, the file that was created must be removed.
             */
            (void)shm_unlink(SHM_NAME);
        }
    }
}

void ChkTurStat(int AxisID)
{

	switch(AXIS_STATE[AxisID])
	{
	case SAC_MOVING:
		StatusReadPos[AxisID] = UdsxReadData( &hndRWData[AxisID][Axis_Pos], &dPos);
		pShmem_data->VC_POS[AxisID * 2] = dPos;

		pShmem_data->Shared_StatFlag[AxisID] |= 0x01;

		if (dPosOffset != 0)
		{
			if (pShmem_data->VC_POS[AxisID * 2] + 18 >= 360)
			{
				pShmem_data->VC_POS[AxisID * 2] = pShmem_data->VC_POS[AxisID * 2] - 360;
			}
		}

		if (pShmem_data->VC_POS[AxisID * 2] > oldturretpos + 9.6)
		{
			pShmem_data->Shared_StatFlag[AxisID] |= 0x02;
		}

		if	(pShmem_data->Shared_CtrFlag[10] == 1.00)
		{
			//Flag270Deg = 16;
			Flag270Deg = 18 - (pShmem_data->Shared_CtrFlag[13]);
		}
		else
		{
			Flag270Deg = 16.5;
		}

		if (pShmem_data->VC_POS[AxisID * 2] > oldturretpos + Flag270Deg)
		{
			if ((pShmem_data->Shared_StatFlag[AxisID] & 0x04) == 0)
			{
				InterpolationPos[AxisID] = pShmem_data->VC_POS[AxisID * 2] - oldturretpos;
			}

			pShmem_data->Shared_StatFlag[AxisID] |= 0x04;
		}

		break;

	case SAC_READY:
		StatusReadPos[AxisID] = UdsxReadData( &hndRWData[AxisID][Axis_Pos], &dPos);
		StatusReadSet[AxisID] = UdsxReadData( &hndRWData[AxisID][Setpoint_Pos], &dSetpointPos);

		//--------------------------------
		// Read encoder and setpoint
		// -------------------------------
		pShmem_data->Shared_SetPointPos[AxisID] = dSetpointPos;		//Pass setpoint pos back to NodeSeq for motion profile calculation
		pShmem_data->VC_POS[(AxisID * 2) + 1] = dSetpointPos;
		pShmem_data->VC_POS[AxisID * 2] = dPos;

		if ((pShmem_data->Shared_StatFlag[AxisID] & 0x01) != 0)
		{
			pShmem_data->Shared_StatFlag[AxisID] |= 0x08;

			if (pShmem_data->VC_POS[(AxisID* 2) + 1] + 18 >= 360)
			{
				dPosOffset = 1;
				oldturretpos = pShmem_data->VC_POS[(AxisID* 2) + 1] - 360;
			}
			else
			{
				dPosOffset = 0;
				oldturretpos = pShmem_data->VC_POS[(AxisID* 2) + 1];
			}
		}

		break;

	case SAC_FREE:
		StatusReadPos[AxisID] = UdsxReadData( &hndRWData[AxisID][Axis_Pos], &dPos);
		StatusReadSet[AxisID] = UdsxReadData( &hndRWData[AxisID][Setpoint_Pos], &dSetpointPos);

		//--------------------------------
		// Read encoder and setpoint
		// -------------------------------
		pShmem_data->Shared_SetPointPos[AxisID] = dSetpointPos;		//Pass setpoint pos back to NodeSeq for motion profile calculation
		pShmem_data->VC_POS[(AxisID * 2) + 1] = dSetpointPos;
		pShmem_data->VC_POS[AxisID * 2] = dPos;

		//--------------------------------
		// Read drive current
		// -------------------------------
		DRIVE_CURRENT[AxisID] = dCurrentDrive;
		pShmem_data->NET_CURRENT[AxisID] = dCurrentDrive - dTweakValue[AxisID];

		pShmem_data->Shared_StatFlag[AxisID] &= 0x81;	//Reset stat

		if ((pShmem_data->Shared_StatFlag[AxisID] & 0x01) != 0)
		{
			pShmem_data->Shared_StatFlag[AxisID] |= 0x40;
		}
		break;
	}
}

void ChkVCStat(int AxisID)
{
	switch(AXIS_STATE[AxisID])
	{
	case SAC_MOVING:
		StatusReadPos[AxisID] = UdsxReadData( &hndRWData[AxisID][Axis_Pos], &dPos);
		StatusReadSet[AxisID] = UdsxReadData( &hndRWData[AxisID][Setpoint_Pos], &dSetpointPos);

		//--------------------------------
		// Read encoder and setpoint
		// -------------------------------
		pShmem_data->Shared_SetPointPos[AxisID] = dSetpointPos;		//Pass setpoint pos back to NodeSeq for motion profile calculation
		pShmem_data->VC_POS[(AxisID * 2) + 1] = dSetpointPos;
		pShmem_data->VC_POS[AxisID * 2] = dPos;

		//if (pShmem_data->VC_POS[AxisID * 2] <= pShmem_data->Shared_CtrFlag[AxisID+60] + LINEAR_THRESHOLD)
		if (pShmem_data->VC_POS[AxisID * 2] <= STANDBY_POS[AxisID] + LINEAR_THRESHOLD)
		{
			if ((pShmem_data->Shared_StatFlag[AxisID] & 0x02) == 0)
			{
				InterpolationPos[AxisID] = pShmem_data->VC_POS[AxisID * 2];
			}

			pShmem_data->Shared_StatFlag[AxisID] |= 0x02;
			//pShmem_data->Shared_StatFlag[AxisID] |= 0x10; // preemp vc ready
		}

		pShmem_data->Shared_StatFlag[AxisID] |= 0x01;

		break;

	case SAC_READY:
		StatusReadPos[AxisID] = UdsxReadData( &hndRWData[AxisID][Axis_Pos], &dPos);
		StatusReadSet[AxisID] = UdsxReadData( &hndRWData[AxisID][Setpoint_Pos], &dSetpointPos);
		StatusReadCur[AxisID] = UdsxReadData(&hndRWData[AxisID][Current_Drive], &dCurrentDrive);

		//--------------------------------
		// Read encoder and setpoint
		// -------------------------------
		pShmem_data->Shared_SetPointPos[AxisID] = dSetpointPos;		//Pass setpoint pos back to NodeSeq for motion profile calculation
		pShmem_data->VC_POS[(AxisID * 2) + 1] = dSetpointPos;
		pShmem_data->VC_POS[AxisID * 2] = dPos;

		//--------------------------------
		// Read drive current
		// -------------------------------
		DRIVE_CURRENT[AxisID] = dCurrentDrive;
		pShmem_data->NET_CURRENT[AxisID] = dCurrentDrive - dTweakValue[AxisID];

		pShmem_data->Shared_StatFlag[AxisID] &= 0x81;	//Reset stat

		//if (pShmem_data->VC_POS[AxisID * 2] <= pShmem_data->Shared_CtrFlag[AxisID+60] + LINEAR_THRESHOLD)
		if (pShmem_data->VC_POS[AxisID * 2] <= STANDBY_POS[AxisID] + LINEAR_THRESHOLD)
		{
			pShmem_data->Shared_StatFlag[AxisID] |= 0x02;
			//pShmem_data->Shared_StatFlag[AxisID] |= 0x10; // preemp vc ready
		}

		if ((pShmem_data->Shared_StatFlag[AxisID] & 0x01) != 0)
		{
			pShmem_data->Shared_StatFlag[AxisID] |= 0x04;

			if (pShmem_data->VC_POS[AxisID * 2] >= pShmem_data->Shared_CtrFlag[AxisID] - DP_THRESHOLD)
			{
				pShmem_data->Shared_StatFlag[AxisID] |= 0x10;
			}

			if ((DRIVE_CURRENT[AxisID] > SAT_LEVEL[AxisID] - 0.05) && (DRIVE_CURRENT[AxisID] < SAT_LEVEL[AxisID] + 0.05))
			{
				pShmem_data->Shared_StatFlag[AxisID] |= 0x20;
			}
		}

		break;

	case SAC_FREE:
		StatusReadPos[AxisID] = UdsxReadData( &hndRWData[AxisID][Axis_Pos], &dPos);
		StatusReadSet[AxisID] = UdsxReadData( &hndRWData[AxisID][Setpoint_Pos], &dSetpointPos);
		StatusReadCur[AxisID] = UdsxReadData(&hndRWData[AxisID][Current_Drive], &dCurrentDrive);

		//--------------------------------
		// Read encoder and setpoint
		// -------------------------------
		pShmem_data->Shared_SetPointPos[AxisID] = dSetpointPos;		//Pass setpoint pos back to NodeSeq for motion profile calculation
		pShmem_data->VC_POS[(AxisID * 2) + 1] = dSetpointPos;
		pShmem_data->VC_POS[AxisID * 2] = dPos;

		//--------------------------------
		// Read drive current
		// -------------------------------
		DRIVE_CURRENT[AxisID] = dCurrentDrive;
		pShmem_data->NET_CURRENT[AxisID] = dCurrentDrive - dTweakValue[AxisID];

		pShmem_data->Shared_StatFlag[AxisID] &= 0x81;	//Reset stat

		//if (pShmem_data->VC_POS[AxisID * 2] <= pShmem_data->Shared_CtrFlag[AxisID+60] + LINEAR_THRESHOLD)
		if (pShmem_data->VC_POS[AxisID * 2] <= STANDBY_POS[AxisID] + LINEAR_THRESHOLD)
		{
			pShmem_data->Shared_StatFlag[AxisID] |= 0x02;
			//pShmem_data->Shared_StatFlag[AxisID] |= 0x10; /// preemp vc ready
		}

		if ((pShmem_data->Shared_StatFlag[AxisID] & 0x01) != 0)
		{
			pShmem_data->Shared_StatFlag[AxisID] |= 0x40;

			//if (pShmem_data->VC_POS[AxisID * 2] <= pShmem_data->Shared_CtrFlag[AxisID+60] + 50)
			if (pShmem_data->VC_POS[AxisID * 2] <= STANDBY_POS[AxisID] + 50)
			{
				pShmem_data->Shared_StatFlag[AxisID] |= 0x08;
			}

			if (pShmem_data->VC_POS[AxisID * 2] >= pShmem_data->Shared_CtrFlag[AxisID] - DP_THRESHOLD)
			{
				pShmem_data->Shared_StatFlag[AxisID] |= 0x10;
			}

			if ((DRIVE_CURRENT[AxisID] > SAT_LEVEL[AxisID] - 0.05) && (DRIVE_CURRENT[AxisID] < SAT_LEVEL[AxisID] + 0.05))
			{
				OpenLoopStab[AxisID]++;
			}
			else
			{
				OpenLoopStab[AxisID] = 0;
			}

			if (OpenLoopStab[AxisID] >= 3)
			{
				pShmem_data->Shared_StatFlag[AxisID] |= 0x20;
			}
		}
		break;

	default:

		pShmem_data->Shared_StatFlag[AxisID] &= 0x81;	//Reset stat

		//if (pShmem_data->VC_POS[AxisID * 2] <= pShmem_data->Shared_CtrFlag[AxisID+60] + LINEAR_THRESHOLD)
		if (pShmem_data->VC_POS[AxisID * 2] <= STANDBY_POS[AxisID] + LINEAR_THRESHOLD)
		{
			pShmem_data->Shared_StatFlag[AxisID] |= 0x02;
			//pShmem_data->Shared_StatFlag[AxisID] |= 0x10; // preemp vc ready
		}

		break;
	}
}


void ChkStdStat(int AxisID)
{
	switch(AXIS_STATE[AxisID])
	{
	case SAC_MOVING:
		pShmem_data->Shared_StatFlag[AxisID] |= 0x01;

		break;

	case SAC_READY:
		StatusReadPos[AxisID] = UdsxReadData( &hndRWData[AxisID][Axis_Pos], &dPos);
		StatusReadSet[AxisID] = UdsxReadData( &hndRWData[AxisID][Setpoint_Pos], &dSetpointPos);
		StatusReadCur[AxisID] = UdsxReadData(&hndRWData[AxisID][Current_Drive], &dCurrentDrive);

		//--------------------------------
		// Read encoder and setpoint
		// -------------------------------
		pShmem_data->Shared_SetPointPos[AxisID] = dSetpointPos;		//Pass setpoint pos back to NodeSeq for motion profile calculation
		pShmem_data->VC_POS[(AxisID * 2) + 1] = dSetpointPos;
		pShmem_data->VC_POS[AxisID * 2] = dPos;

		//--------------------------------
		// Read drive current
		// -------------------------------
		DRIVE_CURRENT[AxisID] = dCurrentDrive;
		pShmem_data->NET_CURRENT[AxisID] = dCurrentDrive - dTweakValue[AxisID];

		pShmem_data->Shared_StatFlag[AxisID] &= 0x83;	//Reset stat

		if ((pShmem_data->Shared_StatFlag[AxisID] & 0x01) != 0)
		{
			pShmem_data->Shared_StatFlag[AxisID] |= 0x04;
		}

		break;
	}
}
