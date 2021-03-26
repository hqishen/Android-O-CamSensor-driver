
/******************************************************************************
 *
 * Copyright 2010, Dream Chip Technologies GmbH. All rights reserved.
 * No part of this work may be reproduced, modified, distributed, transmitted,
 * transcribed, or translated into any language or computer format, in any form
 * or by any means without written permission of:
 * Dream Chip Technologies GmbH, Steinriede 10, 30827 Garbsen / Berenbostel,
 * Germany
 *
 *****************************************************************************/
/**
 * @file IMX307.c
 *
 * @brief
 *   ADD_DESCRIPTION_HERE
 *
 *****************************************************************************/
#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>
#include <common/misc.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"

#include "IMX307_MIPI_priv.h"

#define  IMX307_NEWEST_TUNING_XML "29-07-2017_IMX307_CMK-FLML3-X2A10"

#define CC_OFFSET_SCALING  2.0f
#define I2C_COMPLIANT_STARTBIT 1U

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( IMX307_INFO , "IMX307: ", INFO,    0U );
CREATE_TRACER( IMX307_WARN , "IMX307: ", WARNING, 1U );
CREATE_TRACER( IMX307_ERROR, "IMX307: ", ERROR,   1U );

CREATE_TRACER( IMX307_DEBUG, "IMX307: ", DEBUG,   0U );

CREATE_TRACER( IMX307_REG_INFO , "IMX307: ", INFO, 0);
CREATE_TRACER( IMX307_REG_DEBUG, "IMX307: ", INFO, 0U );

#define IMX307_SLAVE_ADDR       0x6CU                           /**< i2c slave address of the IMX307 camera sensor */
#define IMX307_SLAVE_ADDR2      0x34U
#define IMX307_SLAVE_AF_ADDR    0x18U                           /**< i2c slave address of the IMX307 integrated AD5820 */
#define IMX307_OTP_SLAVE_ADDR   0xA0U

#define IMX307_MIN_GAIN_STEP   ( 1.0f / 16.0f); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define IMX307_MAX_GAIN_AEC    ( 8.0f )            /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */


/*!<
 * Focus position values:
 * 65 logical positions ( 0 - 64 )
 * where 64 is the setting for infinity and 0 for macro
 * corresponding to
 * 1024 register settings (0 - 1023)
 * where 0 is the setting for infinity and 1023 for macro
 */
#define MAX_LOG   64U
#define MAX_REG 1023U

#define MAX_VCMDRV_CURRENT      100U
#define MAX_VCMDRV_REG          1023U


/*!<
 * Lens movement is triggered every 133ms (VGA, 7.5fps processed frames
 * worst case assumed, usually even much slower, see OV5630 driver for
 * details). Thus the lens has to reach the requested position after
 * max. 133ms. Minimum mechanical ringing is expected with mode 1 ,
 * 100us per step. A movement over the full range needs max. 102.3ms
 * (see table 9 AD5820 datasheet).
 */
#define MDI_SLEW_RATE_CTRL 6U /* S3..0 */

#define COLOUR 			0
#define WHITE_BLACK 	1


/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char IMX307_g_acName[] = "IMX307_MIPI";

extern const IsiRegDescription_t IMX307_g_aRegDescription_twolane[];
extern const IsiRegDescription_t IMX307_g_1920x1080_twolane[];
extern const IsiRegDescription_t IMX307_g_1920x1080P30_twolane_fpschg[];
extern const IsiRegDescription_t IMX307_g_1920x1080P25_twolane_fpschg[];
extern const IsiRegDescription_t IMX307_g_1920x1080P15_twolane_fpschg[];
extern const IsiRegDescription_t IMX307_g_1920x1080P10_twolane_fpschg[];
extern const IsiRegDescription_t IMX307_g_1920x1080P5_twolane_fpschg[];

extern const IsiRegDescription_t IMX307_g_aRegDescription_fourlane[];
extern const IsiRegDescription_t IMX307_g_1920x1080_fourlane[];
extern const IsiRegDescription_t IMX307_g_1920x1080P60_fourlane_fpschg[];
extern const IsiRegDescription_t IMX307_g_1920x1080P50_fourlane_fpschg[];
extern const IsiRegDescription_t IMX307_g_1920x1080P30_fourlane_fpschg[];
extern const IsiRegDescription_t IMX307_g_1920x1080P25_fourlane_fpschg[];
extern const IsiRegDescription_t IMX307_g_1920x1080P20_fourlane_fpschg[];
extern const IsiRegDescription_t IMX307_g_1920x1080P15_fourlane_fpschg[];
extern const IsiRegDescription_t IMX307_g_1920x1080P10_fourlane_fpschg[];



const IsiSensorCaps_t IMX307_g_IsiSensorDefaultConfig;


#define IMX307_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define IMX307_I2C_NR_ADR_BYTES     (2U)                        // 1 byte base address and 2 bytes sub address
#define IMX307_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers


static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_TWO_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_TWO_LANE



/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT IMX307_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT IMX307_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT IMX307_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT IMX307_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT IMX307_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT IMX307_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT IMX307_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT IMX307_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);
static RESULT IMX307_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT IMX307_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT IMX307_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT IMX307_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT IMX307_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT IMX307_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT IMX307_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT IMX307_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT IMX307_IsiGetColorIss( IsiSensorHandle_t handle, char *pGetColor);
static RESULT IMX307_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT IMX307_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT IMX307_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT IMX307_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );
static RESULT IMX307_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT IMX307_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );
static RESULT IMX307_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT IMX307_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT IMX307_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT IMX307_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT IMX307_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT IMX307_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT IMX307_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT IMX307_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );
#ifdef IMX307_AF_ENABLE
static RESULT IMX307_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT IMX307_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT IMX307_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT IMX307_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT IMX307_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );
#endif
static RESULT IMX307_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT IMX307_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT IMX307_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);


static float dctfloor( const float f )
{
    if ( f < 0 )
    {
        return ( (float)((int32_t)f - 1L) );
    }
    else
    {
        return ( (float)((uint32_t)f) );
    }
}



/*****************************************************************************/
/**
 *          IMX307_IsiCreateSensorIss
 *
 * @brief   This function creates a new OV13850 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT IMX307_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    IMX307_Context_t *pSensorCtx;

    //TRACE( IMX307_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pSensorCtx = ( IMX307_Context_t * )malloc ( sizeof (IMX307_Context_t) );
    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR,  "%s: Can't allocate ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorCtx, 0, sizeof( IMX307_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pSensorCtx );
        return ( result );
    }

    pSensorCtx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pSensorCtx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pSensorCtx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pSensorCtx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? IMX307_SLAVE_ADDR : pConfig->SlaveAddr;
    pSensorCtx->IsiCtx.NrOfAddressBytes       = 2U;

    pSensorCtx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
#ifdef IMX307_AF_ENABLE
    pSensorCtx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? IMX307_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
#else
	pSensorCtx->IsiCtx.SlaveAfAddress  = pConfig->SlaveAfAddr;
#endif

    pSensorCtx->IsiCtx.NrOfAfAddressBytes     = 1U;

    pSensorCtx->IsiCtx.pSensor                = pConfig->pSensor;

    pSensorCtx->Configured             = BOOL_FALSE;
    pSensorCtx->Streaming              = BOOL_FALSE;
    pSensorCtx->TestPattern            = BOOL_FALSE;
    pSensorCtx->isAfpsRun              = BOOL_FALSE;
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pSensorCtx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pSensorCtx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pSensorCtx->VcmInfo.RatedCurrent   = pSensorCtx->VcmInfo.StartCurrent + MAX_LOG*pSensorCtx->VcmInfo.Step;
    pSensorCtx->VcmInfo.StepMode       = pConfig->VcmStepMode;  

    pSensorCtx->IsiSensorMipiInfo.sensorHalDevID = pSensorCtx->IsiCtx.HalDevID;
    if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }

    pConfig->hSensor = ( IsiSensorHandle_t )pSensorCtx;

    result = HalSetCamConfig( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false, true, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, 37125000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( IMX307_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an Sensor instance.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT IMX307_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    //TRACE( IMX307_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)IMX307_IsiSensorSetStreamingIss( pSensorCtx, BOOL_FALSE );
    (void)IMX307_IsiSensorSetPowerIss( pSensorCtx, BOOL_FALSE );

    (void)HalDelRef( pSensorCtx->IsiCtx.HalHandle );

    MEMSET( pSensorCtx, 0, sizeof( IMX307_Context_t ) );
    free ( pSensorCtx );

    //TRACE( IMX307_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetCapsIss
 *
 * @brief   fills in the correct pointers for the sensor description struct
 *
 * @param   param1      pointer to sensor capabilities structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetCapsIssInternal
(
    IsiSensorCaps_t   *pIsiSensorCaps,
    uint32_t mipi_lanes
)
{

    RESULT result = RET_SUCCESS;

    if ( pIsiSensorCaps == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {

	
	if(mipi_lanes == SUPPORT_MIPI_TWO_LANE) {
		switch (pIsiSensorCaps->Index) 
		{
			case 0:
			{
				pIsiSensorCaps->Resolution = ISI_RES_TV1080P30;
				break;
			}
			case 1:
			{
				pIsiSensorCaps->Resolution = ISI_RES_TV1080P25;
				break;
			}
			case 2:
			{
				pIsiSensorCaps->Resolution = ISI_RES_TV1080P15;
				break;
			}
			case 3:
			{
				pIsiSensorCaps->Resolution = ISI_RES_TV1080P10;
				break;
			}
			default:
			{
				result = RET_OUTOFRANGE;
				goto end;
			}

		}
		}
    else if(mipi_lanes == SUPPORT_MIPI_FOUR_LANE){
        switch (pIsiSensorCaps->Index) 
        {
        	#if 0
            case 0:
            {
                pIsiSensorCaps->Resolution = ISI_RES_TV1080P60;
                break;
            }
			
            case 1:
            {
                pIsiSensorCaps->Resolution = ISI_RES_TV1080P50;
                break;
            }

			case 2:
            {
                pIsiSensorCaps->Resolution = ISI_RES_TV1080P30;
                break;
            }
			#else
			case 0:
            {
                pIsiSensorCaps->Resolution = ISI_RES_TV1080P30;
                break;
            }
			case 1:
            {
                pIsiSensorCaps->Resolution = ISI_RES_TV1080P25;
                break;
            }
			case 2:
            {
                pIsiSensorCaps->Resolution = ISI_RES_TV1080P20;
                break;
            }
			case 3:
            {
                pIsiSensorCaps->Resolution = ISI_RES_TV1080P15;
                break;
            }
			case 4:
            {
                pIsiSensorCaps->Resolution = ISI_RES_TV1080P10;
                break;
            }
			#endif

            default:
            {
                result = RET_OUTOFRANGE;
                goto end;
            }

        }            
        }  
        //TRACE( IMX307_INFO, "res:%d    %s (exit)\n", pIsiSensorCaps->Index, __FUNCTION__);
    
        pIsiSensorCaps->BusWidth        = ISI_BUSWIDTH_10BIT;
        pIsiSensorCaps->Mode            = ISI_MODE_MIPI;
        pIsiSensorCaps->FieldSelection  = ISI_FIELDSEL_BOTH;
        pIsiSensorCaps->YCSequence      = ISI_YCSEQ_YCBYCR;           /**< only Bayer supported, will not be evaluated */
        pIsiSensorCaps->Conv422         = ISI_CONV422_NOCOSITED;
        pIsiSensorCaps->BPat            = ISI_BPAT_RGRGGBGB; 
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS;
        pIsiSensorCaps->VPol            = ISI_VPOL_POS;
        pIsiSensorCaps->Edge            = ISI_EDGE_RISING;
        pIsiSensorCaps->Bls             = ISI_BLS_OFF;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_OFF;
        pIsiSensorCaps->CConv           = ISI_CCONV_OFF;
        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO);
        pIsiSensorCaps->AGC             = ( ISI_AGC_OFF );
        pIsiSensorCaps->AWB             = ( ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_OFF );

        pIsiSensorCaps->DwnSz           = ISI_DWNSZ_SUBSMPL;
        pIsiSensorCaps->CieProfile      = ( ISI_CIEPROF_A
                                          | ISI_CIEPROF_D50
                                          | ISI_CIEPROF_D65
                                          | ISI_CIEPROF_D75
                                          | ISI_CIEPROF_F2
                                          | ISI_CIEPROF_F11 );
        pIsiSensorCaps->SmiaMode        = ISI_SMIA_OFF;
        pIsiSensorCaps->MipiMode        = ISI_MIPI_MODE_RAW_10;
        pIsiSensorCaps->AfpsResolutions = ( ISI_AFPS_NOTSUPP );
		pIsiSensorCaps->SensorOutputMode = ISI_SENSOR_OUTPUT_MODE_RAW;
    }
end:

    return ( result );
}

static RESULT IMX307_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    //TRACE( IMX307_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    result = IMX307_IsiGetCapsIssInternal(pIsiSensorCaps, pSensorCtx->IsiSensorMipiInfo.ucMipiLanes);
   // TRACE( IMX307_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t IMX307_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_10BIT,         // BusWidth
    ISI_MODE_MIPI,              // MIPI
    ISI_FIELDSEL_BOTH,          // FieldSel
    ISI_YCSEQ_YCBYCR,           // YCSeq
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_RGRGGBGB,           // BPat
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_POS,               // VPol
    ISI_EDGE_RISING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_OFF,              // Gamma
    ISI_CCONV_OFF,              // CConv
    ISI_RES_TV1080P30,            // Res
    ISI_DWNSZ_SUBSMPL,          // DwnSz
    ISI_BLC_AUTO,               // BLC
    ISI_AGC_OFF,                // AGC
    ISI_AWB_OFF,                // AWB
    ISI_AEC_OFF,                // AEC
    ISI_DPCC_OFF,               // DPCC
    ISI_CIEPROF_D65,            // CieProfile, this is also used as start profile for AWB (if not altered by menu settings)
    ISI_SMIA_OFF,               // SmiaMode
    ISI_MIPI_MODE_RAW_10,       // MipiMode
    ISI_AFPS_NOTSUPP,           // AfpsResolutions
    ISI_SENSOR_OUTPUT_MODE_RAW,
    0,
};



/*****************************************************************************/
/**
 *          IMX307_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX307_SetupOutputFormat
(
    IMX307_Context_t       *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    //TRACE( IMX307_INFO, "%s%s (enter)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* mode */
    switch ( pConfig->Mode )            /* only ISI_MODE_BAYER supported, no configuration needed here */
    {
        case( ISI_MODE_MIPI ):
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* field-selection */
    switch ( pConfig->FieldSelection )  /* only ISI_FIELDSEL_BOTH supported, no configuration needed */
    {
        case ISI_FIELDSEL_BOTH:
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by Sensor, so the YCSequence parameter is not checked */
    switch ( pConfig->YCSequence )
    {
        default:
        {
            break;
        }
    }

    /* 422 conversion */
    switch ( pConfig->Conv422 )         /* only ISI_CONV422_NOCOSITED supported, no configuration needed */
    {
        case ISI_CONV422_NOCOSITED:
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* bayer-pattern */
    switch ( pConfig->BPat )            /* only ISI_BPAT_BGBGGRGR supported, no configuration needed */
    {
        case ISI_BPAT_BGBGGRGR:
        {
            break;
        }
		case ISI_BPAT_RGRGGBGB:
		{
			break;

		}

        default:
        {
            TRACE( IMX307_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* horizontal polarity */
    switch ( pConfig->HPol )            /* only ISI_HPOL_REFPOS supported, no configuration needed */
    {
        case ISI_HPOL_REFPOS:
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* vertical polarity */
    switch ( pConfig->VPol )            /*no configuration needed */
    {
        case ISI_VPOL_NEG:
        {
            break;
        }
        case ISI_VPOL_POS:
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }


    /* edge */
    switch ( pConfig->Edge )            /* only ISI_EDGE_RISING supported, no configuration needed */
    {
        case ISI_EDGE_RISING:
        {
            break;
        }

        case ISI_EDGE_FALLING:          /*TODO for MIPI debug*/
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* gamma */
    switch ( pConfig->Gamma )           /* only ISI_GAMMA_OFF supported, no configuration needed */
    {
        case ISI_GAMMA_OFF:
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* color conversion */
    switch ( pConfig->CConv )           /* only ISI_CCONV_OFF supported, no configuration needed */
    {
        case ISI_CCONV_OFF:
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->SmiaMode )        /* only ISI_SMIA_OFF supported, no configuration needed */
    {
        case ISI_SMIA_OFF:
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->MipiMode )        /* only ISI_MIPI_MODE_RAW_12 supported, no configuration needed */
    {
        case ISI_MIPI_MODE_RAW_10:
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->AfpsResolutions ) /* no configuration needed */
    {
        case ISI_AFPS_NOTSUPP:
        {
            break;
        }
        default:
        {
            // don't care about what comes in here
            //TRACE( IMX307_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    //TRACE( IMX307_INFO, "%s%s (exit)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

/*****************************************************************************/
/**
 *          IMX307_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_SetupOutputWindowInternal
(
    IMX307_Context_t        *pSensorCtx,
    const IsiSensorConfig_t *pConfig,
    bool_t set2Sensor,
    bool_t res_no_chg
)
{
    RESULT result     = RET_SUCCESS;
    uint16_t usFrameLengthLines = 0;
    uint16_t usLineLengthPck    = 0;
    float    rVtPixClkFreq      = 0.0f;
    int xclk = 37125000U;
	usLineLengthPck = 0x1130;
	usFrameLengthLines = 0x0465; 

    //TRACE( IMX307_ERROR, "%s (enter)-- sun -pConfig->Resolution:%x\n", __FUNCTION__,pConfig->Resolution);
	if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){

        pSensorCtx->IsiSensorMipiInfo.ulMipiFreq = 445;
    	switch ( pConfig->Resolution )
        {
            case ISI_RES_TV1080P30:
			case ISI_RES_TV1080P25:
			case ISI_RES_TV1080P15:
			case ISI_RES_TV1080P10:
	        {
	            if (set2Sensor == BOOL_TRUE) {                    
                    if (res_no_chg == BOOL_FALSE) {
						if((result = IsiRegDefaultsApply((IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080_twolane)) != RET_SUCCESS){
							result = RET_FAILURE;
							TRACE( IMX307_ERROR, "%s: failed to set two lane IMX307_g_1920x1080 \n", __FUNCTION__ );
			            }
					}
                    

                    if (pConfig->Resolution == ISI_RES_TV1080P30) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P30_twolane_fpschg);
					}else if (pConfig->Resolution == ISI_RES_TV1080P25) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P25_twolane_fpschg);
					}else if (pConfig->Resolution == ISI_RES_TV1080P15) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P15_twolane_fpschg);
					}else if (pConfig->Resolution == ISI_RES_TV1080P10) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P10_twolane_fpschg);
					}
                }
	         					
			    if(pConfig->Resolution == ISI_RES_TV1080P30) {				
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P30 \n", __FUNCTION__ );
					usLineLengthPck = 0x1130;
	            	usFrameLengthLines = 0x0465; 
				}else if(pConfig->Resolution == ISI_RES_TV1080P25) {
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P25 \n", __FUNCTION__ );
					usLineLengthPck = 0x1130;
	            	usFrameLengthLines = 0x0546;
				}else if(pConfig->Resolution == ISI_RES_TV1080P15) {
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P15 \n", __FUNCTION__ );
					usLineLengthPck = 0x1130;
	            	usFrameLengthLines = 0x8ca;
				}else if(pConfig->Resolution == ISI_RES_TV1080P10) {
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P10 \n", __FUNCTION__ );
					usLineLengthPck = 0x1130;
	            	usFrameLengthLines = 0xd2f;
				}
				
	            break;
	            
	        }


            default:
            {
                TRACE( IMX307_ERROR, "%s: two lane Resolution(0x%x) not supported\n", __FUNCTION__, pConfig->Resolution);
                return ( RET_NOTSUPP );
            }
    	}
		rVtPixClkFreq = 148500000;
		}

	else if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
	    pSensorCtx->IsiSensorMipiInfo.ulMipiFreq = 445;
	    switch ( pConfig->Resolution )
	    {
			case ISI_RES_TV1080P60:
			case ISI_RES_TV1080P50:
			case ISI_RES_TV1080P30:
			case ISI_RES_TV1080P25:
			case ISI_RES_TV1080P20:
			case ISI_RES_TV1080P15:
			case ISI_RES_TV1080P10:
	        {
	            if (set2Sensor == BOOL_TRUE) {                    
                    if (res_no_chg == BOOL_FALSE) {
						if((result = IsiRegDefaultsApply((IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080_fourlane)) != RET_SUCCESS){
							result = RET_FAILURE;
							TRACE( IMX307_ERROR, "%s: failed to set four lane IMX307_g_1920x1080 \n", __FUNCTION__ );
			            }
					}
                    

                    if (pConfig->Resolution == ISI_RES_TV1080P60) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P60_fourlane_fpschg);
					}else if (pConfig->Resolution == ISI_RES_TV1080P50) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P50_fourlane_fpschg);
					}else if (pConfig->Resolution == ISI_RES_TV1080P30) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P30_fourlane_fpschg);
					}else if (pConfig->Resolution == ISI_RES_TV1080P25) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P25_fourlane_fpschg);
					}else if (pConfig->Resolution == ISI_RES_TV1080P20) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P20_fourlane_fpschg);
					}else if (pConfig->Resolution == ISI_RES_TV1080P15) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P15_fourlane_fpschg);
					}else if (pConfig->Resolution == ISI_RES_TV1080P10) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, IMX307_g_1920x1080P10_fourlane_fpschg);
					}
                }
	         					
			    if(pConfig->Resolution == ISI_RES_TV1080P60) {				
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P60 \n", __FUNCTION__ );
					usLineLengthPck = 0x0898;
	            	usFrameLengthLines = 0x0456;	
				}else if(pConfig->Resolution == ISI_RES_TV1080P50) {
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P50 \n", __FUNCTION__ );
					usLineLengthPck = 0x0898;
	            	usFrameLengthLines = 0x0546; 
				}else if(pConfig->Resolution == ISI_RES_TV1080P30) {
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P30 \n", __FUNCTION__ );
					usLineLengthPck = 0x0898;
	            	usFrameLengthLines = 0x08ca; 
				}else if(pConfig->Resolution == ISI_RES_TV1080P25) {
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P50 \n", __FUNCTION__ );
					usLineLengthPck = 0x0898;
	            	usFrameLengthLines = 0x0a8c; 
				}else if(pConfig->Resolution == ISI_RES_TV1080P20) {
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P30 \n", __FUNCTION__ );
					usLineLengthPck = 0x0898;
	            	usFrameLengthLines = 0x0d2f; 
				}else if(pConfig->Resolution == ISI_RES_TV1080P15) {
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P50 \n", __FUNCTION__ );
					usLineLengthPck = 0x0898;
	            	usFrameLengthLines = 0x1194; 
				}else if(pConfig->Resolution == ISI_RES_TV1080P10) {
					TRACE( IMX307_INFO, "%s: ISI_RES_TV1080P30 \n", __FUNCTION__ );
					usLineLengthPck = 0x0898;
	            	usFrameLengthLines = 0x1a5e; 
				}
	            break;
	            
	        }

	        default:
	        {
	            TRACE( IMX307_ERROR, "%s: four lane Resolution not supported\n", __FUNCTION__ );
	            return ( RET_NOTSUPP );
	        }
	    }
		rVtPixClkFreq = 148500000;
		
	}


	
	// store frame timing for later use in AEC module
	rVtPixClkFreq = xclk;
    pSensorCtx->VtPixClkFreq     = rVtPixClkFreq;
    pSensorCtx->LineLengthPck    = usLineLengthPck;
    pSensorCtx->FrameLengthLines = usFrameLengthLines;	
    
    TRACE( IMX307_INFO, "%s AecMaxIntegrationTime:%f(****************exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   rVtPixClkFreq: %f\n", __FUNCTION__,
    					pSensorCtx->AecMaxIntegrationTime,
                        ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,rVtPixClkFreq);


    return ( result );
}




/*****************************************************************************/
/**
 *          IMX307_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      Sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX307_SetupImageControl
(
    IMX307_Context_t        *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;


    //TRACE( IMX307_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = IMX307_IsiRegReadIss(  pSensorCtx, IMX307_BLC_CTRL00, &RegValue );
            //result = IMX307_IsiRegWriteIss( pSensorCtx, IMX307_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = IMX307_IsiRegReadIss(  pSensorCtx, IMX307_BLC_CTRL00, &RegValue );
            //result = IMX307_IsiRegWriteIss( pSensorCtx, IMX307_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = IMX307_IsiRegReadIss(  pSensorCtx, IMX307_AEC_MANUAL, &RegValue );
            //result = IMX307_IsiRegWriteIss( pSensorCtx, IMX307_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = IMX307_IsiRegReadIss(  pSensorCtx, IMX307_ISP_CTRL01, &RegValue );
            //result = IMX307_IsiRegWriteIss( pSensorCtx, IMX307_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = IMX307_IsiRegReadIss(  pSensorCtx, IMX307_AEC_MANUAL, &RegValue );
            //result = IMX307_IsiRegWriteIss( pSensorCtx, IMX307_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = IMX307_IsiRegReadIss( pSensorCtx, IMX307_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = IMX307_IsiRegWriteIss( pSensorCtx, IMX307_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( IMX307_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT IMX307_SetupOutputWindow
(
    IMX307_Context_t        *pSensorCtx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return IMX307_SetupOutputWindowInternal(pSensorCtx,pConfig,BOOL_TRUE, BOOL_FALSE);
}


/*****************************************************************************/
/**
 *          IMX307_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in Sensor-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      Sensor context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_AecSetModeParameters
(
    IMX307_Context_t       *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    //TRACE( IMX307_INFO, "%s%s (enter)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"");
    TRACE( IMX307_INFO, "%s%s (enter)  Res: 0x%x  0x%x\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"",
        pSensorCtx->Config.Resolution, pConfig->Resolution);

    if ( (pSensorCtx->VtPixClkFreq == 0.0f) )
    {
        TRACE( IMX307_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    pSensorCtx->AecMaxIntegrationTime = ( ((float)(pSensorCtx->FrameLengthLines)) * ((float)pSensorCtx->LineLengthPck) ) / pSensorCtx->VtPixClkFreq;
    pSensorCtx->AecMinIntegrationTime = 0.00005f;    

    pSensorCtx->AecMaxGain = IMX307_MAX_GAIN_AEC;
    pSensorCtx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pSensorCtx->AecIntegrationTimeIncrement = ((float)pSensorCtx->LineLengthPck) / pSensorCtx->VtPixClkFreq;
    pSensorCtx->AecGainIncrement = IMX307_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pSensorCtx->AecCurGain               = pSensorCtx->AecMinGain;
    pSensorCtx->AecCurIntegrationTime    = 0.0f;
    pSensorCtx->OldCoarseIntegrationTime = 0;
    pSensorCtx->OldFineIntegrationTime   = 0;
    //pSensorCtx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( IMX307_INFO, "%s%s (exit)\n pSensorCtx->AecMaxIntegrationTime:%f\n pSensorCtx->FrameLengthLines:%d\n pSensorCtx->LineLengthPck:%d\n pSensorCtx->VtPixClkFreq:%f\n",
    __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"",
    pSensorCtx->AecMaxIntegrationTime,
    pSensorCtx->FrameLengthLines,
    pSensorCtx->LineLengthPck,
    pSensorCtx->VtPixClkFreq
    );

    return ( result );
}

/**
 *          IMX307_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    //TRACE( IMX307_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pSensorCtx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = IMX307_IsiRegWriteIss ( pSensorCtx, IMX307_SOFTWARE_RST, 0x01U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );


    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    result = IMX307_IsiRegWriteIss( pSensorCtx, IMX307_MODE_SELECT, 0x00 );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX307_ERROR, "%s: Can't write Sensor Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){
		result = IsiRegDefaultsApply( pSensorCtx, IMX307_g_aRegDescription_twolane);
	}
	else if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
		result = IsiRegDefaultsApply( pSensorCtx, IMX307_g_aRegDescription_fourlane);
	}
    
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 10 );

    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( pSensorCtx, IMX307_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = IMX307_SetupOutputFormat( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX307_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = IMX307_SetupOutputWindow( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX307_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = IMX307_SetupImageControl( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX307_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = IMX307_AecSetModeParameters( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX307_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pSensorCtx->Configured = BOOL_TRUE;
    }


    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiChangeSensorResolutionIss
 *
 * @brief   Change image sensor resolution while keeping all other static settings.
 *          Dynamic settings like current gain & integration time are kept as
 *          close as possible. Sensor needs 2 frames to engage (first 2 frames
 *          are not correctly exposed!).
 *
 * @note    Re-read current & min/max values as they will probably have changed!
 *
 * @param   handle                  Sensor instance handle
 * @param   Resolution              new resolution ID
 * @param   pNumberOfFramesToSkip   reference to storage for number of frames to skip
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_WRONG_STATE
 * @retval  RET_OUTOFRANGE
 *
 *****************************************************************************/
static RESULT IMX307_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    //TRACE( IMX307_INFO, "%s (enter)\n", __FUNCTION__);
    //TRACE( IMX307_INFO, "%s (enter)  Resolution: %dx%d@%dfps\n", __FUNCTION__,
        //ISI_RES_W_GET(Resolution),ISI_RES_H_GET(Resolution), ISI_FPS_GET(Resolution));

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pSensorCtx->Configured != BOOL_TRUE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (IMX307_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

     if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }
	//TRACE( IMX307_ERROR, "%s (11111111enter)  \n", __FUNCTION__);
    if ( Resolution == pSensorCtx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;

		bool_t res_no_chg;
		//TRACE( IMX307_ERROR, "%s (2222222222enter)  \n", __FUNCTION__);
        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution)) && 
            (ISI_RES_H_GET(Resolution)==ISI_RES_H_GET(pSensorCtx->Config.Resolution))) ) {

            if (pSensorCtx->Streaming != BOOL_FALSE) {
                TRACE( IMX307_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
		//TRACE( IMX307_ERROR, "%s (333333333enter)  \n", __FUNCTION__);
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( IMX307_INFO, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context
        pSensorCtx->Config.Resolution = Resolution;

        // tell sensor about that
        result = IMX307_SetupOutputWindowInternal( pSensorCtx, &pSensorCtx->Config, BOOL_TRUE, res_no_chg );
        if ( result != RET_SUCCESS )
        {
            TRACE( IMX307_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pSensorCtx->AecCurGain;
        float OldIntegrationTime = pSensorCtx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = IMX307_AecSetModeParameters( pSensorCtx, &pSensorCtx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( IMX307_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = IMX307_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( IMX307_ERROR, "%s: IMX307_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
        //	*pNumberOfFramesToSkip = 0;
    }

    TRACE( IMX307_INFO, "%s (exit)  result: 0x%x   pNumberOfFramesToSkip: %d \n", __FUNCTION__, result,
        *pNumberOfFramesToSkip);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiSensorSetStreamingIss
 *
 * @brief   Enables/disables streaming of sensor data, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   on          new streaming state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
static RESULT IMX307_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;


    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    //TRACE( IMX307_INFO, "%s (enter)  on = %d\n", __FUNCTION__,on);
    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSensorCtx->Configured != BOOL_TRUE) || (pSensorCtx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = IMX307_IsiRegReadIss ( pSensorCtx, IMX307_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		 result = IMX307_IsiRegWriteIss ( pSensorCtx, IMX307_MODE_SELECT, (RegValue & ~0x01U) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		
    }
    else
    {   
        /* disable streaming */
        result = IMX307_IsiRegReadIss ( pSensorCtx, IMX307_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       result = IMX307_IsiRegWriteIss ( pSensorCtx, IMX307_MODE_SELECT, (RegValue | 0x01U) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        //TRACE(IMX307_ERROR," STREAM OFF ++++++++++++++");
    }

    if (result == RET_SUCCESS)
    {
        pSensorCtx->Streaming = on;
    }

    //TRACE( IMX307_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pSensorCtx->Configured = BOOL_FALSE;
    pSensorCtx->Streaming  = BOOL_FALSE;

    result = HalSetPower( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    {
        result = HalSetPower( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( IMX307_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = IMX307_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 8U) | (IMX307_CHIP_ID_LOW_BYTE_DEFAULT);

    result = IMX307_IsiGetSensorRevisionIss( handle, &value );

    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( IMX307_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( IMX307_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          IMX307_IsiGetSensorRevisionIss
 *
 * @brief   reads the sensor revision register and returns this value
 *
 * @param   handle      pointer to sensor description struct
 * @param   p_value     pointer to storage value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;
	uint32_t vcm_pos = MAX_LOG;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = IMX307_IsiRegReadIss ( handle, IMX307_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 8U );
    result = IMX307_IsiRegReadIss ( handle, IMX307_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF) );

    TRACE( IMX307_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiRegReadIss
 *
 * @brief   grants user read access to the camera register
 *
 * @param   handle      pointer to sensor description struct
 * @param   address     sensor register to write
 * @param   p_value     pointer to value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    //TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, IMX307_g_aRegDescription_fourlane);
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }
        //TRACE( IMX307_REG_DEBUG, "%s (IsiGetNrDatBytesIss %d 0x%08x)\n", __FUNCTION__, NrOfBytes, address);

        *p_value = 0;
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

    //TRACE( IMX307_ERROR, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiRegWriteIss
 *
 * @brief   grants user write access to the camera register
 *
 * @param   handle      pointer to sensor description struct
 * @param   address     sensor register to write
 * @param   value       value to write
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT IMX307_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

    //TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, IMX307_g_aRegDescription_fourlane);
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }
    //TRACE( IMX307_REG_DEBUG, "%s (IsiGetNrDatBytesIss %d 0x%08x 0x%08x)\n", __FUNCTION__, NrOfBytes, address, value);

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

    //TRACE( IMX307_ERROR, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          Sensor instance
 *
 * @param   handle       Sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( IMX307_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pSensorCtx->AecMinGain;
    *pMaxGain = pSensorCtx->AecMaxGain;

    TRACE( IMX307_INFO, "%s: pMinGain:%f,pMaxGain:%f(exit)\n", __FUNCTION__,pSensorCtx->AecMinGain,pSensorCtx->AecMaxGain);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          Sensor instance
 *
 * @param   handle       Sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( IMX307_INFO, "%s: (------oyyf enter) \n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( IMX307_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pSensorCtx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pSensorCtx->AecMaxIntegrationTime;

    TRACE( IMX307_INFO, "%s: (------oyyf exit) (\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          IMX307_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  Sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX307_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain = pSensorCtx->AecCurGain;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          IMX307_IsiGetColorIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  IMX307 sensor instance handle
 * @param   pSetGain                get color
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX307_IsiGetColorIss
(
    IsiSensorHandle_t   handle,
    char				*pGetColor
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pGetColor == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    *pGetColor = COLOUR;


    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          IMX307_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  Sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX307_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pSensorCtx->AecGainIncrement;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  Sensor instance handle
 * @param   NewGain                 gain to be set
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT IMX307_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;

    //TRACE( IMX307_ERROR, "%s: (enter) NewGain value: %f \n", __FUNCTION__, NewGain);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( IMX307_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pSensorCtx->AecMinGain ) NewGain = pSensorCtx->AecMinGain;
    if( NewGain > pSensorCtx->AecMaxGain ) NewGain = pSensorCtx->AecMaxGain;
	
	//TRACE( IMX307_ERROR, "%s: (enter) NewGain value: %f \n", __FUNCTION__, NewGain);

    usGain = (uint16_t)(NewGain * 10 / 3);

	// if(NewGain >=1.0 && NewGain < 2.0)
	// 	usGain = (uint16_t)((20*NewGain - 20) + 0.5);
	// else if(NewGain >=2.0 && NewGain < 4.0)
	// 	usGain = (uint16_t)((10*NewGain - 0) + 0.5);
	// else if(NewGain >=4.0 && NewGain < 8.0)
	// 	usGain = (uint16_t)((5*NewGain  + 20) + 0.5);
	// else if(NewGain >=8.0 && NewGain < 16.0)
	// 	usGain = (uint16_t)((2.5*NewGain  + 40) + 0.5);
	// else if(NewGain >=16.0 && NewGain < 32.0)
	// 	usGain = (uint16_t)((1.25*NewGain  + 60) + 0.5);
	// else if(NewGain >=32.0 && NewGain <= 64.0)
	// 	usGain = (uint16_t)((0.625*NewGain  + 80) + 0.5);
		
     //TRACE( IMX307_ERROR, "-----------%s---: usGain=%d ,NewGain=%f, curGain(%f)\n", \
      __FUNCTION__, usGain, NewGain,pSensorCtx->AecCurGain);
     //TRACE( IMX307_ERROR, "--%s: psetgain=%f,usGain=%d ,NewGain=%f, curGain(%f)\n", \
     __FUNCTION__, *pSetGain, usGain, NewGain,pSensorCtx->AecCurGain);

    // write new gain into sensor registers, do not write if nothing has changed
    if( (NewGain != pSensorCtx->AecCurGain) )
    {
        result = IMX307_IsiRegWriteIss( pSensorCtx, 0x3014, (usGain&0xff));
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        pSensorCtx->OldGain = pSensorCtx->AecCurGain;
    }


    //calculate gain actually set
    pSensorCtx->AecCurGain = NewGain;

    //return current state
    *pSetGain = pSensorCtx->AecCurGain;
    //TRACE( IMX307_ERROR, "--%s: psetgain=%f,usGain=%d ,NewGain=%f, curGain(%f)\n", \
     __FUNCTION__, *pSetGain, usGain, NewGain,pSensorCtx->AecCurGain);

    //TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

     

/*****************************************************************************/
/**
 *          IMX307_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  Sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX307_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  Sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX307_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (-------oyyf)(enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pSensorCtx->AecIntegrationTimeIncrement;

    TRACE( IMX307_INFO, "%s: (------oyyf)(exit) pSensorCtx->AecIntegrationTimeIncrement(%f)\n", __FUNCTION__,pSensorCtx->AecIntegrationTimeIncrement);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  Sensor instance handle
 * @param   NewIntegrationTime      integration time to be set
 * @param   pSetIntegrationTime     set integration time
 * @param   pNumberOfFramesToSkip   number of frames to skip until AE is
 *                                  executed again
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 * @retval  RET_DIVISION_BY_ZERO
 *
 *****************************************************************************/
RESULT IMX307_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
    //uint32_t FineIntegrationTime   = 0; //not supported by Sensor
	uint32_t result_intertime= 0;
    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods


    TRACE( IMX307_DEBUG, "%s: (enter) NewIntegrationTime: %f (min: %f   max: %f)\n", __FUNCTION__,
        NewIntegrationTime,
        pSensorCtx->AecMinIntegrationTime,
        pSensorCtx->AecMaxIntegrationTime);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( IMX307_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range

	//hkw
	//if ( NewIntegrationTime > 0.15 ) NewIntegrationTime = 0.15;
   if ( NewIntegrationTime > pSensorCtx->AecMaxIntegrationTime ) NewIntegrationTime = pSensorCtx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pSensorCtx->AecMinIntegrationTime ) NewIntegrationTime = pSensorCtx->AecMinIntegrationTime;

    //the actual integration time is given by
    //integration_time = ( coarse_integration_time * line_length_pck + fine_integration_time ) / vt_pix_clk_freq
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck )
    //fine_integration_time   = integration_time * vt_pix_clk_freq - coarse_integration_time * line_length_pck
    //
    //fine integration is not supported by Sensor
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck + 0.5 )

    ShutterWidthPck = NewIntegrationTime * ( (float)pSensorCtx->VtPixClkFreq );

    // avoid division by zero
    if ( pSensorCtx->LineLengthPck == 0 )
    {
        TRACE( IMX307_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    //calculate the integer part of the integration time in units of line length
    //calculate the fractional part of the integration time in units of pixel clocks
    //CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pSensorCtx->LineLengthPck) );
    //FineIntegrationTime   = ( (uint32_t)ShutterWidthPck ) - ( CoarseIntegrationTime * pSensorCtx->LineLengthPck );
    CoarseIntegrationTime = (uint32_t)(pSensorCtx->FrameLengthLines - ShutterWidthPck / ((float)pSensorCtx->LineLengthPck) - 1.0f );

    // write new integration time into sensor registers
    // do not write if nothing has changed
    if(pSensorCtx->OldCoarseIntegrationTime != CoarseIntegrationTime)
    {
        result = IMX307_IsiRegWriteIss( pSensorCtx, 0x3022, (CoarseIntegrationTime & 0x00030000U)>>16);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );	
        result = IMX307_IsiRegWriteIss( pSensorCtx, 0x3021, (CoarseIntegrationTime & 0x0000FF00U)>>8);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = IMX307_IsiRegWriteIss( pSensorCtx, 0x3020, (CoarseIntegrationTime & 0x000000FFU));
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );


        pSensorCtx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
        *pNumberOfFramesToSkip = 1U; //skip 1 frame
    }
    else
    {
        *pNumberOfFramesToSkip = 0U;
    }

    //if( FineIntegrationTime != pSensorCtx->OldFineIntegrationTime )
    //{
    //    result = IMX307_IsiRegWriteIss( pSensorCtx, ... , FineIntegrationTime );
    //    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    //    pSensorCtx->OldFineIntegrationTime = FineIntegrationTime; //remember current integration time
    //    *pNumberOfFramesToSkip = 1U; //skip 1 frame
    //}

    //calculate integration time actually set
    //pSensorCtx->AecCurIntegrationTime = ( ((float)CoarseIntegrationTime) * ((float)pSensorCtx->LineLengthPck) + ((float)FineIntegrationTime) ) / pSensorCtx->VtPixClkFreq;
    pSensorCtx->AecCurIntegrationTime = ((float)(pSensorCtx->FrameLengthLines - CoarseIntegrationTime - 1 ))* ((float)pSensorCtx->LineLengthPck) / pSensorCtx->VtPixClkFreq;

    //return current state
    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;

   // TRACE( IMX307_ERROR, "%s: SetTi=%f NewTi=%f\n", __FUNCTION__, *pSetIntegrationTime,NewIntegrationTime);
    TRACE( IMX307_INFO, "%s: (exit) settime mubiao(%f) shiji(%f)\n", __FUNCTION__, NewIntegrationTime,*pSetIntegrationTime);
	TRACE( IMX307_INFO, "%s:\n"
         "pSensorCtx->VtPixClkFreq:%f pSensorCtx->LineLengthPck:%x \n"
         "SetTi=%f    NewTi=%f  CoarseIntegrationTime=%x\n"
         "result_intertime = %x\n H:%x\n M:%x\n L:%x\n", __FUNCTION__, 
         pSensorCtx->VtPixClkFreq,pSensorCtx->LineLengthPck,
         *pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime,
         result_intertime,
         (CoarseIntegrationTime & 0x00030000U),
         (CoarseIntegrationTime & 0x0000FF00U),
         (CoarseIntegrationTime & 0x000000FFU));
    return ( result );
}




/*****************************************************************************/
/**
 *          IMX307_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  Sensor instance handle
 * @param   NewGain                 newly calculated gain to be set
 * @param   NewIntegrationTime      newly calculated integration time to be set
 * @param   pNumberOfFramesToSkip   number of frames to skip until AE is
 *                                  executed again
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 * @retval  RET_DIVISION_BY_ZERO
 *
 *****************************************************************************/
RESULT IMX307_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_DEBUG, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( IMX307_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( IMX307_DEBUG, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = IMX307_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = IMX307_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( IMX307_DEBUG, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( IMX307_DEBUG, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX307_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pSensorCtx->AecCurGain;
    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetResolutionIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  sensor instance handle
 * @param   pSettResolution         set resolution
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX307_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pSensorCtx->Config.Resolution;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pSensorCtx             Sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetAfpsInfoHelperIss(
    IMX307_Context_t   *pSensorCtx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (-----------oyyf enter) pAfpsInfo->AecMaxIntTime(%f) pSensorCtx->AecMaxIntegrationTime(%f)\n", __FUNCTION__, pAfpsInfo->AecMaxIntTime,pSensorCtx->AecMaxIntegrationTime);

    DCT_ASSERT(pSensorCtx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pSensorCtx->Config.Resolution = Resolution;

    // tell sensor about that
    result = IMX307_SetupOutputWindowInternal( pSensorCtx, &pSensorCtx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX307_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = IMX307_AecSetModeParameters( pSensorCtx, &pSensorCtx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX307_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pSensorCtx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pSensorCtx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pSensorCtx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pSensorCtx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pSensorCtx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;

    TRACE( IMX307_INFO, "%s: (-----------oyyf exit) pAfpsInfo->AecMaxIntTime(%f) pSensorCtx->AecMaxIntegrationTime(%f)\n", __FUNCTION__, pAfpsInfo->AecMaxIntTime,pSensorCtx->AecMaxIntegrationTime);

    return ( result );
}

/*****************************************************************************/
/**
 *          IMX307_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  Sensor instance handle
 * @param   Resolution              Any resolution within the AFPS group to query;
 *                                  0 (zero) to use the currently configured resolution
 * @param   pAfpsInfo               Reference of AFPS info structure to store the results
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
RESULT IMX307_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;



    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pSensorCtx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pSensorCtx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pSensorCtx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pSensorCtx->AecMaxIntegrationTime;
	TRACE( IMX307_INFO, "#%s: (-----------oyyf) pAfpsInfo->AecMaxIntTime(%f) pSensorCtx->AecMaxIntegrationTime(%f)\n", __FUNCTION__, pAfpsInfo->AecMaxIntTime,pSensorCtx->AecMaxIntegrationTime);

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    IMX307_Context_t *pDummyCtx = (IMX307_Context_t*) malloc( sizeof(IMX307_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( IMX307_ERROR,  "%s: Can't allocate dummy ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pSensorCtx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = IMX307_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx ); \
        if ( lres == RET_SUCCESS ) \
        { \
            ++idx; \
        } \
        else \
        { \
            UPDATE_RESULT( result, lres ); \
        } \
    }

    // check which AFPS series is requested and build its params list for the enabled AFPS resolutions
	switch (pSensorCtx->IsiSensorMipiInfo.ucMipiLanes)
		{

			case SUPPORT_MIPI_TWO_LANE:
			{
				switch(Resolution)
				{
			        default:
			            TRACE( IMX307_ERROR,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
			            result = RET_NOTSUPP;
			            break;
					//case ISI_RES_TV1080P60:
					//case ISI_RES_TV1080P50:
					case ISI_RES_TV1080P30:
					//case ISI_RES_TV1080P25:
					//case ISI_RES_TV1080P15:
								
						//TRACE( IMX307_ERROR, "%s: (88888exit)\n", __FUNCTION__);
						//AFPSCHECKANDADD( ISI_RES_TV1080P60);
						//AFPSCHECKANDADD( ISI_RES_TV1080P50);
						AFPSCHECKANDADD( ISI_RES_TV1080P30);
						//AFPSCHECKANDADD( ISI_RES_TV1080P25);
						//AFPSCHECKANDADD( ISI_RES_TV1080P15);
						break;
					
				}
	
				break;
			}
			
			case SUPPORT_MIPI_FOUR_LANE:
			{
				switch(Resolution)
				{
			        default:
			            TRACE( IMX307_ERROR,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
			            result = RET_NOTSUPP;
			            break;
					case ISI_RES_TV1080P60:
					case ISI_RES_TV1080P50:
					case ISI_RES_TV1080P30:
					case ISI_RES_TV1080P25:
					case ISI_RES_TV1080P20:
					case ISI_RES_TV1080P15:
					case ISI_RES_TV1080P10:
								
						//AFPSCHECKANDADD( ISI_RES_TV1080P60);
						//AFPSCHECKANDADD( ISI_RES_TV1080P50);
						AFPSCHECKANDADD( ISI_RES_TV1080P30);
						AFPSCHECKANDADD( ISI_RES_TV1080P25);
						AFPSCHECKANDADD( ISI_RES_TV1080P20);
						AFPSCHECKANDADD( ISI_RES_TV1080P15);
						AFPSCHECKANDADD( ISI_RES_TV1080P10);
						break;
					
				}
	
				break;
			}
			
			default:
            TRACE( IMX307_ERROR,  "%s: pSensorCtx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pSensorCtx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;
			        
	}

    // release dummy context again
    free(pDummyCtx);

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetCalibKFactor
 *
 * @brief   Returns the Sensor specific K-Factor
 *
 * @param   handle       Sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&IMX307_KFactor;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          IMX307_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the Sensor specific PCA-Matrix
 *
 * @param   handle          Sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&IMX307_PCAMatrix;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              Sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&IMX307_SVDMeanValue;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              Sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiCenterLine = (IsiLine_t*)&IMX307_CenterLine;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              Sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&IMX307_AwbClipParm;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              Sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&IMX307_AwbGlobalFadeParm;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              Sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiFadeParam = (IsiAwbFade2Parm_t *)&IMX307_AwbFade2Parm;

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          IMX307_IsiGetIlluProfile
 *
 * @brief   Returns a pointer to illumination profile idetified by CieProfile
 *          bitmask
 *
 * @param   handle              sensor instance handle
 * @param   CieProfile
 * @param   ptIsiIlluProfile    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiIlluProfile == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
    	#if 0
        uint16_t i;

        *ptIsiIlluProfile = NULL;

        /* check if we've a default profile */
        for ( i=0U; i<IMX307_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( IMX307_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &IMX307_IlluProfileDefault[i];
                break;
            }
        }

        result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
		#endif
    }

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetLscMatrixTable
 *
 * @brief   Returns a pointer to illumination profile idetified by CieProfile
 *          bitmask
 *
 * @param   handle              sensor instance handle
 * @param   CieProfile
 * @param   ptIsiIlluProfile    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pLscMatrixTable == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
    	#if 0
        uint16_t i;


        switch ( CieProfile )
        {
            case ISI_CIEPROF_A:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &IMX307_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &IMX307_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( IMX307_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &IMX307_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &IMX307_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( IMX307_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &IMX307_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &IMX307_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( IMX307_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &IMX307_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &IMX307_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( IMX307_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &IMX307_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &IMX307_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( IMX307_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( IMX307_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
		#endif
    }

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

#ifdef IMX307_AF_ENABLE
/*****************************************************************************/
/**
 *          IMX307_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    #if 1
    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);
    #endif
    
    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          Sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;

    #if 1
    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);
    #endif
    
    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          Sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
	IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    
    #if 1
    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
    	TRACE( IMX307_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);
    #endif
    
    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          Sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    #if 1
    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    #endif
    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX307_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    #if 1
    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);
    #endif
    
    return ( result );
}
#endif


/*****************************************************************************/
/**
 *          IMX307_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 ******************************************************************************/
static RESULT IMX307_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    #if 0
    uint32_t ulRegValue = 0UL;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = IMX307_IsiRegReadIss( pSensorCtx, 0x5e00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = IMX307_IsiRegWriteIss( pSensorCtx, 0x5e00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = IMX307_IsiRegReadIss( pSensorCtx, 0x5e00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = IMX307_IsiRegWriteIss( pSensorCtx, 0x5e00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pSensorCtx->TestPattern = enable;
    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    #endif
    return ( result );
}



/*****************************************************************************/
/**
 *          IMX307_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 ******************************************************************************/
static RESULT IMX307_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

    ptIsiSensorMipiInfo->ucMipiLanes = pSensorCtx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pSensorCtx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pSensorCtx->IsiSensorMipiInfo.sensorHalDevID;


    TRACE( IMX307_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT IMX307_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
    	TRACE( IMX307_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( IMX307_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT IMX307_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    IMX307_Context_t *pSensorCtx = (IMX307_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
    	TRACE( IMX307_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( IMX307_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = IMX307_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          IMX307_IsiGetSensorIss
 *
 * @brief   fills in the correct pointers for the sensor description struct
 *
 * @param   param1      pointer to sensor description struct
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX307_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( IMX307_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = IMX307_g_acName;
        pIsiSensor->pRegisterTable                      = IMX307_g_aRegDescription_twolane;
        pIsiSensor->pIsiSensorCaps                      = &IMX307_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer					= IMX307_IsiGetSensorIsiVersion;//oyyf
		pIsiSensor->pIsiGetSensorTuningXmlVersion		= IMX307_IsiGetSensorTuningXmlVersion;//oyyf
        pIsiSensor->pIsiCreateSensorIss                 = IMX307_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = IMX307_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = IMX307_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = IMX307_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = IMX307_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = IMX307_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = IMX307_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = IMX307_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = IMX307_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = IMX307_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = IMX307_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = IMX307_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = IMX307_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = IMX307_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = IMX307_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = IMX307_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = IMX307_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = IMX307_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = IMX307_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = IMX307_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = IMX307_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = IMX307_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = IMX307_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = IMX307_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = IMX307_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = IMX307_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = IMX307_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = IMX307_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = IMX307_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = IMX307_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = IMX307_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = IMX307_IsiGetLscMatrixTable;

        /* AF functions */
#ifdef IMX307_AF_ENABLE
        pIsiSensor->pIsiMdiInitMotoDriveMds             = IMX307_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = IMX307_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = IMX307_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = IMX307_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = IMX307_IsiMdiFocusCalibrate;
#endif
        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = IMX307_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = IMX307_IsiActivateTestPattern;
		pIsiSensor->pIsiGetColorIss						= IMX307_IsiGetColorIss;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( IMX307_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT IMX307_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( IMX307_ERROR,  "%s: Can't allocate IMX307 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = IMX307_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = IMX307_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = IMX307_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = 0x01;
    pSensorI2cInfo->reg_size = 2;
    pSensorI2cInfo->value_size = 1;

    {
        IsiSensorCaps_t Caps;
        sensor_caps_t *pCaps;
        uint32_t lanes,i;        

        for (i=0; i<3; i++) {
            lanes = (1<<i);
            ListInit(&pSensorI2cInfo->lane_res[i]);
            if (g_suppoted_mipi_lanenum_type & lanes) {
                Caps.Index = 0;            
                while(IMX307_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
                    pCaps = malloc(sizeof(sensor_caps_t));
                    if (pCaps != NULL) {
                        memcpy(&pCaps->caps,&Caps,sizeof(IsiSensorCaps_t));
                        ListPrepareItem(pCaps);
                        ListAddTail(&pSensorI2cInfo->lane_res[i], pCaps);
                    }
                    Caps.Index++;
                }
            }
        }
    }
    
    ListInit(&pSensorI2cInfo->chipid_info);

    sensor_chipid_info_t* pChipIDInfo_H = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_H )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_H, 0, sizeof(*pChipIDInfo_H) );    
    pChipIDInfo_H->chipid_reg_addr = IMX307_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = IMX307_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = IMX307_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = IMX307_CHIP_ID_LOW_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_L );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_L );

	//oyyf sensor drv version
	pSensorI2cInfo->sensor_drv_version = CONFIG_SENSOR_DRV_VERSION;
	
    *pdata = pSensorI2cInfo;
    return RET_SUCCESS;
}

/******************************************************************************
 * See header file for detailed comment.
 *****************************************************************************/


/*****************************************************************************/
/**
 */
/*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig =
{
    0,
    IMX307_IsiGetSensorIss,
    {
        0,                      /**< IsiSensor_t.pszName */
        0,                      /**< IsiSensor_t.pRegisterTable */
        0,                      /**< IsiSensor_t.pIsiSensorCaps */
        0,						/**< IsiSensor_t.pIsiGetSensorIsiVer_t>*/   //oyyf add
        0,                      /**< IsiSensor_t.pIsiGetSensorTuningXmlVersion_t>*/   //oyyf add 
        0,                      /**< IsiSensor_t.pIsiWhiteBalanceIlluminationChk>*/   //ddl@rock-chips.com 
        0,                      /**< IsiSensor_t.pIsiWhiteBalanceIlluminationSet>*/   //ddl@rock-chips.com
        0,                      /**< IsiSensor_t.pIsiCheckOTPInfo>*/  //zyc
        0,						/**< IsiSensor_t.pIsiSetSensorOTPInfo>*/  //zyl
        0,						/**< IsiSensor_t.pIsiEnableSensorOTP>*/  //zyl
        0,                      /**< IsiSensor_t.pIsiCreateSensorIss */
        0,                      /**< IsiSensor_t.pIsiReleaseSensorIss */
        0,                      /**< IsiSensor_t.pIsiGetCapsIss */
        0,                      /**< IsiSensor_t.pIsiSetupSensorIss */
        0,                      /**< IsiSensor_t.pIsiChangeSensorResolutionIss */
        0,                      /**< IsiSensor_t.pIsiSensorSetStreamingIss */
        0,                      /**< IsiSensor_t.pIsiSensorSetPowerIss */
        0,                      /**< IsiSensor_t.pIsiCheckSensorConnectionIss */
        0,                      /**< IsiSensor_t.pIsiGetSensorRevisionIss */
        0,                      /**< IsiSensor_t.pIsiRegisterReadIss */
        0,                      /**< IsiSensor_t.pIsiRegisterWriteIss */
        //0,						/**< IsiSensor_t.pIsiEnableSensorOTP>*/  //zyl
       // 0,                      /**< IsiSensor_t.pIsiIsEvenFieldIss */
        0,                      /**< IsiSensor_t.pIsiExposureControlIss */
        0,                      /**< IsiSensor_t.pIsiGetGainLimitsIss */
        0,                      /**< IsiSensor_t.pIsiGetIntegrationTimeLimitsIss */
        0,                      /**< IsiSensor_t.pIsiGetCurrentExposureIss */
        0,                      /**< IsiSensor_t.pIsiGetGainIss */
        0,                      /**< IsiSensor_t.pIsiGetGainIncrementIss */
        0,                      /**< IsiSensor_t.pIsiSetGainIss */
        0,                      /**< IsiSensor_t.pIsiGetIntegrationTimeIss */
        0,                      /**< IsiSensor_t.pIsiGetIntegrationTimeIncrementIss */
        0,                      /**< IsiSensor_t.pIsiSetIntegrationTimeIss */
        0,                      /**< IsiSensor_t.pIsiGetResolutionIss */
        0,                      /**< IsiSensor_t.pIsiGetAfpsInfoIss */

        0,                      /**< IsiSensor_t.pIsiGetCalibKFactor */
        0,                      /**< IsiSensor_t.pIsiGetCalibPcaMatrix */
        0,                      /**< IsiSensor_t.pIsiGetCalibSvdMeanValue */
        0,                      /**< IsiSensor_t.pIsiGetCalibCenterLine */
        0,                      /**< IsiSensor_t.pIsiGetCalibClipParam */
        0,                      /**< IsiSensor_t.pIsiGetCalibGlobalFadeParam */
        0,                      /**< IsiSensor_t.pIsiGetCalibFadeParam */
        0,                      /**< IsiSensor_t.pIsiGetIlluProfile */
        0,                      /**< IsiSensor_t.pIsiGetLscMatrixTable */

        0,                      /**< IsiSensor_t.pIsiMdiInitMotoDriveMds */
        0,                      /**< IsiSensor_t.pIsiMdiSetupMotoDrive */
        0,                      /**< IsiSensor_t.pIsiMdiFocusSet */
        0,                      /**< IsiSensor_t.pIsiMdiFocusGet */
        0,                      /**< IsiSensor_t.pIsiMdiFocusCalibrate */

        0,                      /**< IsiSensor_t.pIsiGetSensorMipiInfoIss */

        0,                      /**< IsiSensor_t.pIsiActivateTestPattern */
        0,
        0,						/**< IsiSensor_t.pIsiGetColorIss */
    },
    IMX307_IsiGetSensorI2cInfo,
};
