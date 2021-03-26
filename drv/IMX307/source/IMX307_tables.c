//IMX307_tables.c
/*****************************************************************************/
/*!
 *  \file        IMX307_tables.c \n
 *  \version     1.0 \n
 *  \author      Meinicke \n
 *  \brief       Image-sensor-specific tables and other
 *               constant values/structures for OV13850. \n
 *
 *  \revision    $Revision: 803 $ \n
 *               $Author: $ \n
 *               $Date: 2010-02-26 16:35:22 +0100 (Fr, 26 Feb 2010) $ \n
 *               $Id: OV13850_tables.c 803 2010-02-26 15:35:22Z  $ \n
 */
/*  This is an unpublished work, the copyright in which vests in Silicon Image
 *  GmbH. The information contained herein is the property of Silicon Image GmbH
 *  and is supplied without liability for errors or omissions. No part may be
 *  reproduced or used expect as authorized by contract or other written
 *  permission. Copyright(c) Silicon Image GmbH, 2009, all rights reserved.
 */
/*****************************************************************************/
/*
#include "stdinc.h"

#if( IMX307_DRIVER_USAGE == USE_CAM_DRV_EN )
*/


#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "IMX307_MIPI_priv.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * GLOBALS
 *****************************************************************************/

// Image sensor register settings default values taken from data sheet OV13850_DS_1.1_SiliconImage.pdf.
// The settings may be altered by the code in IsiSetupSensor.

//two lane

const IsiRegDescription_t IMX307_g_aRegDescription_twolane[] =
{



{0x0000, 0x00, "eReadWrite",eTableEnd}



};

const IsiRegDescription_t IMX307_g_1920x1080_twolane[] =
{

//sensor stanby
{0x3000, 0x01, "0x0100", eReadWrite},
{0x3001, 0x01, "0x0100", eReadWrite},
{0x3002, 0x01, "0x0100", eReadWrite},
//setting
{0x3005, 0x00, "0x0100", eReadWrite},
{0x3007, 0x00, "0x0100", eReadWrite},
{0x3009, 0x12, "0x0100", eReadWrite},
{0x300A, 0x10, "0x0100", eReadWrite},
{0x300B, 0x00, "0x0100", eReadWrite},
{0x3011, 0x0A, "0x0100", eReadWrite},
{0x3012, 0x64, "0x0100", eReadWrite},
{0x3014, 0x00, "0x0100", eReadWrite},
{0x3018, 0x65, "0x0100", eReadWrite},
{0x3019, 0x04, "0x0100", eReadWrite},
{0x301A, 0x00, "0x0100", eReadWrite},
{0x301C, 0x30, "0x0100", eReadWrite},
{0x301D, 0x11, "0x0100", eReadWrite},
{0x3020, 0xFE, "0x0100", eReadWrite},
{0x3021, 0x03, "0x0100", eReadWrite},
{0x3022, 0x00, "0x0100", eReadWrite},
{0x3046, 0x00, "0x0100", eReadWrite},
{0x3048, 0x00, "0x0100", eReadWrite},
{0x3049, 0x08, "0x0100", eReadWrite},
{0x304B, 0x0A, "0x0100", eReadWrite},
{0x305C, 0x18, "0x0100", eReadWrite},
{0x305D, 0x03, "0x0100", eReadWrite},
{0x305E, 0x20, "0x0100", eReadWrite},
{0x305F, 0x01, "0x0100", eReadWrite},
{0x309E, 0x4A, "0x0100", eReadWrite},
{0x309F, 0x4A, "0x0100", eReadWrite},

{0x311C, 0x0E, "0x0100", eReadWrite},
{0x3128, 0x04, "0x0100", eReadWrite},
{0x3129, 0x1D, "0x0100", eReadWrite},
{0x313B, 0x41, "0x0100", eReadWrite},
{0x315E, 0x1A, "0x0100", eReadWrite},
{0x3164, 0x1A, "0x0100", eReadWrite},
{0x317C, 0x12, "0x0100", eReadWrite},
{0x31EC, 0x37, "0x0100", eReadWrite},

//These registers are set in CSI-2 interface only., 0x
{0x3405, 0x10, "0x0100", eReadWrite},
{0x3407, 0x01, "0x0100", eReadWrite},
{0x3414, 0x0A, "0x0100", eReadWrite},
{0x3418, 0x49, "0x0100", eReadWrite},
{0x3419, 0x04, "0x0100", eReadWrite},
{0x3441, 0x0A, "0x0100", eReadWrite},
{0x3442, 0x0A, "0x0100", eReadWrite},
{0x3443, 0x01, "0x0100", eReadWrite},
{0x3444, 0x20, "0x0100", eReadWrite},
{0x3445, 0x25, "0x0100", eReadWrite},
{0x3446, 0x57, "0x0100", eReadWrite},
{0x3447, 0x00, "0x0100", eReadWrite},
{0x3448, 0x37, "0x0100", eReadWrite},
{0x3449, 0x00, "0x0100", eReadWrite},
{0x344A, 0x1F, "0x0100", eReadWrite},
{0x344B, 0x00, "0x0100", eReadWrite},
{0x344C, 0x1F, "0x0100", eReadWrite},
{0x344D, 0x00, "0x0100", eReadWrite},
{0x344E, 0x1F, "0x0100", eReadWrite},
{0x344F, 0x00, "0x0100", eReadWrite},
{0x3450, 0x77, "0x0100", eReadWrite},
{0x3451, 0x00, "0x0100", eReadWrite},
{0x3452, 0x1F, "0x0100", eReadWrite},
{0x3453, 0x00, "0x0100", eReadWrite},
{0x3454, 0x17, "0x0100", eReadWrite},
{0x3455, 0x00, "0x0100", eReadWrite},
{0x3472, 0x9C, "0x0100", eReadWrite},
{0x3473, 0x07, "0x0100", eReadWrite},
{0x3480, 0x49, "0x0100", eReadWrite},

//stanby cancel
//{0x3000, 0x00, "0x0100", eReadWrite},
{0x3002, 0x00, "0x0100", eReadWrite},
{0x3001, 0x00, "0x0100", eReadWrite},

	{0x0000, 0x00, "eReadWrite",eTableEnd}
};

const IsiRegDescription_t IMX307_g_1920x1080P30_twolane_fpschg[] =
{
    //{0x3018,0x65, "0x0100", eReadWrite}, 
    //{0x3019,0x04, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};

const IsiRegDescription_t IMX307_g_1920x1080P25_twolane_fpschg[] =
{
    {0x3018,0x46, "0x0100", eReadWrite}, 
    {0x3019,0x05, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};

const IsiRegDescription_t IMX307_g_1920x1080P15_twolane_fpschg[] =
{
    {0x3018,0xca, "0x0100", eReadWrite}, 
    {0x3019,0x08, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};

const IsiRegDescription_t IMX307_g_1920x1080P10_twolane_fpschg[] =
{
    {0x3018,0x2f, "0x0100", eReadWrite}, 
    {0x3019,0x0d, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};

const IsiRegDescription_t IMX307_g_1920x1080P5_twolane_fpschg[] =
{
    {0x3018,0x5e, "0x0100", eReadWrite}, 
    {0x3019,0x1a, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};

//four lane
const IsiRegDescription_t IMX307_g_aRegDescription_fourlane[] =
{
{0x0000, 0x00, "eReadWrite",eTableEnd}

};


const IsiRegDescription_t IMX307_g_1920x1080_fourlane[] =
{

/* CHIP ID = 02h */
	{0x3000, 0x01, "0x0100", eReadWrite},	// standby
	{0x3001, 0x01, "0x0100", eReadWrite},
	{0x3002, 0x01, "0x0100", eReadWrite},
	
	{0x3005, 0x00, "0x0100", eReadWrite}, 	// 10bit : 0x00   12bit :0x01 A/D Conversion Bits Setting
	{0x3007, 0x00, "0x0100", eReadWrite},	// Full HD 1080p
	{0x3009, 0x11, "0x0100", eReadWrite},	// HCG & FRSEL bit[4] 0: LCG mode, 1: HCG mode ;bit[1:0] 1：60FPS, 2: 30FPS
	{0x300A, 0xC8, "0x0100", eReadWrite},	// black level offset value
	{0x300B, 0x00, "0x0100", eReadWrite},
	{0x3011, 0x0A, "0x0100", eReadWrite},	// Set to "0A"
	{0x3012, 0x64, "0x0100", eReadWrite},
	{0x3014, 0x00, "0x0100", eReadWrite},	// Gain (0 ~ 72 / 0.3 step)
	{0x3018, 0x65, "0x0100", eReadWrite},	// vertical span setting
	{0x3019, 0x04, "0x0100", eReadWrite},	// 1125 lines
	{0x301A, 0x00, "0x0100", eReadWrite},
	{0x301C, 0x98, "0x0100", eReadWrite},	// 30 FPS:0x1130, 60 FPS:0x0898
	{0x301D, 0x08, "0x0100", eReadWrite},
	{0x3020, 0xFE, "0x0100", eReadWrite},	// SHS1
	{0x3021, 0x03, "0x0100", eReadWrite},
	{0x3022, 0x00, "0x0100", eReadWrite},
	{0x3046, 0x00, "0x0100", eReadWrite},	// bit[0] 10 bit: 0 12bit: 1
	{0x3048, 0x00, "0x0100", eReadWrite},	// XVS
	{0x3049, 0x08, "0x0100", eReadWrite},	// XHSLNG
	{0x304B, 0x0A, "0x0100", eReadWrite},	// XVSOUTSEL, XHSOUTSEL
	{0x305C, 0x18, "0x0100", eReadWrite},	// 37.125
	{0x305D, 0x03, "0x0100", eReadWrite},	// 37.125
	{0x305E, 0x20, "0x0100", eReadWrite},	// 37.125
	{0x305F, 0x01, "0x0100", eReadWrite},	// 37.125
	{0x309E, 0x4A, "0x0100", eReadWrite},	// Set to "4Ah"
	{0x309F, 0x4A, "0x0100", eReadWrite},	// Set to "4Ah"
	
	/* CHIP ID = 03h */
	{0x311C, 0x0E, "0x0100", eReadWrite},	// Set to "0Eh"
	{0x3128, 0x04, "0x0100", eReadWrite},	// Set to "04h"
	{0x3129, 0x1D, "0x0100", eReadWrite},	// 10bit 0x1D	, 12bit 0x00
	{0x313B, 0x41, "0x0100", eReadWrite},
	{0x315E, 0x1A, "0x0100", eReadWrite},	// 37.125
	{0x3164, 0x1A, "0x0100", eReadWrite},   // 37.125
	{0x317C, 0x12, "0x0100", eReadWrite},   // 10bit 0x12  , 12bit 0x00
	{0x31EC, 0x37, "0x0100", eReadWrite},	// 10bit 0x37  , 12bit 0x0E
	
	/* CHIP ID = 06h */
	//These registers are set in CSI-2 interface only., 
	{0x3405, 0x10, "0x0100", eReadWrite},	// bit[5:4] 0: 	891Mbps/lane ；1: 445.5Mbps/lane；  2:222.75Mbps/Lane
	{0x3407, 0x03, "0x0100", eReadWrite},	// Lane number
	{0x3414, 0x0A, "0x0100", eReadWrite},	// OPB_SIZE_V
	{0x3418, 0x49, "0x0100", eReadWrite},	// Y_OUT_SIZE
	{0x3419, 0x04, "0x0100", eReadWrite},
	{0x3441, 0x0A, "0x0100", eReadWrite},	// RAW10: 0A0Ah
	{0x3442, 0x0A, "0x0100", eReadWrite},
	{0x3443, 0x03, "0x0100", eReadWrite},	// 4 lane
	{0x3444, 0x20, "0x0100", eReadWrite},	// 2520h: INCK = 37.125 MHz
	{0x3445, 0x25, "0x0100", eReadWrite},
	{0x3446, 0x47, "0x0100", eReadWrite},	// Global timing setting
	{0x3447, 0x00, "0x0100", eReadWrite},   // Global timing setting
	{0x3448, 0x1F, "0x0100", eReadWrite},   // Global timing setting
	{0x3449, 0x00, "0x0100", eReadWrite},   // Global timing setting
	{0x344A, 0x17, "0x0100", eReadWrite},   // Global timing setting
	{0x344B, 0x00, "0x0100", eReadWrite},   // Global timing setting
	{0x344C, 0x0F, "0x0100", eReadWrite},   // Global timing setting
	{0x344D, 0x00, "0x0100", eReadWrite},   // Global timing setting
	{0x344E, 0x17, "0x0100", eReadWrite},   // Global timing setting
	{0x344F, 0x00, "0x0100", eReadWrite},   // Global timing setting
	{0x3450, 0x47, "0x0100", eReadWrite},   // Global timing setting
	{0x3451, 0x00, "0x0100", eReadWrite},   // Global timing setting
	{0x3452, 0x0F, "0x0100", eReadWrite},   // Global timing setting
	{0x3453, 0x00, "0x0100", eReadWrite},   // Global timing setting
	{0x3454, 0x0F, "0x0100", eReadWrite},	// Global timing setting
	{0x3455, 0x00, "0x0100", eReadWrite},	// Global timing setting
	{0x3472, 0x9C, "0x0100", eReadWrite},	// X_OUT_SIZE
	{0x3473, 0x07, "0x0100", eReadWrite},	// X_OUT_SIZE
	{0x3480, 0x49, "0x0100", eReadWrite},	// INCK = 37.125MHC 49h
	
	{0x0000, 0x00, "0x0100", eDelay}, 		//delay
	/* CHIP ID = 02h */
	{0x3002, 0x00, "0x0100", eReadWrite},
	{0x3001, 0x00, "0x0100", eReadWrite},
	//{0x3000, 0x00, "0x0100", eReadWrite},	//stanby cancel

	{0x0000, 0x00, "eReadWrite",eTableEnd}
};

const IsiRegDescription_t IMX307_g_1920x1080P60_fourlane_fpschg[] =
{
    {0x3018,0x65, "0x0100", eReadWrite},
    {0x3019,0x04, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};
const IsiRegDescription_t IMX307_g_1920x1080P50_fourlane_fpschg[] =
{
    {0x3018,0x46, "0x0100", eReadWrite}, 
    {0x3019,0x05, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};
const IsiRegDescription_t IMX307_g_1920x1080P30_fourlane_fpschg[] =
{
    {0x3018,0xca, "0x0100", eReadWrite}, 
    {0x3019,0x08, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};

const IsiRegDescription_t IMX307_g_1920x1080P25_fourlane_fpschg[] =
{
    {0x3018,0x8c, "0x0100", eReadWrite}, 
    {0x3019,0x0a, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};
const IsiRegDescription_t IMX307_g_1920x1080P20_fourlane_fpschg[] =
{
    {0x3018,0x2f, "0x0100", eReadWrite}, 
    {0x3019,0x0d, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};

const IsiRegDescription_t IMX307_g_1920x1080P15_fourlane_fpschg[] =
{
    {0x3018,0x94, "0x0100", eReadWrite}, 
    {0x3019,0x11, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};

const IsiRegDescription_t IMX307_g_1920x1080P10_fourlane_fpschg[] =
{
    {0x3018,0x5e, "0x0100", eReadWrite}, 
    {0x3019,0x1a, "0x0100", eReadWrite},
	{0x0000, 0x00, "eReadWrite",eTableEnd}
	
};







