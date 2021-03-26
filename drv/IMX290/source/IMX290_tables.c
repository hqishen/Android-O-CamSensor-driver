/******************************************************************************
 *
 * The copyright in this software is owned by Rockchip and/or its licensors.
 * This software is made available subject to the conditions of the license 
 * terms to be determined and negotiated by Rockchip and you.
 * THIS SOFTWARE IS PROVIDED TO YOU ON AN "AS IS" BASIS and ROCKCHP AND/OR 
 * ITS LICENSORS DISCLAIMS ANY AND ALL WARRANTIES AND REPRESENTATIONS WITH 
 * RESPECT TO SUCH SOFTWARE, WHETHER EXPRESS,IMPLIED, STATUTORY OR OTHERWISE, 
 * INCLUDING WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF TITLE, NON-INFRINGEMENT, 
 * MERCHANTABILITY, SATISFACTROY QUALITY, ACCURACY OR FITNESS FOR A PARTICULAR PURPOSE. 
 * Except as expressively authorized by Rockchip and/or its licensors, you may not 
 * (a) disclose, distribute, sell, sub-license, or transfer this software to any third party, 
 * in whole or part; (b) modify this software, in whole or part; (c) decompile, reverse-engineer, 
 * dissemble, or attempt to derive any source code from the software.
 *
 *****************************************************************************/
#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "IMX290_MIPI_priv.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * GLOBALS
 *****************************************************************************/

// Image sensor register settings default values taken from data sheet OV13850_DS_1.1_SiliconImage.pdf.
// The settings may be altered by the code in IsiSetupSensor.

//four lane
const IsiRegDescription_t Sensor_IMX290_g_aRegDescription_fourlane[] =
{
	/* CHIP ID = 02h */
	{0x3000, 0x01, "0x0100", eReadWrite},   // standby
	{0x3001, 0x00, "0x0100", eReadWrite}, 	//
	{0x3002, 0x01, "0x0100", eReadWrite}, 
	{0x3003, 0x00, "0x0100", eReadWrite},  	// sw reset, not reset
	{0x3004, 0x10, "0x0100", eReadWrite},	//
	{0x3005, 0x00, "0x0100", eReadWrite}, 	// 10 bit A/D Conversion Bits Setting
	{0x3006, 0x00, "0x0100", eReadWrite}, 	//
	{0x3007, 0x00, "0x0100", eReadWrite},	// Full HD 1080p
	{0x3008, 0xA0, "0x0100", eReadWrite},	// Default 
	{0x3009, 0x12, "0x0100", eReadWrite}, 	//HCG & FRSEL
	{0x300A, 0xC8, "0x0100", eReadWrite}, 	// black level offset value
	{0x300B, 0x00, "0x0100", eReadWrite}, 	// default
	{0x300C, 0x00, "0x0100", eReadWrite},	// Default
	{0x300D, 0x00, "0x0100", eReadWrite},	// Default
	{0x300E, 0x01, "0x0100", eReadWrite},	// Default
	{0x300F, 0x00, "0x0100", eReadWrite},	 	
	{0x3010, 0x21, "0x0100", eReadWrite}, 
	{0x3011, 0x00, "0x0100", eReadWrite}, 
	{0x3012, 0x64, "0x0100", eReadWrite}, 
	{0x3013, 0x00, "0x0100", eReadWrite}, 
	{0x3014, 0x00, "0x0100", eReadWrite},	// Gain (0 ~ 72 / 0.3 step)
	{0x3015, 0x00, "0x0100", eReadWrite},	
	{0x3016, 0x09, "0x0100", eReadWrite}, 
	{0x3017, 0x00, "0x0100", eReadWrite},
	{0x3018, 0x65, "0x0100", eReadWrite},	// vertical span setting
	{0x3019, 0x04, "0x0100", eReadWrite},	//  1125 lines
	{0x301A, 0x00, "0x0100", eReadWrite},
	{0x301B, 0x00, "0x0100", eReadWrite},	
	{0x301C, 0x30, "0x0100", eReadWrite}, 	// 1130 for 30 FPS
	{0x301D, 0x11, "0x0100", eReadWrite}, 
	{0x301E, 0xB2, "0x0100", eReadWrite},
	{0x301F, 0x01, "0x0100", eReadWrite},
	{0x3020, 0x00, "0x0100", eReadWrite},	// SHS1
	{0x3021, 0x00, "0x0100", eReadWrite},	// 
	{0x3022, 0x00, "0x0100", eReadWrite},	//
	{0x303A, 0x0C, "0x0100", eReadWrite},	//
	{0x303B, 0x00, "0x0100", eReadWrite},	//
	{0x303C, 0x00, "0x0100", eReadWrite},	//
	{0x303D, 0x00, "0x0100", eReadWrite},	//
	{0x303E, 0x49, "0x0100", eReadWrite},	//
	{0x303F, 0x04, "0x0100", eReadWrite},	//
	{0x3040, 0x00, "0x0100", eReadWrite},	//
	{0x3041, 0x00, "0x0100", eReadWrite},	//
	{0x3042, 0x9C, "0x0100", eReadWrite},	//
	{0x3043, 0x07, "0x0100", eReadWrite},	//
	{0x3046, 0x00, "0x0100", eReadWrite},	// 10 bit
	{0x3047, 0x01, "0x0100", eReadWrite},	//
	{0x3048, 0x00, "0x0100", eReadWrite},	// XVS
	{0x3049, 0x08, "0x0100", eReadWrite},	// XHSLNG
	{0x304A, 0x00, "0x0100", eReadWrite},	//
	{0x304B, 0x00, "0x0100", eReadWrite},	// XVSOUTSEL, XHSOUTSEL
	{0x305C, 0x18, "0x0100", eReadWrite}, 	// 37.125
	{0x305D, 0x03, "0x0100", eReadWrite}, 	// 37.125
	{0x305E, 0x20, "0x0100", eReadWrite}, 	// 37.125
	{0x305F, 0x01, "0x0100", eReadWrite}, 	// 37.125
	{0x3070, 0x02, "0x0100", eReadWrite}, 
	{0x3071, 0x11, "0x0100", eReadWrite}, 
	{0x309B, 0x10, "0x0100", eReadWrite}, 
	//{0x309C, 0x22, "0x0100", eReadWrite}, 
	{0x30A2, 0x02, "0x0100", eReadWrite}, 
	{0x30A6, 0x20, "0x0100", eReadWrite}, 
	{0x30A7, 0x00, "0x0100", eReadWrite}, 
	{0x30A8, 0x20, "0x0100", eReadWrite}, 
	{0x30A9, 0x00, "0x0100", eReadWrite},
	{0x30AA, 0x20, "0x0100", eReadWrite}, 
	{0x30AB, 0x00, "0x0100", eReadWrite},
	{0x30AC, 0x20, "0x0100", eReadWrite}, 
	{0x30B0, 0x43, "0x0100", eReadWrite}, 
	
	/* CHIP ID = 03h */
	{0x3119, 0x9E, "0x0100", eReadWrite}, 
	{0x311C, 0x1E, "0x0100", eReadWrite}, 
	{0x311D, 0x00, "0x0100", eReadWrite},
	{0x311E, 0x08, "0x0100", eReadWrite}, 
	{0x3128, 0x05, "0x0100", eReadWrite}, 
	{0x3129, 0x1D, "0x0100", eReadWrite}, 	// 10bit ADC
	{0x313D, 0x83, "0x0100", eReadWrite}, 
	{0x3150, 0x03, "0x0100", eReadWrite}, 
	{0x315E, 0x1A, "0x0100", eReadWrite}, 	// 37.125
	{0x3164, 0x1A, "0x0100", eReadWrite}, 	// 37.125
	{0x317C, 0x12, "0x0100", eReadWrite}, 	// 10 bit
	{0x317D, 0x00, "0x0100", eReadWrite},
	{0x317E, 0x00, "0x0100", eReadWrite}, 
	{0x31EC, 0x37, "0x0100", eReadWrite}, 	// 10 bit
	
	/* CHIP ID = 04h */
	{0x32B8, 0x50, "0x0100", eReadWrite}, 
	{0x32B9, 0x10, "0x0100", eReadWrite}, 
	{0x32BA, 0x00, "0x0100", eReadWrite}, 
	{0x32BB, 0x04, "0x0100", eReadWrite}, 
	{0x32C8, 0x50, "0x0100", eReadWrite}, 
	{0x32C9, 0x10, "0x0100", eReadWrite}, 
	{0x32CA, 0x00, "0x0100", eReadWrite}, 
	{0x32CB, 0x04, "0x0100", eReadWrite}, 
	
	/* CHIP ID = 05h */
	{0x332C, 0xD3, "0x0100", eReadWrite}, 
	{0x332D, 0x10, "0x0100", eReadWrite}, 
	{0x332E, 0x0D, "0x0100", eReadWrite}, 
	{0x3358, 0x06, "0x0100", eReadWrite}, 
	{0x3359, 0xE1, "0x0100", eReadWrite}, 
	{0x335A, 0x11, "0x0100", eReadWrite}, 
	{0x3360, 0x1E, "0x0100", eReadWrite}, 
	{0x3361, 0x61, "0x0100", eReadWrite}, 
	{0x3362, 0x10, "0x0100", eReadWrite}, 
	{0x33B0, 0x50, "0x0100", eReadWrite}, 
	{0x33B2, 0x1A, "0x0100", eReadWrite}, 
	{0x33B3, 0x04, "0x0100", eReadWrite}, 
	
	/* CHIP ID = 06h */
	{0x3405, 0x20, "0x0100", eReadWrite}, /*Refer to Output signal Interface Control */
	{0x3406, 0x00, "0x0100", eReadWrite},
	{0x3407, 0x03, "0x0100", eReadWrite}, 
	{0x3414, 0x0A, "0x0100", eReadWrite}, 	// OPB_SIZE_V
	{0x3418, 0x49, "0x0100", eReadWrite}, 	// Y_OUT_SIZE
	{0x3419, 0x04, "0x0100", eReadWrite}, 
	{0x342C, 0x47, "0x0100", eReadWrite},	// Global timing setting
	{0x342D, 0x00, "0x0100", eReadWrite},	// Global timing setting
	{0x3430, 0x0F, "0x0100", eReadWrite},	// Global timing setting
	{0x3431, 0x00, "0x0100", eReadWrite},	// Global timing setting
	{0x3441, 0x0A, "0x0100", eReadWrite}, 	// RAW10: 0A0Ah
	{0x3442, 0x0A, "0x0100", eReadWrite}, 	// RAW10: 0A0Ah
	{0x3443, 0x03, "0x0100", eReadWrite}, 	// 4 lane
	{0x3444, 0x20, "0x0100", eReadWrite}, 
	{0x3445, 0x25, "0x0100", eReadWrite}, 	
	{0x3446, 0x47, "0x0100", eReadWrite}, 	// Global timing setting
	{0x3447, 0x00, "0x0100", eReadWrite}, 	// Global timing setting
	{0x3448, 0x1F, "0x0100", eReadWrite}, 	// Global timing setting
	{0x3449, 0x00, "0x0100", eReadWrite}, 	// Global timing setting
	{0x344A, 0x17, "0x0100", eReadWrite}, 	// Global timing setting
	{0x344B, 0x00, "0x0100", eReadWrite}, 	// Global timing setting
	{0x344C, 0x0F, "0x0100", eReadWrite}, 	// Global timing setting
	{0x344D, 0x00, "0x0100", eReadWrite}, 	// Global timing setting
	{0x344E, 0x17, "0x0100", eReadWrite}, 	// Global timing setting
	{0x344F, 0x00, "0x0100", eReadWrite}, 	// Global timing setting
	{0x3450, 0x47, "0x0100", eReadWrite}, 	// Global timing setting
	{0x3451, 0x00, "0x0100", eReadWrite},	// Global timing setting
	{0x3452, 0x0F, "0x0100", eReadWrite}, 	// Global timing setting
	{0x3453, 0x00, "0x0100", eReadWrite}, 	// Global timing setting
	{0x3454, 0x0F, "0x0100", eReadWrite}, 
	{0x3455, 0x00, "0x0100", eReadWrite}, 
	{0x3472, 0x9C, "0x0100", eReadWrite},	// X_OUT_SIZE
	{0x3473, 0x07, "0x0100", eReadWrite},	// X_OUT_SIZE
	//{0x3480, 0x49, "0x0100", eReadWrite}, 
#if 0	
	/* CHIP ID = 02h */
	{0x3000, 0x00, "0x0100", eReadWrite}, 
	
	{0x0, 0x28, "0x0100", eDelay}, 		//delay 40ms
	
	{0x3002, 0x00, "0x0100", eReadWrite}, 
#endif	
//	{0x304B , 0x0A, "0x0100", eReadWrite}, 	//HSYNC & VSYNC output
	
	{0x0000,   0x00, "eTableEnd",eTableEnd}

};




