//SP5506_tables.c
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
/*
#include "stdinc.h"

#if( SP5506_DRIVER_USAGE == USE_CAM_DRV_EN )
*/


#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "SP5506_MIPI_priv.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * GLOBALS
 *****************************************************************************/

// Image sensor register settings default values taken from sp
// The settings may be altered by the code in IsiSetupSensor.

//two lane
const IsiRegDescription_t Sensor_g_aRegDescription_twolane[] =
{
    //;Xclk 24Mhz
    //;Pclk clock frequency: 45Mhz
    //;linelength = 750(0x2ee)
    //;framelength = 2000(0x7d0)
    //;grabwindow_width  = 2592
    //;grabwindow_height = 1944
    //;max_framerate: 30fps 
    //;mipi_datarate per lane: 900Mbps
    
    {0x0103, 0x01,"0x0100",eReadWrite},   //software reset
    {0x0000,0x0a,"0x0100",eDelay},
    {0x0100,0x00,"0x0100",eReadWrite},
    {0x0300,0x05,"0x0100",eReadWrite},
    {0x0302,0x96,"0x0100",eReadWrite},
    {0x0303,0x00,"0x0100",eReadWrite},
    {0x3002,0x21,"0x0100",eReadWrite},
    {0x3107,0x23,"0x0100",eReadWrite},
    {0x3501,0x20,"0x0100",eReadWrite},
    {0x3503,0x0c,"0x0100",eReadWrite},  //gain delay 1 frame, exposure delay 1 frame, real gain
    {0x3508,0x03,"0x0100",eReadWrite},
    {0x3509,0x00,"0x0100",eReadWrite},
    {0x3600,0x66,"0x0100",eReadWrite},
    {0x3602,0x30,"0x0100",eReadWrite},
    {0x3610,0xa5,"0x0100",eReadWrite},
    {0x3612,0x93,"0x0100",eReadWrite},
    {0x3620,0x80,"0x0100",eReadWrite},
    {0x3642,0x0e,"0x0100",eReadWrite},
    {0x3661,0x00,"0x0100",eReadWrite},
    {0x3662,0x10,"0x0100",eReadWrite},
    {0x3664,0xf3,"0x0100",eReadWrite},
    {0x3665,0x9e,"0x0100",eReadWrite},
    {0x3667,0xa5,"0x0100",eReadWrite},
    {0x366e,0x55,"0x0100",eReadWrite},
    {0x366f,0x55,"0x0100",eReadWrite},
    {0x3670,0x11,"0x0100",eReadWrite},
    {0x3671,0x11,"0x0100",eReadWrite},
    {0x3672,0x11,"0x0100",eReadWrite},
    {0x3673,0x11,"0x0100",eReadWrite},
    {0x3714,0x24,"0x0100",eReadWrite},
    {0x371a,0x3e,"0x0100",eReadWrite},
    {0x3733,0x10,"0x0100",eReadWrite},
    {0x3734,0x00,"0x0100",eReadWrite},
    {0x373d,0x24,"0x0100",eReadWrite},
    {0x3764,0x20,"0x0100",eReadWrite},
    {0x3765,0x20,"0x0100",eReadWrite},
    {0x3766,0x12,"0x0100",eReadWrite},
    {0x37a1,0x14,"0x0100",eReadWrite},
    {0x37a8,0x1c,"0x0100",eReadWrite},
    {0x37ab,0x0f,"0x0100",eReadWrite},
    {0x37c2,0x04,"0x0100",eReadWrite},
    {0x37cb,0x09,"0x0100",eReadWrite},
    {0x37cc,0x15,"0x0100",eReadWrite},
    {0x37cd,0x1f,"0x0100",eReadWrite},
    {0x37ce,0x00,"0x0100",eReadWrite},
    {0x37d8,0x02,"0x0100",eReadWrite},
    {0x37d9,0x08,"0x0100",eReadWrite},
    {0x37dc,0x04,"0x0100",eReadWrite},
    {0x3800,0x00,"0x0100",eReadWrite},
    {0x3801,0x00,"0x0100",eReadWrite},
    {0x3802,0x00,"0x0100",eReadWrite},
    {0x3803,0x04,"0x0100",eReadWrite},
    {0x3804,0x0a,"0x0100",eReadWrite},
    {0x3805,0x3f,"0x0100",eReadWrite},
    {0x3806,0x07,"0x0100",eReadWrite},
    {0x3807,0xb3,"0x0100",eReadWrite},
    {0x3808,0x0a,"0x0100",eReadWrite},
    {0x3809,0x20,"0x0100",eReadWrite},
    {0x380a,0x07,"0x0100",eReadWrite},
    {0x380b,0x98,"0x0100",eReadWrite},
    {0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0xee,"0x0100",eReadWrite},
    {0x380e,0x07,"0x0100",eReadWrite},
    {0x380f,0xd0,"0x0100",eReadWrite},
    {0x3811,0x10,"0x0100",eReadWrite},
    {0x3813,0x0c,"0x0100",eReadWrite},
    {0x3814,0x01,"0x0100",eReadWrite},
    {0x3815,0x01,"0x0100",eReadWrite},
    {0x3816,0x01,"0x0100",eReadWrite},
    {0x3817,0x01,"0x0100",eReadWrite},
    {0x381e,0x02,"0x0100",eReadWrite},
    {0x3820,0x88,"0x0100",eReadWrite},
    {0x3821,0x01,"0x0100",eReadWrite},
    {0x3832,0x04,"0x0100",eReadWrite},
    {0x3c80,0x01,"0x0100",eReadWrite},
    {0x3c82,0x00,"0x0100",eReadWrite},
    {0x3c83,0xc8,"0x0100",eReadWrite},
    {0x3c8c,0x10,"0x0100",eReadWrite},
    {0x3c8d,0x00,"0x0100",eReadWrite},
    {0x3c90,0x07,"0x0100",eReadWrite},
    {0x3c91,0x00,"0x0100",eReadWrite},
    {0x3c92,0x00,"0x0100",eReadWrite},
    {0x3c93,0x00,"0x0100",eReadWrite},
    {0x3c94,0xd0,"0x0100",eReadWrite},
    {0x3c95,0x50,"0x0100",eReadWrite},
    {0x3c96,0x35,"0x0100",eReadWrite},
    {0x3c97,0x00,"0x0100",eReadWrite},
    {0x4001,0xe0,"0x0100",eReadWrite},
    {0x4008,0x02,"0x0100",eReadWrite},
    {0x4009,0x0d,"0x0100",eReadWrite},
    {0x400f,0x80,"0x0100",eReadWrite},
    {0x4013,0x02,"0x0100",eReadWrite},
    {0x4040,0x00,"0x0100",eReadWrite},
    {0x4041,0x07,"0x0100",eReadWrite},
    {0x404c,0x50,"0x0100",eReadWrite},
    {0x404e,0x20,"0x0100",eReadWrite},
    {0x4500,0x06,"0x0100",eReadWrite},
    {0x4503,0x00,"0x0100",eReadWrite},
    {0x450a,0x04,"0x0100",eReadWrite},
    {0x4809,0x04,"0x0100",eReadWrite},
    {0x480c,0x12,"0x0100",eReadWrite},
    {0x4819,0x70,"0x0100",eReadWrite},
    {0x4825,0x32,"0x0100",eReadWrite},
    {0x4826,0x32,"0x0100",eReadWrite},
    {0x482a,0x06,"0x0100",eReadWrite},
    {0x4833,0x08,"0x0100",eReadWrite},
    {0x4837,0x0d,"0x0100",eReadWrite},
    {0x5000,0x77,"0x0100",eReadWrite},
    {0x5b00,0x01,"0x0100",eReadWrite},
    {0x5b01,0x10,"0x0100",eReadWrite},
    {0x5b02,0x01,"0x0100",eReadWrite},
    {0x5b03,0xdb,"0x0100",eReadWrite},
    {0x5b05,0x6c,"0x0100",eReadWrite},
    {0x5e10,0xfc,"0x0100",eReadWrite},
    //max exposure is (VTS-4)/2                                            
    //expo is twice as before. Ex. [3501,3502]=0040 means 8 Tline exposure.
    {0x3500,0x00,"0x0100",eReadWrite},
    {0x3501,0x3E,"0x0100",eReadWrite},   //max expo= ([380e,380f]-4)/2.
    {0x3502,0x60,"0x0100",eReadWrite},    
    //;8xgain 
    {0x3503,0x08,"0x0100",eReadWrite},    //[2]=0 real gain             
    {0x3508,0x04,"0x0100",eReadWrite},    //                            
    {0x3509,0x00,"0x0100",eReadWrite},    //[3508,3509]=0x0080 is 1xgain
    //;Vsync
    {0x3832,0x48,"0x0100",eReadWrite},    //[7:4]vsync_width ; R3002[5] p_fsin_oen
    //;MIPI 
    {0x3c90,0x00,"0x0100",eReadWrite},    //MIPI Continuous mode (07 Gated mode) 
    {0x5780,0x3e,"0x0100",eReadWrite},
    {0x5781,0x0f,"0x0100",eReadWrite},
    {0x5782,0x44,"0x0100",eReadWrite},
    {0x5783,0x02,"0x0100",eReadWrite},
    {0x5784,0x01,"0x0100",eReadWrite},
    {0x5785,0x01,"0x0100",eReadWrite},
    {0x5786,0x00,"0x0100",eReadWrite},
    {0x5787,0x04,"0x0100",eReadWrite},
    {0x5788,0x02,"0x0100",eReadWrite},
    {0x5789,0x0f,"0x0100",eReadWrite},
    {0x578a,0xfd,"0x0100",eReadWrite},
    {0x578b,0xf5,"0x0100",eReadWrite},
    {0x578c,0xf5,"0x0100",eReadWrite},
    {0x578d,0x03,"0x0100",eReadWrite},
    {0x578e,0x08,"0x0100",eReadWrite},
    {0x578f,0x0c,"0x0100",eReadWrite},
    {0x5790,0x08,"0x0100",eReadWrite},
    {0x5791,0x06,"0x0100",eReadWrite},
    {0x5792,0x00,"0x0100",eReadWrite},
    {0x5793,0x52,"0x0100",eReadWrite},
    {0x5794,0xa3,"0x0100",eReadWrite},
    {0x4003,0x40,"0x0100",eReadWrite},    //blc target 0x40
    {0x3107,0x01,"0x0100",eReadWrite},    //keep clock on during dummy lines
    {0x3c80,0x08,"0x0100",eReadWrite},
    {0x3c83,0xb1,"0x0100",eReadWrite},
    {0x3c8c,0x10,"0x0100",eReadWrite},
    {0x3c8d,0x00,"0x0100",eReadWrite},
    {0x3c90,0x00,"0x0100",eReadWrite},
    {0x3c94,0x00,"0x0100",eReadWrite},
    {0x3c95,0x00,"0x0100",eReadWrite},
    {0x3c96,0x00,"0x0100",eReadWrite},
    {0x3d8c,0x71,"0x0100",eReadWrite},    //Header address high byte 
    {0x3d8d,0xe7,"0x0100",eReadWrite},    //Header address low byte  
    {0x37cb,0x09,"0x0100",eReadWrite},        
    {0x37cc,0x15,"0x0100",eReadWrite},
    {0x37cd,0x1f,"0x0100",eReadWrite},
    {0x37ce,0x1f,"0x0100",eReadWrite},
    {0x0100,0x00,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_twolane_resolution_2592_1944[] =
{
    //@@2592x1944_30FPS_MIPI_2_LANE
    //;Xclk 24Mhz
    //;Pclk clock frequency: 45Mhz
    //;linelength = 750(0x2ee)
    //;framelength = 2000(0xfa0)
    //;grabwindow_width  = 2592
    //;grabwindow_height = 1944
    //;max_framerate: 30fps 
    //;mipi_datarate per lane: 900Mbps
    
    {0x0100,0x00,"0x0100",eReadWrite},
    {0x3662,0x10,"0x0100",eReadWrite},
    {0x3714,0x24,"0x0100",eReadWrite},
    {0x371a,0x3e,"0x0100",eReadWrite},
    {0x37c2,0x04,"0x0100",eReadWrite},
    {0x37d9,0x08,"0x0100",eReadWrite},
    {0x3800,0x00,"0x0100",eReadWrite},
    {0x3801,0x00,"0x0100",eReadWrite},
    {0x3802,0x00,"0x0100",eReadWrite},
    {0x3803,0x04,"0x0100",eReadWrite},
    {0x3804,0x0a,"0x0100",eReadWrite},
    {0x3805,0x3f,"0x0100",eReadWrite},
    {0x3806,0x07,"0x0100",eReadWrite},
    {0x3807,0xb3,"0x0100",eReadWrite},
    {0x3808,0x0a,"0x0100",eReadWrite},
    {0x3809,0x20,"0x0100",eReadWrite},
    {0x380a,0x07,"0x0100",eReadWrite},
    {0x380b,0x98,"0x0100",eReadWrite},
    {0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0xee,"0x0100",eReadWrite},
    {0x380e,0x07,"0x0100",eReadWrite},
    {0x380f,0xd0,"0x0100",eReadWrite},
    {0x3811,0x10,"0x0100",eReadWrite},
    {0x3813,0x0c,"0x0100",eReadWrite},
    {0x3814,0x01,"0x0100",eReadWrite},
    {0x3815,0x01,"0x0100",eReadWrite},
    {0x3816,0x01,"0x0100",eReadWrite},
    {0x3820,0x88,"0x0100",eReadWrite},
    {0x3821,0x01,"0x0100",eReadWrite},
    {0x4008,0x02,"0x0100",eReadWrite},
    {0x4009,0x0d,"0x0100",eReadWrite},
    {0x4041,0x07,"0x0100",eReadWrite},
    {0x3501,0x3e,"0x0100",eReadWrite},
    {0x3502,0x60,"0x0100",eReadWrite},
    {0x0100,0x00,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_2592x1944P30_twolane_fpschg[] =
{
    {0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0xee,"0x0100",eReadWrite},
    {0x380e,0x07,"0x0100",eReadWrite},
    {0x380f,0xd0,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_2592x1944P25_twolane_fpschg[] =
{
    {0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0xee,"0x0100",eReadWrite},
    {0x380e,0x09,"0x0100",eReadWrite},
    {0x380f,0x80,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_2592x1944P20_twolane_fpschg[] =
{
    {0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0xee,"0x0100",eReadWrite},
    {0x380e,0x0b,"0x0100",eReadWrite},
    {0x380f,0xd0,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_2592x1944P15_twolane_fpschg[] =
{
    {0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0xee,"0x0100",eReadWrite},
    {0x380e,0x0f,"0x0100",eReadWrite},
    {0x380f,0xc0,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_2592x1944P10_twolane_fpschg[] =
{
    {0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0xee,"0x0100",eReadWrite},
    {0x380e,0x17,"0x0100",eReadWrite},
    {0x380f,0x90,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_twolane_resolution_1296_972[] =
{
    //@@1296X972_30fps
    //;;1296X972_HBIN_VBIN_30FPS_MIPI_2_LANE
    //;Xclk 24Mhz
    //;Pclk clock frequency: 45Mhz
    //;linelength = 750(0x2ee)
    //;framelength = 2000(0x7d0)
    //;grabwindow_width  = 1296
    //;grabwindow_height = 972
    //;max_framerate: 30fps 
    //;mipi_datarate per lane: 900Mbps                
                                                                              
    {0x0100,0x00,"0x0100",eReadWrite},  // 
    {0x3662,0x08,"0x0100",eReadWrite},
    {0x3714,0x28,"0x0100",eReadWrite},
    {0x371a,0x3e,"0x0100",eReadWrite},
    {0x37c2,0x14,"0x0100",eReadWrite},
    {0x37d9,0x04,"0x0100",eReadWrite},
    {0x3800,0x00,"0x0100",eReadWrite},
    {0x3801,0x00,"0x0100",eReadWrite},
    {0x3802,0x00,"0x0100",eReadWrite},
    {0x3803,0x00,"0x0100",eReadWrite},
    {0x3804,0x0a,"0x0100",eReadWrite},
    {0x3805,0x3f,"0x0100",eReadWrite},
    {0x3806,0x07,"0x0100",eReadWrite},
    {0x3807,0xb7,"0x0100",eReadWrite},
    {0x3808,0x05,"0x0100",eReadWrite},
    {0x3809,0x10,"0x0100",eReadWrite},
    {0x380a,0x03,"0x0100",eReadWrite},
    {0x380b,0xcc,"0x0100",eReadWrite},
    {0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0xee,"0x0100",eReadWrite},
    {0x380e,0x07,"0x0100",eReadWrite},
    {0x380f,0xd0,"0x0100",eReadWrite},
    {0x3811,0x08,"0x0100",eReadWrite},
    {0x3813,0x08,"0x0100",eReadWrite},
    {0x3814,0x03,"0x0100",eReadWrite},
    {0x3815,0x01,"0x0100",eReadWrite},
    {0x3816,0x03,"0x0100",eReadWrite},
    {0x3820,0x8b,"0x0100",eReadWrite},
    {0x3821,0x01,"0x0100",eReadWrite},
    {0x4008,0x00,"0x0100",eReadWrite},
    {0x4009,0x07,"0x0100",eReadWrite},
    {0x4041,0x03,"0x0100",eReadWrite},
    {0x3501,0x3e,"0x0100",eReadWrite},    //max expo= ([380e,380f]-4)/2.
    {0x3502,0x60,"0x0100",eReadWrite},
    {0x0100,0x00,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_1296x972P30_twolane_fpschg[] =
{
    {0x380e,0x07,"0x0100",eReadWrite},
    {0x380f,0xd0,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_1296x972P25_twolane_fpschg[] =
{
    {0x380e,0x09,"0x0100",eReadWrite},
    {0x380f,0x60,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_1296x972P20_twolane_fpschg[] =
{
    {0x380e,0x0b,"0x0100",eReadWrite},
    {0x380f,0xb8,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_1296x972P15_twolane_fpschg[] =
{
    {0x380e,0x0f,"0x0100",eReadWrite},
    {0x380f,0xa0,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_1296x972P10_twolane_fpschg[] =
{
    {0x380e,0x17,"0x0100",eReadWrite},
    {0x380f,0x70,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_twolane_resolution_1920_1080[] =
{
    //@@1920x1080_30FPS_MIPI_2_LANE
    //;Xclk 24Mhz
    //;Pclk clock frequency: 45Mhz
    //;linelength = 660(0x294)
    //;framelength = 2272(0x8e0)
    //;grabwindow_width  = 1920
    //;grabwindow_height = 1080
    //;max_framerate: 30fps 
    //;mipi_datarate per lane: 900Mbps

    {0x0100,0x00,"0x0100",eReadWrite},
    {0x3662,0x10,"0x0100",eReadWrite},
    {0x3714,0x24,"0x0100",eReadWrite},
    {0x371a,0x3e,"0x0100",eReadWrite},
    {0x37c2,0x04,"0x0100",eReadWrite},
    {0x37d9,0x08,"0x0100",eReadWrite},
    {0x3800,0x01,"0x0100",eReadWrite},
    {0x3801,0x50,"0x0100",eReadWrite},
    {0x3802,0x01,"0x0100",eReadWrite},
    {0x3803,0xbc,"0x0100",eReadWrite},
    {0x3804,0x08,"0x0100",eReadWrite},
    {0x3805,0xef,"0x0100",eReadWrite},
    {0x3806,0x05,"0x0100",eReadWrite},
    {0x3807,0xfb,"0x0100",eReadWrite},
    {0x3808,0x07,"0x0100",eReadWrite},
    {0x3809,0x80,"0x0100",eReadWrite},
    {0x380a,0x04,"0x0100",eReadWrite},
    {0x380b,0x38,"0x0100",eReadWrite},
    {0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0x94,"0x0100",eReadWrite},
    {0x380e,0x08,"0x0100",eReadWrite},
    {0x380f,0xe0,"0x0100",eReadWrite},
    {0x3811,0x10,"0x0100",eReadWrite},
    {0x3813,0x04,"0x0100",eReadWrite},
    {0x3814,0x01,"0x0100",eReadWrite},
    {0x3815,0x01,"0x0100",eReadWrite},
    {0x3816,0x01,"0x0100",eReadWrite},
    {0x3820,0x88,"0x0100",eReadWrite},
    {0x3821,0x01,"0x0100",eReadWrite},
    {0x4008,0x02,"0x0100",eReadWrite},
    {0x4009,0x0d,"0x0100",eReadWrite},
    {0x4041,0x07,"0x0100",eReadWrite},
    {0x3501,0x46,"0x0100",eReadWrite},    //max expo= ([380e,380f]-4)/2.
    {0x3502,0xe0,"0x0100",eReadWrite},
    //{0x0100,0x01,"0x0100",eReadWrite},
    {0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_1920x1080P30_twolane_fpschg[] =
{
	{0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0x94,"0x0100",eReadWrite},
    {0x380e,0x08,"0x0100",eReadWrite},
    {0x380f,0xe0,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_1920x1080P25_twolane_fpschg[] =
{
	{0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0x94,"0x0100",eReadWrite},
    {0x380e,0x0a,"0x0100",eReadWrite},
    {0x380f,0xc0,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_1920x1080P20_twolane_fpschg[] =
{
	{0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0x94,"0x0100",eReadWrite},
    {0x380e,0x0d,"0x0100",eReadWrite},
    {0x380f,0x70,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_1920x1080P15_twolane_fpschg[] =
{
	{0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0x94,"0x0100",eReadWrite},
    {0x380e,0x11,"0x0100",eReadWrite},
    {0x380f,0xe0,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_1920x1080P10_twolane_fpschg[] =
{
	{0x380c,0x02,"0x0100",eReadWrite},
    {0x380d,0x94,"0x0100",eReadWrite},
    {0x380e,0x1a,"0x0100",eReadWrite},
    {0x380f,0xc0,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

