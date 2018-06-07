/* 
 *Copyright (C) 2015, DJI Innovations, Inc. All rights reserved.
 *
 */
 

#ifndef __DJICAM_H
#define __DJICAM_H

#define DISPLAY_MODE            (1 << 0)
#define GETBUFFER_MODE          (1 << 1)
#define TRANSFER_MODE           (1 << 2)

#define CAM_BLOCK                1
#define CAM_NON_BLOCK            0

#ifdef __cplusplus
extern "C" {
#endif
   int manifold_cam_init(int mode);
   int manifold_cam_read(unsigned char *buffer, unsigned int *nframe, unsigned int block);
   int manifold_cam_exit();
#ifdef __cplusplus
}
#endif

#endif
