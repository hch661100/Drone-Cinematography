#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sys/time.h>


#include <setjmp.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include "djicam.h"

#include "cv.h"
#include "highgui.h"


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/ByteMultiArray.h"

typedef unsigned char   BYTE;
#define IMAGE_W 1280
#define IMAGE_H 720
//#define FRAME_SIZE              IMAGE_W*IMAGE_H*3

#define FRAME_SIZE              (1280*720*3)  /*format NV12*/
#define BLOCK_MODE                     1
#define TRANSFER_MODE           (1 << 2)

static unsigned char buffer[FRAME_SIZE] = {0};
static unsigned int nframe = 0;
static int mode = 0;

IplImage *pRawImg;
IplImage *pImg;
unsigned char *pData;



image_transport::Publisher image_pub;
cv_bridge::CvImage cvi;
ros::Publisher caminfo_pub;

struct sRGB{
	int r;
	int g;
	int b;
};

sRGB yuvTorgb(int Y, int U, int V)
{
	sRGB rgb;
	rgb.r = (int)(Y + 1.4075 * (V-128));
	rgb.g = (int)(Y - 0.3455 * (U-128) - 0.7169*(V-128));
	rgb.b = (int)(Y + 1.779 * (U-128));
	rgb.r =(rgb.r<0? 0: rgb.r>255? 255 : rgb.r);
	rgb.g =(rgb.g<0? 0: rgb.g>255? 255 : rgb.g);
	rgb.b =(rgb.b<0? 0: rgb.b>255? 255 : rgb.b);
	return rgb;
}

unsigned char * NV12ToRGB(unsigned char * src, unsigned char * rgb, int width, int height){
	int numOfPixel = width * height;
	int positionOfU = numOfPixel;
	int startY,step,startU,Y,U,V,index,nTmp;
	sRGB tmp;

	for(int i=0; i<height; i++){
		startY = i*width;
		step = i/2*width;
		startU = positionOfU + step;
		for(int j = 0; j < width; j++){
			Y = startY + j;
			if(j%2 == 0)
				nTmp = j;
			else
				nTmp = j - 1;
			U = startU + nTmp;
			V = U + 1;
			index = Y*3;
			tmp = yuvTorgb((int)src[Y], (int)src[U], (int)src[V]);
			rgb[index+0] = (char)tmp.b;
			rgb[index+1] = (char)tmp.g;
			rgb[index+2] = (char)tmp.r;
		}
	}
	return rgb;
}

bool YUV420_To_BGR24(unsigned char *puc_y, unsigned char *puc_u, unsigned char *puc_v, unsigned char *puc_rgb, int width_y, int height_y)
{
	if (!puc_y || !puc_u || !puc_v || !puc_rgb)
	{
		return false;
	}
	int baseSize = width_y * height_y;
	int rgbSize = baseSize * 3;

	BYTE* rgbData = new BYTE[rgbSize];
	memset(rgbData, 0, rgbSize);

	int temp = 0;

	BYTE* rData = rgbData; 
	BYTE* gData = rgbData + baseSize;
	BYTE* bData = gData + baseSize;

	int uvIndex =0, yIndex =0;


	for(int y=0; y < height_y; y++)
	{
		for(int x=0; x < width_y; x++)
		{
			uvIndex = (y>>1) * (width_y>>1) + (x>>1);
			yIndex = y * width_y + x;

			temp = (int)(puc_y[yIndex] + (puc_v[uvIndex] - 128) * 1.4022);
			rData[yIndex] = temp<0 ? 0 : (temp > 255 ? 255 : temp);

			temp = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * (-0.3456) +
					(puc_v[uvIndex] - 128) * (-0.7145));
			gData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);

			temp = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * 1.771);
			bData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);
		}
	}

	int widthStep = width_y*3;
	for (int y = 0; y < height_y; y++)
	{
		for (int x = 0; x < width_y; x++)
		{
			puc_rgb[y * widthStep + x * 3 + 2] = rData[y * width_y + x]; //R
			puc_rgb[y * widthStep + x * 3 + 1] = gData[y * width_y + x]; //G
			puc_rgb[y * widthStep + x * 3 + 0] = bData[y * width_y + x]; //B
		}
	}

	if (!puc_rgb)
	{
		return false;
	}
	delete [] rgbData;
	return true;
}

IplImage* YUV420_To_IplImage(unsigned char* pYUV420, int width, int height)
{
	if (!pYUV420)
	{
		return NULL;
	}

	int baseSize = width*height;
	int imgSize = baseSize*3;
	BYTE* pRGB24 = new BYTE[imgSize];
	memset(pRGB24, 0, imgSize);

	int temp = 0;

	BYTE* yData = pYUV420; 
	BYTE* uData = pYUV420 + baseSize; 
	BYTE* vData = uData + (baseSize>>2); 

	if(YUV420_To_BGR24(yData, uData, vData, pRGB24, width, height) == false || !pRGB24)
	{
		return NULL;
	}

	IplImage *image = cvCreateImage(cvSize(width, height), 8,3);
	memcpy(image->imageData, pRGB24, imgSize);

	if (!image)
	{
		return NULL;
	}

	delete [] pRGB24;
	return image;
}

/*
static void *get_images_loop(void *data)
{
    int ret;

    while(!manifold_cam_exit()) 
    {
        if(mode & GETBUFFER_MODE)
        {
#if BLOCK_MODE
            ret = manifold_cam_read(buffer, &nframe, CAM_BLOCK); 
            printf("%d %d %d\n", buffer[0], buffer[1], buffer[2]);
            if(ret < 0)
            {
                printf("manifold_cam_read error \n");
                break;
            }

#else
            ret = manifold_cam_read(buffer, &nframe, CAM_NON_BLOCK); 
            if(ret < 0)
            {
                printf("manifold_cam_read error \n");
                break;
            }
                printf("rgb successfully\n");
				NV12ToRGB(buffer,pData,1280,720);
                printf("nv12torgb successfully\n");
				memcpy(pRawImg->imageData,pData,FRAME_SIZE);

            ros::Time time=ros::Time::now();

	sensor_msgs::Image im;
	sensor_msgs::CameraInfo cam_info;

	cam_info.header.frame_id = "/camera";
	cam_info.height = IMAGE_H/2;
	cam_info.width = IMAGE_W/2;
	cam_info.distortion_model = "";
	cam_info.D.push_back(-0.1297646493949856);
	cam_info.D.push_back(0.0946885697670611);
	cam_info.D.push_back(-0.0002935002712265514);
	cam_info.D.push_back(-0.00022663675362156343);
	cam_info.D.push_back(0.0);
	cam_info.K = {388.40923066779754, 0.0, 318.06257844065226, 0.0, 518.1538449374815, 241.17339016626644, 0.0, 0.0, 1.0};
	cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	cam_info.P = {373.5429992675781, 0.0, 317.51131336952494, 0.0, 0.0, 504.4360656738281, 240.6131009245937, 0.0, 0.0, 0.0, 1.0, 0.0};
	cam_info.binning_x = 0;
	cam_info.binning_x = 0;

	cam_info.roi.x_offset = 0;
	cam_info.roi.y_offset = 0;
	cam_info.roi.height = 0;
	cam_info.roi.width = 0;
	cam_info.roi.do_rectify = false;

			cvi.header.stamp = time;
			cvi.header.frame_id = "image";

				cvi.encoding = "bgr8";

			cvi.image = pImg;
			cvi.toImageMsg(im);
			cam_info.header.seq = nCount;
			cam_info.header.stamp = time;
			caminfo_pub.publish(cam_info);
			image_pub.publish(im);

#endif
        }

        usleep(1000);
    }

    printf("get_images_loop thread exit! \n");

}
*/

int main(int argc, char **argv)
{
    ros::init(argc,argv,"image_raw");

        pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,3);
		pImg = cvCreateImage(cvSize(640, 480),IPL_DEPTH_8U,3);
		pData  = new unsigned char[1280 * 720 * 3];

	int ret,nKey;
	int nState = 1;
	int nCount = 1;

    int mode = 2;
    mode |= TRANSFER_MODE;
	ret = manifold_cam_init(mode);
	if(ret == -1)
	{
		printf("manifold init error \n");
		return -1;
	}

/*
    ros::NodeHandle nh_private("~");
    ros::NodeHandle node;

	image_transport::ImageTransport transport(node);


	image_pub = transport.advertise("dji_sdk/image_raw", 1);
	caminfo_pub = node.advertise<sensor_msgs::CameraInfo>("dji_sdk/camera_info",1);

	ros::Time time=ros::Time::now();

	

	pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,3);
	pImg = cvCreateImage(cvSize(640, 480),IPL_DEPTH_8U,3);
	pData  = new unsigned char[1280 * 720 * 3];

    int ret;
    pthread_attr_t attr;
    struct sched_param schedparam;
    pthread_t read_thread;

    mode = 2;
    printf("mode: %d\n", mode);
    ret = manifold_cam_init(mode);
    if(-1 == ret)
    {
        printf("manifold init error \n");
        return -1;
    }
*/


    ros::NodeHandle node;
	image_transport::ImageTransport transport(node);
	image_transport::Publisher image_pub = transport.advertise("dji_sdk/image_raw", 1);
	ros::Publisher caminfo_pub = node.advertise<sensor_msgs::CameraInfo>("dji_sdk/camera_info",1);
    ros::Publisher imgarray_pub = node.advertise<std_msgs::ByteMultiArray>("image_array", 100);
	ros::Time time=ros::Time::now();

	cv_bridge::CvImage cvi;


	sensor_msgs::Image im;
	sensor_msgs::CameraInfo cam_info;

	cam_info.header.frame_id = "/camera";
	cam_info.height = IMAGE_H/2;
	cam_info.width = IMAGE_W/2;
	cam_info.distortion_model = "";
	cam_info.D.push_back(-0.1297646493949856);
	cam_info.D.push_back(0.0946885697670611);
	cam_info.D.push_back(-0.0002935002712265514);
	cam_info.D.push_back(-0.00022663675362156343);
	cam_info.D.push_back(0.0);
	cam_info.K = {388.40923066779754, 0.0, 318.06257844065226, 0.0, 518.1538449374815, 241.17339016626644, 0.0, 0.0, 1.0};
	cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	cam_info.P = {373.5429992675781, 0.0, 317.51131336952494, 0.0, 0.0, 504.4360656738281, 240.6131009245937, 0.0, 0.0, 0.0, 1.0, 0.0};
	cam_info.binning_x = 0;
	cam_info.binning_x = 0;

	cam_info.roi.x_offset = 0;
	cam_info.roi.y_offset = 0;
	cam_info.roi.height = 0;
	cam_info.roi.width = 0;
	cam_info.roi.do_rectify = false;

   



while(1)
	{
		ret = manifold_cam_read(buffer, &nframe, 1);

		if(ret != -1)
		{

				NV12ToRGB(buffer,pData,1280,720);
				memcpy(pRawImg->imageData,pData,FRAME_SIZE);

			cvResize(pRawImg,pImg,CV_INTER_LINEAR);
/*
            cvNamedWindow( "Image", CV_WINDOW_AUTOSIZE );
            cvShowImage("Image", pImg);
            cvWaitKey(0);
*/
            std_msgs::ByteMultiArray array;
		    array.data.clear();
            signed char* imgdata = (signed char*)(pImg->imageData);
  		    std::vector<signed char> vec(imgdata, imgdata+FRAME_SIZE);
            
			array.data = vec;
		
		    imgarray_pub.publish(array);

			time=ros::Time::now();
			//cvi.header.stamp = time;
			//cvi.header.frame_id = "image";
			//cvi.encoding = "bgr8";

            //sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pImg).toImageMsg();
    
			//cvi.image = pImg;
			//cvi.toImageMsg(im);
			cam_info.header.seq = nCount;
			cam_info.header.stamp = time;
			caminfo_pub.publish(cam_info);
			//image_pub.publish(img_msg);

			ros::spinOnce();
			nCount++;


		}
		else 
			break;

		usleep(1000);
	}
	while(!manifold_cam_exit())
	{
		sleep(1);
	}

    /*
     * if the cpu usage is high, the scheduling policy of the read thread
     * is recommended setting to FIFO, and also, the priority of the thread should be high enough.
     */

    /*
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy((pthread_attr_t *)&attr, SCHED_FIFO);
    schedparam.sched_priority = 90;
    pthread_attr_setschedparam(&attr,&schedparam);
    pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);

    if (pthread_create(&read_thread, &attr, get_images_loop, NULL) != 0)
    {
        perror ("usbRead_thread create");
        assert(0);
    }

    if(pthread_attr_destroy(&attr) != 0)
    {
        perror("pthread_attr_destroy error");
    }

    pthread_join(read_thread, NULL);

    while (!manifold_cam_exit())
    {
        sleep(1);
    }
    */
    return 0;
}
