#include "ros/ros.h"//ros
#include "std_msgs/String.h"
#include "hps_camera/camera.h"//srv
#include "../include/api.h"//api interface
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

HPS3D_HandleTypeDef handle;
AsyncIObserver_t My_Observer;
ObstacleConfigTypedef ObstacleConf;
ros::Publisher points_pub; //Global variable, because the observer callback function needs to be used
ros::Publisher depth_pub;
ros::Publisher depthinfo_pub;
bool device_usb_ = true;
std::string device_usb_port_;
std::string device_ethernet_port_;

//The observer callback function
void *User_Func(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event) {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	sensor_msgs::PointCloud2 output;
	uint16_t distance[MAX_PIX_NUM] = { 0 };
	if (event->AsyncEvent == ISubject_Event_DataRecvd) {
		switch (event->RetPacketType) {
		case SIMPLE_ROI_PACKET:
			break;
		case FULL_ROI_PACKET:
			break;
		case FULL_DEPTH_PACKET: /*point cloud data and depth data*/
			if (ros::ok()) {
				cloud.width = event->MeasureData.point_cloud_data->width;
				cloud.height = event->MeasureData.point_cloud_data->height;
				cloud.points.resize(cloud.width * cloud.height);
				for (size_t i = 0; i < cloud.points.size(); i++) {
					if (event->MeasureData.point_cloud_data[0].point_data[i].z
							< LOW_AMPLITUDE) {
						cloud.points[i].x =
								event->MeasureData.point_cloud_data[0].point_data[i].x
										/ 1000.0;
						cloud.points[i].y =
								event->MeasureData.point_cloud_data[0].point_data[i].y
										/ 1000.0;
						cloud.points[i].z =
								event->MeasureData.point_cloud_data[0].point_data[i].z
										/ 1000.0;
						//cloud.points[i].x  = i%cloud.width - cloud.width/2;
						//cloud.points[i].y = i/cloud.height - cloud.height/2;
						//cloud.points[i].z = event->MeasureData.full_depth_data->distance[i]/1000.0;
					} else {
						cloud.points[i].x = 0;
						cloud.points[i].y = 0;
						cloud.points[i].z = 0;
					}

				}
				//Convert the cloud to ROS message
				pcl::toROSMsg(cloud, output);
				output.header.frame_id = "hps";
				points_pub.publish(output);

				//get depth image
				sensor_msgs::Image outputImage;
				outputImage.header.stamp = ros::Time::now();
				outputImage.header.frame_id = "hps";
				outputImage.height = RES_HEIGHT;
				outputImage.width = RES_WIDTH;
				outputImage.encoding = "16UC1";
				outputImage.step = RES_WIDTH;
				int numberOfPixels = RES_HEIGHT * RES_WIDTH;
				int sizeOfOutputDataArray = numberOfPixels * 2;
				outputImage.data.resize(sizeOfOutputDataArray);
				int indexInputDataArray = 0;
				for (int i = 0; i < sizeOfOutputDataArray; i += 2) {
					// Extract the lower byte.
					outputImage.data[i] =
							(uint8_t) event->MeasureData.full_depth_data->distance[indexInputDataArray];
					// Extract the upper byte.
					outputImage.data[i + 1] =
							(uint8_t) (event->MeasureData.full_depth_data->distance[indexInputDataArray]
									>> 8);
					indexInputDataArray += 1;
				}
				depth_pub.publish(outputImage);

				//publish camera_info
				sensor_msgs::CameraInfo cam_info;
		        cam_info.header.stamp = outputImage.header.stamp;
		        cam_info.header.seq = outputImage.header.seq;
		        cam_info.height = outputImage.height;
		        cam_info.width = outputImage.width;
		        cam_info.K[0] = 102.39535500730614;
		        cam_info.K[1] = 0.0;
		        cam_info.K[2] = 80.5;
		        cam_info.K[3] = 0.0;
		        cam_info.K[4] = 102.39535500730614;
		        cam_info.K[5] = 30.5;
		        cam_info.K[6] = 0.0;
		        cam_info.K[7] = 0.0;
		        cam_info.K[8] = 1.0;

		        cam_info.R[0] = 1.0;
		        cam_info.R[1] = 0.0;
		        cam_info.R[2] = 0.0;
		        cam_info.R[3] = 0.0;
		        cam_info.R[4] = 1.0;
		        cam_info.R[5] = 0.0;
		        cam_info.R[6] = 0.0;
		        cam_info.R[7] = 0.0;
		        cam_info.R[8] = 1.0;

		        cam_info.P[0] = 102.39535500730614;
		        cam_info.P[1] = 0;
		        cam_info.P[2] = 80.5;
		        cam_info.P[3] = 0;
		        cam_info.P[4] = 0;
		        cam_info.P[5] = 102.39535500730614;
		        cam_info.P[6] =  30.5;
		        cam_info.P[7] = 0;
		        cam_info.P[8] = 0;
		        cam_info.P[9] = 0;
		        cam_info.P[10] = 1;
		        cam_info.P[11] = 0;

		        depthinfo_pub.publish(cam_info);

			}
			break;
		case SIMPLE_DEPTH_PACKET:
			break;
		case OBSTACLE_PACKET:
			break;
		case NULL_PACKET:
			printf("null packet\n");
			break;
		default:
			printf("system error!\n");
			break;
		}
	}
}

//check ctrl+c signal
void signal_handler(int signo) {
	if (HPS3D_RemoveDevice(&handle) != RET_OK) {
		printf("HPS3D_RemoveDevice faild\n");
	} else {
		printf("HPS3D_RemoveDevice succeed\n");
	}
	exit(0);
}

//printf log callback function
void my_printf(char *str) {
	std::cout << str;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ros_camera_client");						//ros init
	//ros::NodeHandle n ;							//Create a node
	ros::NodeHandle n("~");
	uint32_t a = 0;
	char fileName[10][20];
	uint32_t dev_cnt = 0;
	uint32_t indx = 0;
	RET_StatusTypeDef ret = RET_OK;
	AsyncIObserver_t My_Observer;

	std::stringstream sclient_name;

	//Install the signal
	if (signal(SIGINT, signal_handler) == SIG_ERR
			|| signal(SIGTSTP, signal_handler) == SIG_ERR) {
		printf("sigint error");
	}

	//Create a topic
	points_pub = n.advertise<sensor_msgs::PointCloud2>("points", 1);
	depth_pub = n.advertise<sensor_msgs::Image>("depth",1);
	depthinfo_pub = n.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

	//set debug enable and install printf log callback function
	HPS3D_SetDebugEnable(false);
	HPS3D_SetDebugFunc(&my_printf);
	//HPS3D_SetMeasurePacketType(ROI_DATA_PACKET);

	//get rosparam
	std::string device_connection_type;
	n.param<std::string>("deviceConnectionType",device_connection_type, "usb");
	n.param<std::string>("deviceUsbPort",device_usb_port_, "/dev/ttyACM0");
	n.param<std::string>("deviceEthernetPort",device_ethernet_port_, "192.168.0.10");

	printf("device_connection_type = %s\n",device_connection_type.c_str());
	if("usb" == device_connection_type){
		device_usb_ = true;
		printf("device_port = %s\n",device_usb_port_.c_str());
	}else{
		device_usb_ = false;
		printf("device_port = %s\n",device_ethernet_port_.c_str());
	}

	if (false == device_usb_) {
		char* name_tmp = const_cast<char*>(device_ethernet_port_.c_str());
		HPS3D_SetEthernetServerInfo(&handle, name_tmp, 12345);
	} else {
		char* name_tmp = const_cast<char*>(device_usb_port_.c_str());
		handle.DeviceName = name_tmp;
	}
//	int b = 0;
//	printf("select Transport type: 0:USB 1:Ethernet\n");
//	scanf("%d", &b);
//	if (b == 1) {
//		HPS3D_SetEthernetServerInfo(&handle, (char *) "192.168.0.10", 12345);
//	} else {
//		dev_cnt = HPS3D_GetDeviceList((char *) "/dev/", (char *) "ttyACM",
//				fileName);
//		handle.DeviceName = fileName[0];
//	}
	do {
		//Device Connection
		ret = HPS3D_Connect(&handle);
		if (ret != RET_OK) {
			printf("Device open failed,ret = %d\n", ret);
			break;
		}

		//Device init
		ret = HPS3D_ConfigInit(&handle);
		if (RET_OK != ret) {
			printf("Initialization failed:%d\n", ret);
			break;
		}
		printf("Initialization succeed\n");
		HPS3D_SetEdgeDetectionEnable(true);
		/*set convert point cloud data enable*/
		HPS3D_SetOpticalEnable(&handle, true);
		HPS3D_SetPointCloudEn(true);

		//Add observer one
		My_Observer.AsyncEvent = ISubject_Event_DataRecvd;
		My_Observer.NotifyEnable = true;
		HPS3D_AddObserver(&User_Func, &handle, &My_Observer);

		//Set running mode
		handle.RunMode = RUN_CONTINUOUS;
		HPS3D_SetRunMode(&handle);

	} while (0);

	if (ret != RET_OK) {
		//Remove device and disconnect
		HPS3D_RemoveDevice(&handle);
		printf("Initialization failed, Remove device\n");
		return 0;
	}

	while (1) {
		sleep(10);
	}

	return 0;
}

