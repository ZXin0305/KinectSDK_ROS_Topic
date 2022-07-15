#include<iostream>
#include<stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// kinect的头文件
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

using namespace std;
using namespace cv;

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "kinect_color_depth_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher color_pub = it.advertise("kinectSDK/color", 5);
    image_transport::Publisher depth_pub = it.advertise("kinectSDK/depth", 5);
    sensor_msgs::ImagePtr color_msg;
    sensor_msgs::ImagePtr depth_msg;


    std::cout << "Streaming from Kinect One sensor!" << std::endl;
    // Initialize and Discover Devices
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;    //传感器设备, 0就是个地址
    libfreenect2::PacketPipeline *pipeline = 0; //数据传输方式

    //! [discovery]
    // 检测设备，返回的是一个列表
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;   // 003415165047 -- kinect设备的序列号

    if(pipeline)
    {
        //! [open]
        // Open and Configure the Device
        dev = freenect2.openDevice(serial, pipeline);   //按照某种数据传输方式打开设备
        //! [open]
    } else {
        dev = freenect2.openDevice(serial);
        // cout<<"新的设备号码"<<dev<<endl;  //检测得到的设备重新赋予一个设备地址
    }
    // 再次检测设备是否不存在
    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }
    
    signal(SIGINT, sigint_handler);
    protonect_shutdown = false;


    //! [listeners]
    // 同步多窗口监听器, 监听对象包括 RGB/depth
    // You cannot configure the device after starting it.
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
                                                  libfreenect2::Frame::Depth |
                                                  libfreenect2::Frame::Ir);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);   //RGB
    dev->setIrAndDepthFrameListener(&listener); //Depth

    //! Start the Device
    // 开始监听
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    //! [start]

    //! [registration setup]
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
    //! [registration setup]

    cv::Mat rgbmat, depthmatUndistorted, irmat, rgbd, depth_fullscale, depth_ros;
    unsigned int i = 1;

    // Receive Image Frames
    ros::Rate rate(50);
    int key;
    while(ros::ok() || !protonect_shutdown){

        if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
        {
          std::cout << "timeout!" << std::endl;
          return -1;
        }

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        //! [loop start]

        cv::Mat rgbmat(rgb->height, rgb->width, CV_8UC4, rgb->data);  //将从libfreenect2类订阅得到的RGB图像转换到opencv
        cv::Mat depthmat(depth->height, depth->width, CV_32FC1, depth->data); //同样，深度图

        cv::flip(rgbmat, rgbmat, 1);

        //! [registration]
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        //! [registration]

        // cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        // cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(depth_fullscale);  //转换到和RGB图像一个scale,单通道
        cv::flip(depth_fullscale, depth_fullscale, 1);  // RGB 和 depth 都需要进行翻转
        depth_fullscale = depth_fullscale / 4096.0f * 255;  // 归一化之后，转换成单通道的图， 这里必须要乘255，不然CV_8UC1直接变为0
        depth_fullscale.convertTo(depth_ros, CV_8UC1);

        color_msg = cv_bridge::CvImage(std_msgs::Header(), "rgba8", rgbmat).toImageMsg();  // kinect读取得到的图像格式是BGRA8 ， 我在转换成ROS的话题信息的时候转换成RGBA8, 在python端直接接受成RGBA8就可以了
        color_pub.publish(color_msg);

        depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_ros).toImageMsg();
        depth_pub.publish(depth_msg);

        // cv::imshow("depth", depth_ros);
        // cv::waitKey(1);
	
        listener.release(frames);
        rate.sleep();
        ROS_INFO("have send the image ..");
    }

    dev->stop();
    dev->close();
    delete registration;              
    std::cout << "Streaming Ends!" << std::endl;
    return 0;
}
