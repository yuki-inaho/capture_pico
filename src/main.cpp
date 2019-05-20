#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "ParameterManager.hpp"
#include "PicoZenseSensorClass.h"
#include "PicoZenseSetter.h"
#include "SensorManager.h"
#include "SensorWrapper.h"

// ros
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


using namespace std;
using namespace cv;

std::string HOME_PATH = std::getenv("HOME");
std::string CFG_PARAM_PATH = HOME_PATH + "/catkin_ws/src/capture_zense/cfg/recognition_parameter.toml";

// One (and only one) of your C++ files must define CVUI_IMPLEMENTATION
// before the inclusion of cvui.h to ensure its implementaiton is compiled.
#define CVUI_IMPLEMENTATION
#include "cvui.h"

using namespace std;


/// Depth -> Point
pcl::PointCloud<pcl::PointXYZ>::Ptr Depth2Point(cv::Mat src, CameraParameter cam_p) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int h = 0; h < src.rows; h++) {
        for (int w = 0; w < src.cols; w++) {

            unsigned short z_value_short;
            float z_value_float;
            z_value_short = src.at<short>(h, w);
//            z_value_float = src.at<float>(h, w);

            if (z_value_short > 0 || z_value_float >0.0) {
                Eigen::Vector3f v;
                v = Eigen::Vector3f::Zero();

                v.z() = (float)(z_value_short)/1000;

                if(v.z() == 0) continue;
                v.x() = v.z() * (w - cam_p.cx) * (1.0 / cam_p.fx);
                v.y() = v.z() * (h - cam_p.cy) * (1.0 / cam_p.fy);

                pcl::PointXYZ point_tmp;
                point_tmp.x = v.x();
                point_tmp.y = v.y();
                point_tmp.z = v.z();
                cloud->points.push_back(point_tmp);
            }
        }
    }

    return cloud;
}

/// 1shot
void CaptureImageMode(){

    /// cvui GUI
    string WINDOW_NAME = "capture_mode";
    ParameterManager cfg_param(CFG_PARAM_PATH);

    int window_width = cfg_param.ReadIntData("Camera", "image_width")*2;
    int window_height = cfg_param.ReadIntData("Camera", "image_height")*2;
    int image_width = cfg_param.ReadIntData("Camera", "image_width");
    int image_height = cfg_param.ReadIntData("Camera", "image_height");
    int capture_fps = cfg_param.ReadIntData("Camera", "capture_fps");

    string ZENSE_ID = cfg_param.ReadStringData("Camera", "sensor_id");

    cv::Mat frame = cv::Mat(window_height, window_width, CV_8UC3);
    cvui::init(WINDOW_NAME);

    
    PicoZenseSensorSetter pico_zense_setter;
    pico_zense_setter.initialize(image_width, image_height, capture_fps);
    
    std::vector<SensorWrapper> sensors_pico;
    pico_zense_setter.setSensorObject(sensors_pico);

    SensorManager sens_mng;  
    sens_mng.setIdxSerialMap(pico_zense_setter.bm_idx2serial);
    sens_mng.setSensors(sensors_pico);
    sens_mng.activateSensor(ZENSE_ID);
    sens_mng.start();

    /// default folder check
    int mkdir_e = mkdir("../capture_tmp/" , 0777);
    int file_count = -2; // "." ".."分をカウントから引く
    DIR* dp=opendir("../capture_tmp/");
    if (dp!=NULL)
    {
        struct dirent* dent;
        do{
            dent = readdir(dp);
            if (dent!=NULL){
//                cout<<dent->d_name<<endl;
                file_count++;
            }
        }while(dent!=NULL);
        closedir(dp);
    }

    file_count = file_count / 4; // depth, color, color_depth, pcd
    std::cout << "file count = " << file_count << std::endl;

    CameraParameter camera_param;
    camera_param = sens_mng.getCameraParameter();

    std::cout << "fx = " << camera_param.fx << std::endl;
    std::cout << "fy = " << camera_param.fy << std::endl;
    std::cout << "cx = " << camera_param.cx << std::endl;
    std::cout << "cy = " << camera_param.cy << std::endl;

    /// Capture Start
    while (true) {
        frame = cv::Scalar(49, 52, 49);
        sens_mng.update();
        
        cv::Mat color, ir_left,  depth, depth_color;
        color = sens_mng.getRGBImage().clone();
        depth = sens_mng.getDepthImage().clone(); 
        ir_left = sens_mng.getIRImage().clone();  
        
        if(depth.rows == 0) continue;
        if(ir_left.rows == 0) continue;
        depth_color = sens_mng.getColorizedDepthImage();  

        /// display resize
        cv::Mat color_r, depth_color_r, depth_r, depth_convert;
        cv::resize(color, color_r, cv::Size(image_width, image_height));
        cv::resize(depth_color, depth_color_r, cv::Size(image_width, image_height));

//        cv::cvtColor(ir_left,ir_left,cv::COLOR_GRAY2RGB);

        cvui::image(frame, 0, 0, color_r);
        cvui::image(frame, image_width, 0, depth_color_r);

        /// pcd data
        pcl::PointCloud<pcl::PointXYZ>::Ptr point(new pcl::PointCloud<pcl::PointXYZ>);

        
        point = Depth2Point(depth, camera_param);

        ///　1枚撮影(Button)
        if (cvui::button(frame, 50, window_height - 300, 300, 100,  "Capture RGB/Depth Image")) {

            std::ostringstream oss;
            oss << std::setw(4) << std::setfill('0') << file_count;
            string num_str = oss.str().c_str();

            string color_name = "../capture_tmp/color_" + num_str + ".jpg";
            string depth_name = "../capture_tmp/depth_" + num_str + ".png";
            string ir_name = "../capture_tmp/ir_" + num_str + ".png";
            string depth_color_name = "../capture_tmp/depth_color_" + num_str + ".jpg";
            string pcd_name = "../capture_tmp/point_" + num_str + ".pcd";

            cv::imwrite(color_name, color);

            Mat depth_write;
 //           depth.convertTo(depth_write, CV_8UC4, 1.0/1.0);
//            cv::Mat depth_write = Mat(depth.rows, depth.cols, CV_8UC4, depth.data);
            cv::imwrite(depth_name, depth);
            cv::imwrite(ir_name, ir_left);
            cv::imwrite(depth_color_name, depth_color);

            pcl::io::savePCDFileBinary(pcd_name, *point);
            std::cout << "count = " << file_count << " Saved !!" << std::endl;
            file_count++;

        }

        /// 撮影枚数表示
        cvui::printf(frame, 400, window_height - 200, 0.5, 0x00ff00, "image_num = %.4d", file_count);

        /// text display
        cvui::text(frame, 50, window_height - 100, "Press [ESC] key -> Exit", 0.5);

        cvui::imshow(WINDOW_NAME, frame);

        if (cv::waitKey(20) == 27) {
            break;
        }
    }

    sens_mng.stop();
}

///
void ROSBugMode(int argc, char** argv) {

    /// ROS Init
    ros::init(argc, argv, "ros_bug_mode");
    ros::NodeHandle n;

    /// CameraSetting
    string WINDOW_NAME = "capture_mode";
    ParameterManager cfg_param(CFG_PARAM_PATH);

    int window_width = cfg_param.ReadIntData("Camera", "image_width")*2;
    int window_height = cfg_param.ReadIntData("Camera", "image_height")*2;
    int image_width = cfg_param.ReadIntData("Camera", "image_width");
    int image_height = cfg_param.ReadIntData("Camera", "image_height");
    int capture_fps = cfg_param.ReadIntData("Camera", "capture_fps");

    string ZENSE_ID = cfg_param.ReadStringData("Camera", "sensor_id");

    cv::Mat frame = cv::Mat(window_height, window_width, CV_8UC3);
    cvui::init(WINDOW_NAME);

    PicoZenseSensorSetter pico_zense_setter;
    pico_zense_setter.initialize(image_width, image_height, capture_fps);

    std::vector<SensorWrapper> sensors_pico;
    pico_zense_setter.setSensorObject(sensors_pico);

    SensorManager sens_mng;
    sens_mng.setIdxSerialMap(pico_zense_setter.bm_idx2serial);
    sens_mng.setSensors(sensors_pico);
    sens_mng.activateSensor(ZENSE_ID);
    sens_mng.start();


    CameraParameter camera_param;
    camera_param = sens_mng.getCameraParameter();

    std::cout << "fx = " << camera_param.fx << std::endl;
    std::cout << "fy = " << camera_param.fy << std::endl;
    std::cout << "cx = " << camera_param.cx << std::endl;
    std::cout << "cy = " << camera_param.cy << std::endl;

    /// default folder check
    int mkdir_e = mkdir("../capture_tmp/" , 0777);
    int file_count = -2; // "." ".."分をカウントから引く
    DIR* dp=opendir("../capture_tmp/");
    if (dp!=NULL)
    {
        struct dirent* dent;
        do{
            dent = readdir(dp);
            if (dent!=NULL){
//                cout<<dent->d_name<<endl;
                file_count++;
            }
        }while(dent!=NULL);
        closedir(dp);
    }

//    file_count = file_count / 4; // depth, color, color_depth, pcd
    std::cout << "file count = " << file_count << std::endl;


    /// Capture Start
    while (true) {
        frame = cv::Scalar(49, 52, 49);
        sens_mng.update();

        cv::Mat color, ir_left,  depth, depth_color;
        color = sens_mng.getRGBImage().clone();
        depth = sens_mng.getDepthImage().clone();
        ir_left = sens_mng.getIRImage().clone();

        if(depth.rows == 0) continue;
        if(ir_left.rows == 0) continue;
        depth_color = sens_mng.getColorizedDepthImage();

        /// display resize
        cv::Mat color_r, depth_color_r, depth_r, depth_convert;
        cv::resize(color, color_r, cv::Size(image_width, image_height));
        cv::resize(depth_color, depth_color_r, cv::Size(image_width, image_height));

//        cv::cvtColor(ir_left,ir_left,cv::COLOR_GRAY2RGB);

        cvui::image(frame, 0, 0, color_r);
        cvui::image(frame, image_width, 0, depth_color_r);

        ///　BugFile撮影(Button)
        if (cvui::button(frame, 50, window_height - 300, 300, 100,  "Capture ROSBug data")) {

            std::ostringstream oss;
            oss << std::setw(4) << std::setfill('0') << file_count;
            string num_str = oss.str().c_str();

            string bag_name = "../capture_tmp/bag_" + num_str + ".bag";

            /// rosbag capture start
            rosbag::Bag bag;
            bag.open(bag_name, rosbag::bagmode::Write);

            int frame_tmp = 0;
            while (ros::ok()) {

                sens_mng.update();
                color = sens_mng.getRGBImage().clone();
                depth = sens_mng.getDepthImage().clone();
                ir_left = sens_mng.getIRImage().clone();

                if(depth.rows == 0) continue;
                if(ir_left.rows == 0) continue;
                depth_color = sens_mng.getColorizedDepthImage();

                sensor_msgs::ImagePtr color_msg, depth_msg, depth_color_msg;
                color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
                depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth).toImageMsg();
                depth_color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_color).toImageMsg();

                bag.write("color", ros::Time::now(), color_msg);
                bag.write("depth", ros::Time::now(), depth_msg);
                bag.write("depth_color", ros::Time::now(), depth_color_msg);

                /// msg->cv
//                cv_bridge::CvImagePtr cv_ptr;
//                cv_ptr  = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
//                cv::imshow("test.png", cv_ptr->image);
//                pcl::PointCloud<pcl::PointXYZ>::Ptr point1(new pcl::PointCloud<pcl::PointXYZ>);
//                point1 = Depth2Point(depth, camera_param);
//                pcl::PointCloud<pcl::PointXYZ>::Ptr point2(new pcl::PointCloud<pcl::PointXYZ>);
//                point2 = Depth2Point(cv_ptr->image, camera_param);
//                pcl::io::savePCDFileBinary("point1.pcd", *point1);
//                pcl::io::savePCDFileBinary("point2.pcd", *point2);

                std::cout << "frame = " << frame_tmp << std::endl;
                frame_tmp++;

                /// tmp window display
                cv::resize(color, color_r, cv::Size(image_width, image_height));
                cv::resize(depth_color, depth_color_r, cv::Size(image_width, image_height));
                cv::imshow("color", color_r);
                cv::imshow("depth_color", depth_color_r);

                if (cv::waitKey(20) == 27) {
                    cv::destroyWindow("color");
                    cv::destroyWindow("depth_color");
                    break;
                }
            }

            bag.close();


            std::cout << "count = " << file_count << " Saved !!" << std::endl;
            file_count++;

        }

        /// 撮影枚数表示
        cvui::printf(frame, 400, window_height - 200, 0.5, 0x00ff00, "image_num = %.4d", file_count);

        /// text display
        cvui::text(frame, 50, window_height - 100, "Press [ESC] key -> Exit", 0.5);

        cvui::imshow(WINDOW_NAME, frame);

        if (cv::waitKey(20) == 27) {
            break;
        }
    }

    sens_mng.stop();
}


///
/// main
///

int main(int argc, char** argv)
{
    std::cout << argv[1] << std::endl;

    string mode = argv[1];

    if(mode == "rosbag"){
        ROSBugMode(argc, argv);
    }
    else if(mode == "1shot"){
        CaptureImageMode();
    }
    else{
        std::cout << "command error !!" << std::endl;
    }

    return 0;
}