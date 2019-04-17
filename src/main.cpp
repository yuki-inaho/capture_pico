#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <librealsense2/rs.hpp>

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "ParameterManager.hpp"
#include "PicoSensorClass.h"
#include "PicoSensorSetter.h"
#include "PicoListener.hpp"
#include "SensorManager.h"
#include "SensorWrapper.h"


std::string HOME_PATH = std::getenv("HOME");
std::string CFG_PARAM_PATH = HOME_PATH + "/work/cpp/capture_pico/cfg/recognition_parameter.toml";


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

            unsigned short z_value = src.at<cv::uint16_t>(h, w);

            if (z_value > 0) {
                Eigen::Vector3f v;
                v = Eigen::Vector3f::Zero();

                v.z() = z_value / 1000.0f;
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

    int window_width = cfg_param.ReadIntData("Camera", "image_width")*4;
    int window_height = cfg_param.ReadIntData("Camera", "image_height")*4;
    int image_width = cfg_param.ReadIntData("Camera", "image_width");
    int image_height = cfg_param.ReadIntData("Camera", "image_height");
        
    int capture_fps = cfg_param.ReadIntData("Camera", "capture_fps");


    string PICO_ID = cfg_param.ReadStringData("Camera", "sensor_id");

    cv::Mat frame = cv::Mat(window_height, window_width, CV_8UC3);
    cvui::init(WINDOW_NAME);

    PicoSensorSetter pico_setter;
    pico_setter.initialize(image_width, image_height, capture_fps);
    std::vector<SensorWrapper> sensors_pico;
    pico_setter.setSensorObject(sensors_pico);

    SensorManager sens_mng;  
    sens_mng.setIdxSerialMap(pico_setter.bm_idx2serial);
    sens_mng.setSensors(sensors_pico);
    sens_mng.activateSensor(PICO_ID);    
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
        frame = cv::Scalar(30, 33, 30);

        cv::Mat color, ir_left,  depth, depth_color;
        color = sens_mng.getRGBImage();
        depth = sens_mng.getDepthImage(); 
        ir_left = sens_mng.getIRImage();  
        
        if(depth.rows == 0) continue;
        if(ir_left.rows == 0) continue;
        depth_color = sens_mng.getColorizedDepthImage();  

        /// display resize
        cv::Mat color_r, depth_color_r, depth_r, depth_convert;
        cv::cvtColor(ir_left,ir_left,cv::COLOR_GRAY2RGB);
        cv::resize(ir_left, color_r,cv::Size(image_width*2,image_height*2));
        cv::resize(depth_color, depth_color_r,cv::Size(image_width*2,image_height*2));

        cvui::image(frame, 0, 0, color_r);
        cvui::image(frame, image_width*2 , 0, depth_color_r);
        /// pcd data
        pcl::PointCloud<pcl::PointXYZ>::Ptr point(new pcl::PointCloud<pcl::PointXYZ>);
        point = Depth2Point(depth, camera_param);

        ///　1枚撮影(Button)
        if (cvui::button(frame, 50, int(2.4*image_height), 200, 100,  "Capture RGB/Depth Image")) {

            std::ostringstream oss;
            oss << std::setw(4) << std::setfill('0') << file_count;
            string num_str = oss.str().c_str();

            string color_name = "../capture_tmp/color_" + num_str + ".jpg";
            string depth_name = "../capture_tmp/depth_" + num_str + ".png";
            string depth_color_name = "../capture_tmp/depth_color_" + num_str + ".jpg";
            string pcd_name = "../capture_tmp/point_" + num_str + ".pcd";

            cv::imwrite(color_name, ir_left);
            cv::imwrite(depth_name, depth);
            cv::imwrite(depth_color_name, depth_color);
            pcl::io::savePCDFileBinary(pcd_name, *point);

            std::cout << "count = " << file_count << " Saved !!" << std::endl;
            file_count++;

        }

        /// 撮影枚数表示
        cvui::printf(frame, 400, int(2.4*image_height), 0.5, 0x00ff00, "image_num = %.4d", file_count);

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

    CaptureImageMode();
    return 0;
}