#pragma once
#include "header.h"
#include "PicoZense_api.h"

class PicoZenseSensor{
public:

    struct PicoZenseOption{
        bool RGB_Map_Flag;
        bool Depth_Map_Flag;
    };

    void initialize(int32_t deviceIndex,  int _image_width, int _image_height, int _image_fps, PicoZenseOption option);
    void update(int32_t deviceIndex);
    void start();//dummy
    void stop(int32_t deviceIndex);
    cv::Mat getRGBImage();
    cv::Mat getDepthImage();
    cv::Mat getIRImage();
    cv::Mat getColorizedDepthImage();
    CameraParameter getCameraParameter(int32_t deviceIndex);

    std::string serial_number;
    std::string name;

private:

    cv::Mat color_img, depth_img, ir_img, colorized_depth_img;
    CameraParameter camera_param;

    int image_width;
    int image_height;
    SensorType type;

};

