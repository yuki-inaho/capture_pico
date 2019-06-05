#pragma once
#include "header.h"
#include "PicoListener.hpp"

class PicoSensor{
    public:
        void initialize(std::string _serial_number, int _image_width, int _image_height, int _image_fps, std::unique_ptr<ICameraDevice>& _cameraDevice, PicoListener &listener);
        void start(std::unique_ptr<ICameraDevice>& cameraDevice);
        void update();
        cv::Mat getRGBImage(PicoListener &listener);
        cv::Mat getDepthImage(PicoListener &listener);
        cv::Mat getIRImage(PicoListener &listener);
        cv::Mat getColorizedDepthImage(PicoListener &listener);
        CameraParameter getCameraParameter();        
        void stop(std::unique_ptr<ICameraDevice>& cameraDevice);

    private:
        cv::Mat color_img, depth_img;
        CameraParameter camera_param;
        std::string serial_number;
        std::string name;
        
        int image_width;
        int image_height;
        SensorType type;

};