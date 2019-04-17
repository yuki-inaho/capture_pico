#include "PicoSensorClass.h"

using namespace std;
using namespace royale;

void
PicoSensor::initialize(std::string _serial_number, int _image_width, int _image_height, int _image_fps, std::unique_ptr<ICameraDevice>&& cameraDevice, PicoListener &listener){
    // Pico Flexxの始動処理
    cout << "Trying to open : " << _serial_number << endl;

    // the camera device is now available and CameraManager can be deallocated here
    if (cameraDevice == nullptr)
    {
        // no cameraDevice available
        cerr << "Could not open pico flexx" << endl;
        return;
    }

    // IMPORTANT: call the initialize method before working with the camera device
    auto status = cameraDevice->initialize();
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device, error string : " << getErrorString (status) << endl;
        return;
    }
    // retrieve the lens parameters from Royale

    LensParameters lensParameters;
    status = cameraDevice->getLensParameters (lensParameters);
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Can't read out the lens parameters" << endl;
        return;
    }

    listener.setLensParameters (lensParameters);
    // register a data listener
    if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener" << endl;
        return;
    }

    
    // exposure time is available between 100-2000
    if (cameraDevice->setExposureTime(750) != CameraStatus::SUCCESS) //
    {
        cerr << "Error setting exposure time" << endl;
        return;
    }

/*
    cout << "test" << endl;
    // exposure time is available between 100-2000
    if (cameraDevice->setFrameRate(5) != CameraStatus::SUCCESS) //
    {
        cerr << "Error setting fps" << endl;
        return;
    }
    cout << "test" << endl;
*/
    // register a data listener
    if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener" << endl;
        return;
    }

    name = "Pico Flexx";
    image_width = _image_width;
    image_height = _image_height;

    serial_number = _serial_number;
    type = SensorType::TOF;

    camera_param.fx = lensParameters.focalLength.first;
    camera_param.fy = lensParameters.focalLength.second;
    camera_param.cx = lensParameters.principalPoint.first;
    camera_param.cy = lensParameters.principalPoint.second;
}

void
PicoSensor::update(){

}

void
PicoSensor::start(std::unique_ptr<ICameraDevice>&& cameraDevice){
    if (cameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing" << endl;
        return ;
    }
}

void
PicoSensor::stop(std::unique_ptr<ICameraDevice>&& cameraDevice){
    // stop capture mode
    if (cameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error stopping the capturing" << endl;
        return;
    }
}

cv::Mat
PicoSensor::getRGBImage(PicoListener &listener){
    cv::Mat _RGB_null;
    return _RGB_null;
}

cv::Mat
PicoSensor::getDepthImage(PicoListener &listener){
    cv::Mat depth;
    depth = listener.getZMat().clone();
    return depth;
}

cv::Mat
PicoSensor::getColorizedDepthImage(PicoListener &listener){
    cv::Mat _depth, _depth_color ;
    _depth = listener.getZMat().clone();

    for(int y=0;y< _depth.rows;y++){
      for(int x=0;x< _depth.cols;x++){
        if(_depth.at<float>(y,x)>2.00)
            _depth.at<float>(y,x) = 0;
      }
    }

    _depth.convertTo(_depth_color, CV_8UC1, 255.0 / 2.0 );

    cv::Mat depth_color = cv::Mat::zeros( image_height,
                                image_width,
                                CV_8UC3);


    for(int y=0;y< depth_color.rows;y++){
        for(int x=0;x< depth_color.cols;x++){
            if(_depth_color.at<uchar>(y,x) == 0){
                depth_color.at<Vec3b>(y,x)[0] = _depth_color.at<uchar>(y,x);
                depth_color.at<Vec3b>(y,x)[1] = 0;
                depth_color.at<Vec3b>(y,x)[2] = 0;
            }else{
                depth_color.at<Vec3b>(y,x)[0] = _depth_color.at<uchar>(y,x);
                depth_color.at<Vec3b>(y,x)[1] = 255;
                depth_color.at<Vec3b>(y,x)[2] = 255;
            }
        }
    }

    cvtColor(depth_color, depth_color, COLOR_HSV2BGR);
    cv::Mat depth_color_out;
    depth_color_out = depth_color.clone();

    return depth_color_out;
}

cv::Mat
PicoSensor::getIRImage(PicoListener &listener){
    cv::Mat gray;
    gray = listener.getGrayMat().clone();
    return gray;
}

CameraParameter
PicoSensor::getCameraParameter(){
    return camera_param;
}

