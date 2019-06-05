#include "header.h"
#include "PicoSensorSetter.h"


using namespace std;

void
PicoSensorSetter::initialize(int _image_width, int _image_height, int _image_fps){
    royale::Vector<royale::String> camlist (manager.getConnectedCameraList());
    cout << "Detected " << camlist.size() << " camera(s)." << endl;

    for(int i=0;i<camlist.size();i++){
        if (!camlist.empty())
        {
            //cameraDevice = manager.createCamera (camlist[0]);
            std::unique_ptr<ICameraDevice> cameraDevice;
            cameraDevice = manager.createCamera (camlist[i]);
            camCameraDeviceVec.push_back(std::move(cameraDevice));
            royale::String _id;
            auto status = camCameraDeviceVec[i]->getId (_id);
            std::string id = _id.toStdString();
            cout << "serial : " << id << endl;
            PicoSensor _sensor;

            _sensor.initialize(id, _image_width, _image_height, _image_fps, camCameraDeviceVec[i], listener);
            pico_sensor_list.push_back(_sensor);
            bm_idx2serial.insert(bimap_value_t(i, _id.toStdString()));
            n_sensor = camlist.size();    
        }
    }
}

void
PicoSensorSetter::setSensorObject(std::vector<SensorWrapper> &_sensor_vec){
    SensorWrapper _sensor_wrapper;
    for(int i=0;i<n_sensor;i++){
        _sensor_wrapper._get_rgb_image_func = [=](cv::Mat &_rgb){
            _rgb = pico_sensor_list[i].getRGBImage(listener);
        };
        _sensor_wrapper._get_depth_image_func = [=](cv::Mat &_depth){
            _depth = pico_sensor_list[i].getDepthImage(listener);
        };
        _sensor_wrapper._get_colorized_depth_image_func = [=](cv::Mat &_colorized_depth){
            _colorized_depth = pico_sensor_list[i].getColorizedDepthImage(listener);
        };        
        _sensor_wrapper._get_ir_image_func = [=](cv::Mat &_ir){
            _ir = pico_sensor_list[i].getIRImage(listener);
        };
        _sensor_wrapper._get_camera_parameter_func = [=](CameraParameter &_camera_parameter){
            _camera_parameter = pico_sensor_list[i].getCameraParameter();
        };        
        _sensor_wrapper._start_func = [=](){
            pico_sensor_list[i].start(camCameraDeviceVec[i]);
        };
        _sensor_wrapper._stop_func = [=](){
            pico_sensor_list[i].stop(camCameraDeviceVec[i]);
        };
        _sensor_wrapper._update_func = [=](){
            pico_sensor_list[i].update();
        };
        _sensor_vec.emplace_back(_sensor_wrapper);
    }
}

int
PicoSensorSetter::getNumSensor(){
    return n_sensor;
}

