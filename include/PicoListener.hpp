#pragma once
#include "header.h"
#include <royale.hpp>
#include <mutex>

using namespace royale;
using namespace std;
using namespace cv;

class PicoListener : public IDepthDataListener
{
    public:
    PicoListener():undistortImage()
    {

    }

    void onNewData (const DepthData *data)
    {
        // this callback function will be called for every new
        // depth frame
        std::lock_guard<std::mutex> lock (flagMutex);

        // create two images which will be filled afterwards
        // each image containing one 32Bit channel
        zImage.create (Size (data->width, data->height), CV_32FC1);
        grayImage.create (Size (data->width, data->height), CV_32FC1);

        // set the image to zero
        zImage = Scalar::all (0);
        grayImage = Scalar::all (0);

        int k = 0;
        for (int y = 0; y < zImage.rows; y++)
        {
            float *zRowPtr = zImage.ptr<float> (y);
            float *grayRowPtr = grayImage.ptr<float> (y);
            for (int x = 0; x < zImage.cols; x++, k++)
            {
                auto curPoint = data->points.at (k);
                if (curPoint.depthConfidence > 0)
                {
                    // if the point is valid, map the pixel from 3D world
                    // coordinates to a 2D plane (this will distort the image)
                    //zRowPtr[x] = adjustZValue (curPoint.z);
                    zRowPtr[x] = curPoint.z;
                    grayRowPtr[x] = adjustGrayValue (curPoint.grayValue);
                }
            }
        }

        zImage8.create (Size (data->width, data->height), CV_8UC1);
        grayImage8.create (Size (data->width, data->height), CV_8UC1);

        zImage.convertTo (zImage8, CV_8UC1);
        grayImage.convertTo (grayImage8, CV_8UC1);

        if (undistortImage)
        {
            cv::Mat temp = zImage8.clone();
            cv::Mat tempz = zImage.clone();
            undistort (temp, zImage8, cameraMatrix, distortionCoefficients);            
            undistort (tempz, zImage, cameraMatrix, distortionCoefficients);            
        }

        scaledZImage.create (Size (data->width * 4, data->height * 4), CV_8UC1);
        resize (zImage8, scaledZImage, scaledZImage.size());

        if (undistortImage)
        {
            cv::Mat temp = grayImage8.clone();
            undistort (temp, grayImage8, cameraMatrix, distortionCoefficients);
        }

        // scale and display the gray image
        scaledGrayImage.create (Size (data->width * 4, data->height * 4), CV_8UC1);
        resize (grayImage8, scaledGrayImage, scaledGrayImage.size());
    }

    void setLensParameters (const LensParameters &lensParameters)
    {
        // Construct the camera matrix
        // (fx   0    cx)
        // (0    fy   cy)
        // (0    0    1 )
        cameraMatrix = (Mat1d (3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
                        0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
                        0, 0, 1);

        camera_param.fx = lensParameters.focalLength.first;
        camera_param.fy = lensParameters.focalLength.second;
        camera_param.cx = lensParameters.principalPoint.first;
        camera_param.cy = lensParameters.principalPoint.second;


        // Construct the distortion coefficients
        // k1 k2 p1 p2 k3
        distortionCoefficients = (Mat1d (1, 5) << lensParameters.distortionRadial[0],
                                  lensParameters.distortionRadial[1],
                                  lensParameters.distortionTangential.first,
                                  lensParameters.distortionTangential.second,
                                  lensParameters.distortionRadial[2]);
    }

    void toggleUndistort()
    {
        std::lock_guard<std::mutex> lock (flagMutex);
        undistortImage = !undistortImage;
    }

    cv::Mat getZMat ()
    {
        std::lock_guard<std::mutex> lock (flagMutex);
        //return scaledZImage;
        return zImage;
    }

    // adjust gray value to fit fixed scaling, here max value is 180
    // the max value here is used as an example and can be modified
    cv::Mat getGrayMat ()
    {
        std::lock_guard<std::mutex> lock (flagMutex);
        //return scaledGrayImage;
        return grayImage;
    }

    CameraParameter getCameraParameter ()
    {
        std::lock_guard<std::mutex> lock (flagMutex);
        //return scaledGrayImage;
        return camera_param;
    }

    std::mutex flagMutex;
        
private:

    // adjust z value to fit fixed scaling, here max dist is 2.5m
    // the max dist here is used as an example and can be modified
    float adjustZValue (float zValue)
    {
        float clampedDist = std::min (2.5f, zValue);
        float newZValue = clampedDist / 2.5f * 255.0f;
        return newZValue;
    }

    // adjust gray value to fit fixed scaling, here max value is 180
    // the max value here is used as an example and can be modified
    float adjustGrayValue (uint16_t grayValue)
    {
        float clampedVal = std::min (180.0f, grayValue * 1.0f);
        float newGrayValue = clampedVal / 180.f * 255.0f;
        return newGrayValue;
    }

    // define images for depth and gray
    // and for their 8Bit and scaled versions
    cv::Mat zImage, zImage8, scaledZImage;
    cv::Mat grayImage, grayImage8, scaledGrayImage;

    // lens matrices used for the undistortion of
    // the image
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;

    CameraParameter camera_param;

    bool undistortImage;
};
