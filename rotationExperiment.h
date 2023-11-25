#pragma once
#include "common.h"
#include <opencv2/opencv.hpp>

class CRotationExperiment
{
public:
    CRotationExperiment();
    ~CRotationExperiment();
    void setPitch(angle _pitch);
    void setYaw(angle _yaw);
    void setRoll(angle _roll);
    void setComposeOrder(rotationOrder _order);
    void setDecomposeOrder(rotationOrder _order);
    angle getPitch() const;
    angle getYaw() const;
    angle getRoll() const;
    void decomposeEuler(angle& _pitch, angle& _yaw, angle& _roll);
    cv::Mat getRotationMatrix();
    void drawProjectedImage(const cv::Mat& points3D, cv::Mat& imageProjected);

private:
    angle pitch;
    angle yaw;
    angle roll;
    enum rotationOrder orderCompose;
    enum rotationOrder orderDecompose;
};

