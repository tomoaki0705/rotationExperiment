#include "rotationExperiment.h"
using namespace cv;
const uint64_t kAngleMinusZero = 0x8000000000000000;
const Scalar colorBlack = Scalar(0, 0, 0);
const Scalar colorWhite = Scalar(255, 255, 255);

inline bool compareHex(angle value, uint64_t hexLiteral) {
    uint64_t angleValueAsHex;
    std::memcpy(&angleValueAsHex, &value, sizeof(angle));

    return angleValueAsHex == hexLiteral;
}

CRotationExperiment::CRotationExperiment()
    : pitch(0.)
    , yaw(0.)
    , roll(0.)
    , orderCompose(rotationOrder::XYZ)
    , orderDecompose(rotationOrder::XYZ)
{
}

CRotationExperiment::~CRotationExperiment()
{
}

void CRotationExperiment::setPitch(angle _pitch)
{
    pitch = _pitch;
}

void CRotationExperiment::setYaw(angle _yaw)
{
    yaw = _yaw;
}

void CRotationExperiment::setRoll(angle _roll)
{
    roll = _roll;
}

angle CRotationExperiment::getPitch() const
{
    return pitch;
}

angle CRotationExperiment::getYaw() const
{
    return yaw;
}

angle CRotationExperiment::getRoll() const
{
    return roll;
}

void CRotationExperiment::decomposeEuler(angle& _pitch, angle& _yaw, angle& _roll)
{
    Mat rotationMatrix = getRotationMatrix();
    angle r00 = rotationMatrix.at<angle>(0, 0);
    angle r01 = rotationMatrix.at<angle>(0, 1);
    angle r02 = rotationMatrix.at<angle>(0, 2);
    angle r10 = rotationMatrix.at<angle>(1, 0);
    angle r11 = rotationMatrix.at<angle>(1, 1);
    angle r12 = rotationMatrix.at<angle>(1, 2);
    angle r20 = rotationMatrix.at<angle>(2, 0);
    angle r21 = rotationMatrix.at<angle>(2, 1);
    angle r22 = rotationMatrix.at<angle>(2, 2);
    switch (orderDecompose)
    {
    default:
    case XYZ:
        if (r02 < 1)
        {
            if (r02 > -1)
            {
                _yaw = asin(r02);
                _pitch = atan2(-r12, r22);
                _roll = atan2(-r01, r00);
            }
            else
            {
                _yaw = -CV_PI / 2;
                _pitch = -atan2(r10, r11);
                _roll = 0;
            }
        }
        else // r02 = +1
        {
            _yaw = CV_PI / 2;
            _pitch = atan2(r10, r11);
            _roll = 0;
        }
        break;
    case XZY:
        if (r01 < 1)
        {
            if (r01 > -1)
            {
                _roll = asin(-r01);
                _pitch = atan2(r21, r11);
                _yaw = atan2(r02, r00);
            }
            else
            {
                _roll = -CV_PI / 2;
                _pitch = -atan2(r20, r22);
                _yaw = 0;
            }
        }
        else // r01 = +1
        {
            _roll = -CV_PI / 2;
            _pitch = atan2(-r20, r22);
            _yaw = 0;
        }
        break;
    case YXZ:
        if (r12 < 1)
        {
            if (r12 > -1)
            {
                _pitch = asin(-r12);
                _yaw = atan2(r02, r22);
                _roll = atan2(r10, r11);
            }
            else
            {
                _pitch = CV_PI / 2;
                _yaw = -atan2(-r01, r00);
                _roll = 0;
            }
        }
        else // r12 = +1
        {
            _pitch = -CV_PI / 2;
            _yaw = atan2(-r01, r00);
            _roll = 0;
        }
        break;
    case YZX:
        if (r10 < 1)
        {
            if (r10 > -1)
            {
                _roll = asin(r10);
                _yaw = atan2(-r20, r00);
                _pitch = atan2(-r12, r11);
            }
            else
            {
                _roll = -CV_PI / 2;
                _yaw = -atan2(r21, r22);
                _pitch = 0;
            }
        }
        else // r10 = +1
        {
            _roll = -CV_PI / 2;
            _yaw = atan2(r21, r22);
            _pitch = 0;
        }
        break;
    case ZXY:
        if (r21 < 1)
        {
            if (r21 > -1)
            {
                _pitch = asin(r21);
                _yaw = atan2(-r01, r11);
                _roll = atan2(-r20, r22);
            }
            else
            {
                _pitch = -CV_PI / 2;
                _yaw = -atan2(r02, r00);
                _roll = 0;
            }
        }
        else // r21 = +1
        {
            _pitch = CV_PI / 2;
            _yaw = atan2(r02, r00);
            _roll = 0;
        }
        break;
    case ZYX:
        if (r20 < 1)
        {
            if (r20 > -1)
            {
                _yaw = asin(-r20);
                _roll = atan2(r10, r00);
                _pitch = atan2(r21, r22);
            }
            else
            {
                _yaw = CV_PI / 2;
                _roll = -atan2(-r12, r11);
                _pitch = 0;
            }
        }
        else // r02 = +1
        {
            _yaw = -CV_PI / 2;
            _roll = atan2(-r12, r11);
            _pitch = 0;
        }
        break;
    }
    _yaw = compareHex(_yaw, kAngleMinusZero) ? 0. : _yaw;
    _pitch = compareHex(_pitch, kAngleMinusZero) ? 0. : _pitch;
    _roll = compareHex(_roll, kAngleMinusZero) ? 0. : _roll;
}

cv::Mat CRotationExperiment::getRotationMatrix()
{
    // Define rotation matrices for yaw, pitch, and roll
    Mat rotationMatrixRoll = (Mat_<angle>(3, 3) <<
        cos(roll), -sin(roll), 0,
        sin(roll), cos(roll), 0,
        0, 0, 1);

    Mat rotationMatrixYaw = (Mat_<angle>(3, 3) <<
        cos(yaw), 0, sin(yaw),
        0, 1, 0,
        -sin(yaw), 0, cos(yaw));

    Mat rotationMatrixPitch = (Mat_<angle>(3, 3) <<
        1, 0, 0,
        0, cos(pitch), -sin(pitch),
        0, sin(pitch), cos(pitch));

    // Combine rotation matrices
    Mat rotationMatrix;
    switch (orderCompose)
    {
    case XYZ:
    default:
        rotationMatrix = rotationMatrixPitch * rotationMatrixYaw * rotationMatrixRoll;
        break;
    case XZY:
        rotationMatrix = rotationMatrixPitch * rotationMatrixRoll * rotationMatrixYaw;
        break;
    case YXZ:
        rotationMatrix = rotationMatrixYaw * rotationMatrixPitch * rotationMatrixRoll;
        break;
    case YZX:
        rotationMatrix = rotationMatrixYaw * rotationMatrixRoll * rotationMatrixPitch;
        break;
    case ZXY:
        rotationMatrix = rotationMatrixRoll * rotationMatrixPitch * rotationMatrixYaw;
        break;
    case ZYX:
        rotationMatrix = rotationMatrixRoll * rotationMatrixYaw * rotationMatrixPitch;
        break;
    }
    return rotationMatrix;
}

void CRotationExperiment::drawProjectedImage(const cv::Mat& points3D, cv::Mat& imageProjected)
{
    Mat rotationMatrix = getRotationMatrix();

    // Intrinsic parameters
    intrinsic fx = 1000.0;  // focal length in x
    intrinsic fy = 1000.0;  // focal length in y
    intrinsic cx = 320.0;   // optical center in x
    intrinsic cy = 240.0;   // optical center in y

    // Distortion parameters (assuming no distortion)
    Mat dist_coeffs = Mat::zeros(4, 1, CV_64F);

    // Rotation to quaternion
    Mat rvec = Mat::zeros(3, 1, CV_64F);
    Rodrigues(rotationMatrix, rvec);

    // Translation
    Mat tvec = Mat::zeros(3, 1, CV_64F);
    tvec.at<coordinate>(2) = 300;

    // Build the camera matrix
    Mat camera_matrix = (Mat_<intrinsic>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // Project 3D point to 2D
    Mat image_points;
    projectPoints(points3D, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    // Draw points on the image
    imageProjected.setTo(colorWhite);

    Point prev_point(
        static_cast<int>(image_points.at<coordinate>(image_points.rows - 1, 0)),
        static_cast<int>(image_points.at<coordinate>(image_points.rows - 1, 1)));
    for (int i = 0; i < image_points.rows; ++i) {
        Point point(
            static_cast<int>(image_points.at<coordinate>(i, 0)),
            static_cast<int>(image_points.at<coordinate>(i, 1)));
        circle(imageProjected, point, 5, colorBlack, -1);
        line(imageProjected, prev_point, point, colorBlack, 2);
        prev_point = point;
    }
}

rotationOrder CRotationExperiment::getOrderCompose() const
{
    return orderCompose;
}

rotationOrder CRotationExperiment::getOrderDecompose() const
{
    return orderDecompose;
}

void CRotationExperiment::setComposeOrder(rotationOrder _order)
{
    orderCompose = _order;
}

void CRotationExperiment::setDecomposeOrder(rotationOrder _order)
{
    orderDecompose = _order;
}
