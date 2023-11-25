#include <opencv2/opencv.hpp>
#include "common.h"
#include "rotationExperiment.h"

using namespace cv;
using namespace std;
double yaw_, pitch_, roll_;
Mat mtxR, mtxQ, Qx, Qy, Qz;
Vec3d decomposedEuler;

inline double deg2rad(double degree)
{
    return (degree * CV_PI) / 180.;
}

inline double rad2deg(double radian)
{
    return (radian * 180.) / CV_PI;
}

void mat2euler( Mat& rotationMatrix, double& yaw, double& pitch, double& roll, enum rotationOrder order)
{
    double r00 = rotationMatrix.at<double>(0, 0);
    double r01 = rotationMatrix.at<double>(0, 1);
    double r02 = rotationMatrix.at<double>(0, 2);
    double r10 = rotationMatrix.at<double>(1, 0);
    double r11 = rotationMatrix.at<double>(1, 1);
    double r12 = rotationMatrix.at<double>(1, 2);
    double r20 = rotationMatrix.at<double>(2, 0);
    double r21 = rotationMatrix.at<double>(2, 1);
    double r22 = rotationMatrix.at<double>(2, 2);
    switch (order)
    {
    case XYZ:
        if (r02 < 1)
        {
            if (r02 > -1)
            {
                yaw = asin(r02);
                pitch = atan2(-r12, r22);
                roll = atan2(-r01, r00);
            }
            else
            {
                yaw = -CV_PI / 2;
                pitch = -atan2(r10, r11);
                roll = 0;
            }
        }
        else // r02 = +1
        {            
            {
                yaw = CV_PI / 2;
                pitch = atan2(r10, r11);
                roll = 0;
            }
        }
        break;
    case XZY:
        if (r01 < 1)
        {
            if (r01 > -1)
            {
                roll = asin(-r01);
                pitch = atan2(r21, r11);
                yaw = atan2(r02, r00);
            }
            else
            {
                roll = -CV_PI / 2;
                pitch = -atan2(r20, r22);
                yaw = 0;
            }
        }
        else // r01 = +1
        {
            {
                roll = -CV_PI / 2;
                pitch = atan2(-r20, r22);
                yaw = 0;
            }
        }
        break;
    case YXZ:
        if (r12 < 1)
        {
            if (r12 > -1)
            {
                pitch = asin(-r12);
                yaw = atan2(r02, r22);
                roll = atan2(r10, r11);
            }
            else
            {
                pitch = CV_PI / 2;
                yaw = -atan2(-r01, r00);
                roll = 0;
            }
        }
        else
        {
            pitch = -CV_PI / 2;
            yaw = atan2(-r01, r00);
            roll = 0;
        }
        break;
    case YZX:
        if (r10 < 1)
        {
            if (r10 > -1)
            {
                roll = asin(r10);
                yaw = atan2(-r20, r00);
                pitch = atan2(-r12, r11);
            }
            else
            {
                roll = -CV_PI / 2;
                yaw = -atan2(r21, r22);
                pitch = 0;
            }
        }
        else // r01 = +1
        {
            {
                roll = -CV_PI / 2;
                yaw = atan2(r21, r22);
                pitch = 0;
            }
        }
        break;
    case ZXY:
        if (r21 < 1)
        {
            if (r21 > -1)
            {
                pitch = asin(r21);
                yaw = atan2(-r01, r11);
                roll = atan2(-r20, r22);
            }
            else
            {
                pitch = -CV_PI / 2;
                yaw = -atan2(r02, r00);
                roll = 0;
            }
        }
        else
        {
            pitch = CV_PI / 2;
            yaw = atan2(r02, r00);
            roll = 0;
        }
        break;
    case ZYX:
        if (r02 < 1)
        {
            if (r02 > -1)
            {
                yaw = asin(-r20);
                roll = atan2(r10, r00);
                pitch = atan2(r21, r22);
            }
            else
            {
                yaw = CV_PI / 2;
                roll = -atan2(-r12, r11);
                pitch = 0;
            }
        }
        else // r02 = +1
        {
            {
                yaw = -CV_PI / 2;
                roll = atan2(-r12, r11);
                pitch = 0;
            }
        }
        break;
    default:
        break;
    }
}

// Function to project 3D points onto a 2D image
Mat project3DPoints(const Mat& points3D, double yaw, double pitch, double roll, enum rotationOrder order, enum rotationOrder decompose) {
    // Define rotation matrices for yaw, pitch, and roll
    Mat rotationMatrixRoll = (Mat_<double>(3, 3) <<
        cos(roll), -sin(roll), 0,
        sin(roll), cos(roll), 0,
        0, 0, 1);

    Mat rotationMatrixYaw = (Mat_<double>(3, 3) <<
        cos(yaw), 0, sin(yaw),
        0, 1, 0,
        -sin(yaw), 0, cos(yaw));

    Mat rotationMatrixPitch = (Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(pitch), -sin(pitch),
        0, sin(pitch), cos(pitch));

    // Combine rotation matrices
    Mat rotationMatrix;
    switch (order)
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


    decomposedEuler = RQDecomp3x3(rotationMatrix, mtxR, mtxQ, Qx, Qy, Qz);
    volatile bool flag = false;
    if (flag)
    {
        cout << rotationMatrixPitch << endl;
        cout << Qx << endl;
        cout << "=========" << endl;
        cout << rotationMatrixYaw << endl;
        cout << Qy << endl;
        cout << "=========" << endl;
        cout << rotationMatrixRoll << endl;
        cout << Qz << endl;
        cout << "=========" << endl;
    }


    mat2euler(rotationMatrix, yaw_, pitch_, roll_, decompose);
    //std::cout << yaw << ',' << yaw_ << '\t' << pitch << ',' << pitch_ << '\t' << roll << ',' << roll_ << endl;

    // Intrinsic parameters
    double fx = 1000.0;  // focal length in x
    double fy = 1000.0;  // focal length in y
    double cx = 320.0;   // optical center in x
    double cy = 240.0;   // optical center in y

    // Distortion parameters (assuming no distortion)
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

    // Rotation to quaternion
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    Rodrigues(rotationMatrix, rvec);

    // Translation
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    tvec.at<double>(2) = 300;

    // Build the camera matrix
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // Project 3D point to 2D
    Mat image_points;
    cv::projectPoints(points3D, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    return image_points;
}

void putTextInternal(Mat& image, const string& result, const Point position)
{
    // Font settings
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    int thickness = 2;
    cv::Scalar textColor(0, 0, 0); // Black color

    // Paste text on the image
    cv::putText(image, result, position, fontFace, fontScale, textColor, thickness);
}

void drawDebug(Mat& image, double yaw, double pitch, double roll, double decomposedYaw, double decomposedPitch, double decomposedRoll)
{
    std::string result = cv::format("% 3.2f, % 3.2f, % 3.2f", rad2deg(pitch), rad2deg(yaw), rad2deg(roll));
    cv::Point textPosition(5, 550);
    putTextInternal(image, result, textPosition);

    result = cv::format("% 3.2f, % 3.2f, % 3.2f", rad2deg(decomposedPitch), rad2deg(decomposedYaw), rad2deg(decomposedRoll));
    textPosition = Point(5, 590);
    putTextInternal(image, result, textPosition);

    //double pitchAPI = atan2(Qx.at<double>(1, 2), Qx.at<double>(2, 2));
    //double r00 = Qy.at<double>(0, 0);
    //double r02 = Qy.at<double>(0, 2);
    //double yawAPI = atan2(r00, r02);
    //double rollAPI = atan2(Qz.at<double>(0, 0), Qz.at<double>(1, 0));
    double pitchAPI = decomposedEuler(0);
    double yawAPI = decomposedEuler(1);
    double rollAPI = decomposedEuler(2);
    result = cv::format("% 3.2f, % 3.2f, % 3.2f", pitchAPI, yawAPI, rollAPI);
    textPosition = Point(5, 630);
    putTextInternal(image, result, textPosition);
}


// Callback functions for trackbars
void onEulerTrackbar(int value, void* ptr) {
    double* eulerAnglePtr = static_cast<double*>(ptr);
    *eulerAnglePtr = deg2rad(value);
}

void setAllTrackBarPos(int pos)
{
    setTrackbarPos("Pitch (X)", "3D Projection", pos);
    setTrackbarPos("Yaw (Y)", "3D Projection", pos);
    setTrackbarPos("Roll (Z)", "3D Projection", pos);
}

int main() {
    // Create a blank image
    Mat image(640, 640, CV_8UC3, Scalar(255, 255, 255));

    // Define 3D points
    Mat points3D = (Mat_<double>(3, 10) <<
        20, -22, -22, -12, -12,  19, 19,  -12,  -12,  20,  /* X */
       -50, -50,  50,  50,  10,  10,  0,    0,  -40, -40,  /* Y */
         0,   0,   0 ,  0,   0,   0,  0,    0,    0,   0); /* Z */

    // Initial angles
    double yaw = 0, pitch = 0, roll = 0;
    rotationOrder constructOrder = rotationOrder::XYZ;
    rotationOrder decomposeOrder = rotationOrder::XYZ;

    // Create a window
    namedWindow("3D Projection");

    // Create trackbars
    createTrackbar("Pitch (X)", "3D Projection", 0, 720, onEulerTrackbar, &pitch);
    createTrackbar("Yaw (Y)", "3D Projection", 0, 720, onEulerTrackbar, &yaw);
    createTrackbar("Roll (Z)", "3D Projection", 0, 720, onEulerTrackbar, &roll);
    setAllTrackBarPos(0);

    while (true) {
        // Project 3D points onto 2D image
        Mat projectedPoints2D = project3DPoints(points3D, yaw, pitch, roll, constructOrder, decomposeOrder);

        // Draw points on the image
        image.setTo(Scalar(255, 255, 255));

        // draw debug info
        drawDebug(image, yaw, pitch, roll, yaw_, pitch_, roll_);

        Point prev_point(static_cast<int>(projectedPoints2D.at<double>(projectedPoints2D.rows-1, 0)),
            static_cast<int>(projectedPoints2D.at<double>(projectedPoints2D.rows-1, 1)));
        for (int i = 0; i < projectedPoints2D.rows; ++i) {
            Point point(static_cast<int>(projectedPoints2D.at<double>(i, 0)),
                static_cast<int>(projectedPoints2D.at<double>(i, 1)));
            circle(image, point, 5, Scalar(0, 0, 0), -1);
            line(image, prev_point, point, Scalar(0, 0, 0), 2);
            prev_point = point;
        }

        // Display the image
        imshow("3D Projection", image);

        char c = waitKey(1);
        bool exitFlag = false;
        // Break the loop when 'ESC' key is pressed
        switch (c)
        {
        case 'q':
        case 'Q':
        case 27:
            exitFlag = true;
            break;
        case 'r':
            setAllTrackBarPos(0);
            break;
        case '1':
            constructOrder = rotationOrder::XYZ;
            break;
        case '2':
            constructOrder = rotationOrder::XZY;
            break;
        case '3':
            constructOrder = rotationOrder::YXZ;
            break;
        case '4':
            constructOrder = rotationOrder::YZX;
            break;
        case '5':
            constructOrder = rotationOrder::ZXY;
            break;
        case '6':
            constructOrder = rotationOrder::ZYX;
            break;
        case '!':
            decomposeOrder = rotationOrder::XYZ;
            break;
        case '@':
            decomposeOrder = rotationOrder::XZY;
            break;
        case '#':
            decomposeOrder = rotationOrder::YXZ;
            break;
        case '$':
            decomposeOrder = rotationOrder::YZX;
            break;
        case '%':
            decomposeOrder = rotationOrder::ZXY;
            break;
        case '^':
            decomposeOrder = rotationOrder::ZYX;
            break;
        default:
            break;
        }
        if (exitFlag)
            break;
    }

    // Destroy OpenCV window
    destroyAllWindows();

    return 0;
}
