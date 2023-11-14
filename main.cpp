#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

enum rotationOrder {
    XYZ,
    XZY,
    YXZ,
    YZX,
    ZXY,
    ZYX
};

// Function to project 3D points onto a 2D image
Mat project3DPoints(const Mat& points3D, double yaw, double pitch, double roll, enum rotationOrder order) {
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


    Mat mtxR, mtxQ, Qx, Qy, Qz;
    RQDecomp3x3(rotationMatrix, mtxR, mtxQ, Qx, Qy, Qz);

    // Rotate 3D points
    Mat rotated = rotationMatrix * points3D;

    // Intrinsic parameters
    double fx = 1000.0;  // focal length in x
    double fy = 1000.0;  // focal length in y
    double cx = 320.0;   // optical center in x
    double cy = 240.0;   // optical center in y

    // Distortion parameters (assuming no distortion)
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

    // Rotation and translation (assumed to be identity for this example)
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    tvec.at<double>(2) = 300;

    // Build the camera matrix
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // Project 3D point to 2D
    Mat image_points;
    cv::projectPoints(rotated, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    return image_points;
}

// Callback functions for trackbars
void onEulerTrackbar(int value, void* ptr) {
    double* eulerAnglePtr = static_cast<double*>(ptr);
    *eulerAnglePtr = value * CV_PI / 180.0;
}

void setAllTrackBarPos(int pos)
{
    setTrackbarPos("Pitch (X)", "3D Projection", pos);
    setTrackbarPos("Yaw (Y)", "3D Projection", pos);
    setTrackbarPos("Roll (Z)", "3D Projection", pos);
}

int main() {
    // Create a blank image
    Mat image(480, 640, CV_8UC3, Scalar(255, 255, 255));

    // Define 3D points
    Mat points3D = (Mat_<double>(3, 10) <<
        20, -22, -22, -12, -12,  19, 19,  -12,  -12,  20,  /* X */
       -50, -50,  50,  50,  10,  10,  0,    0,  -40, -40,  /* Y */
         0,   0,   0 ,  0,   0,   0,  0,    0,    0,   0); /* Z */

    // Initial angles
    double yaw = 0, pitch = 0, roll = 0;
    rotationOrder order = rotationOrder::XYZ;

    // Create a window
    namedWindow("3D Projection");

    // Create trackbars
    createTrackbar("Pitch (X)", "3D Projection", 0, 720, onEulerTrackbar, &pitch);
    createTrackbar("Yaw (Y)", "3D Projection", 0, 720, onEulerTrackbar, &yaw);
    createTrackbar("Roll (Z)", "3D Projection", 0, 720, onEulerTrackbar, &roll);
    setAllTrackBarPos(360);

    while (true) {
        // Project 3D points onto 2D image
        Mat projectedPoints2D = project3DPoints(points3D, yaw, pitch, roll, order); 

        // Draw points on the image
        image.setTo(Scalar(255, 255, 255));
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
            setAllTrackBarPos(360);
            break;
        case '1':
            order = rotationOrder::XYZ;
            break;
        case '2':
            order = rotationOrder::XZY;
            break;
        case '3':
            order = rotationOrder::YXZ;
            break;
        case '4':
            order = rotationOrder::YZX;
            break;
        case '5':
            order = rotationOrder::ZXY;
            break;
        case '6':
            order = rotationOrder::ZYX;
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
