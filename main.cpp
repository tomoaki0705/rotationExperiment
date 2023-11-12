#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Function to project 3D points onto a 2D image
Mat project3DPoints(const Mat& points3D, double yaw, double pitch, double roll) {
    // Define rotation matrices for yaw, pitch, and roll
    Mat rotationMatrixRoll = (Mat_<double>(3, 3) <<
        cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1);

    Mat rotationMatrixYaw = (Mat_<double>(3, 3) <<
        cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch));

    Mat rotationMatrixPitch = (Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll));

    // Combine rotation matrices
    Mat rotationMatrix = rotationMatrixYaw * rotationMatrixPitch * rotationMatrixRoll;

    // Project 3D points onto 2D image
    Mat result = rotationMatrix * points3D;

    return result;
}

// Callback functions for trackbars
void onYawTrackbar(int value, void* ptr) {
    double* yawPtr = static_cast<double*>(ptr);
    *yawPtr = value * CV_PI / 180.0;
}

void onPitchTrackbar(int value, void* ptr) {
    double* pitchPtr = static_cast<double*>(ptr);
    *pitchPtr = value * CV_PI / 180.0;
}

void onRollTrackbar(int value, void* ptr) {
    double* rollPtr = static_cast<double*>(ptr);
    *rollPtr = value * CV_PI / 180.0;
}

Mat projetCamera2Image(const Mat& camera_coordinates)
{
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
    cv::projectPoints(camera_coordinates, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    return image_points;
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

    // Create a window
    namedWindow("3D Projection");

    // Create trackbars
    createTrackbar("Roll", "3D Projection", 0, 360, onYawTrackbar, &yaw);
    createTrackbar("Yaw", "3D Projection", 0, 360, onPitchTrackbar, &pitch);
    createTrackbar("Pitch", "3D Projection", 0, 360, onRollTrackbar, &roll);

    while (true) {
        // Project 3D points onto 2D image
        Mat projectedPoints = project3DPoints(points3D, yaw, pitch, roll);

        Mat projectedPoints2D = projetCamera2Image(projectedPoints);

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

        // Break the loop when 'ESC' key is pressed
        if (waitKey(1) == 27)
            break;
    }

    // Destroy OpenCV window
    destroyAllWindows();

    return 0;
}
