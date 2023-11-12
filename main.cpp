#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Function to project 3D points onto a 2D image
Mat project3DPoints(const Mat& points3D, double yaw, double pitch, double roll) {
    // Define rotation matrices for yaw, pitch, and roll
    Mat rotationMatrixYaw = (Mat_<double>(3, 3) <<
        cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1);

    Mat rotationMatrixPitch = (Mat_<double>(3, 3) <<
        cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch));

    Mat rotationMatrixRoll = (Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll));

    // Combine rotation matrices
    Mat rotationMatrix = rotationMatrixYaw * rotationMatrixPitch * rotationMatrixRoll;

    // Project 3D points onto 2D image
    Mat points2D = rotationMatrix * points3D;

    return points2D.rowRange(0, 2);
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

int main() {
    // Create a blank image
    Mat image = Mat::ones(500, 500, CV_8UC3) * Scalar(255, 255, 255);

    // Define 3D points
    Mat points3D = (Mat_<double>(3, 4) <<
        1, 1, 1,
        1, -1, 1,
        -1, -1, 1,
        -1, 1, 1);

    // Initial angles
    double yaw = 0, pitch = 0, roll = 0;

    // Create a window
    namedWindow("3D Projection");

    // Create trackbars
    createTrackbar("Yaw", "3D Projection", 0, 360, onYawTrackbar, &yaw);
    createTrackbar("Pitch", "3D Projection", 0, 360, onPitchTrackbar, &pitch);
    createTrackbar("Roll", "3D Projection", 0, 360, onRollTrackbar, &roll);

    while (true) {
        // Project 3D points onto 2D image
        Mat projectedPoints = project3DPoints(points3D, yaw, pitch, roll);

        // Draw points on the image
        image.setTo(Scalar(255, 255, 255));
        for (int i = 0; i < projectedPoints.cols; ++i) {
            Point point(static_cast<int>(projectedPoints.at<double>(0, i) * 100) + 250,
                static_cast<int>(projectedPoints.at<double>(1, i) * 100) + 250);
            circle(image, point, 5, Scalar(0, 0, 0), -1);
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
