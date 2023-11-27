#include <opencv2/opencv.hpp>
#include "common.h"
#include "rotationExperiment.h"

using namespace cv;
using namespace std;

// constant data
const char kWindowName[] = "3D Projection";
const char trackBarX[] = "Pitch (X)";
const char trackBarY[] = "Yaw (Y)";
const char trackBarZ[] = "Roll (Z)";
const char kFormat[] = "%s % 3.2f, % 3.2f, % 3.2f";
const int kSize = 640;
const Size kWindowSize = Size(kSize, kSize);
Mat points3D = (Mat_<coordinate>(3, 10) << /* F character */
    20,  -22, -22, -12, -12,  19,  19, -12,  -12,  20,  /* X */
    -50, -50,  50,  50,  10,  10,  0,    0,  -40, -40,  /* Y */
    0,     0,   0,   0,   0,   0,  0,    0,   0,    0); /* Z */

inline double deg2rad(double degree)
{
    return (degree * CV_PI) / 180.;
}

inline double rad2deg(double radian)
{
    return (radian * 180.) / CV_PI;
}

void putTextInternal(Mat& image, const string& result, const Point position)
{
    // Font settings
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    int thickness = 2;
    cv::Scalar kBlackFont(0, 0, 0); // Black color

    // Paste text on the image
    cv::putText(image, result, position, fontFace, fontScale, kBlackFont, thickness);
}

void drawDebug(Mat& image, CRotationExperiment& rotationEx)
{
    // Input angle
    string result = format(
        kFormat,
        order2String[rotationEx.getOrderCompose()],
        rad2deg(rotationEx.getPitch()),
        rad2deg(rotationEx.getYaw()), 
        rad2deg(rotationEx.getRoll()));
    Point textPosition(5, image.rows - 90);
    putTextInternal(image, result, textPosition);

    // Decompose matrix to Euler
    angle decomposePitch = 0.;
    angle decomposeYaw = 0.;
    angle decomposeRoll = 0.;
    rotationEx.decomposeEuler(decomposePitch, decomposeYaw, decomposeRoll);
    result = format(
        kFormat,
        order2String[rotationEx.getOrderDecompose()],
        rad2deg(decomposePitch), 
        rad2deg(decomposeYaw), 
        rad2deg(decomposeRoll));
    textPosition = Point(5, image.rows - 50);
    putTextInternal(image, result, textPosition);

    // Use RQDecomomp3x3
    Mat mtxR, mtxQ, Qx, Qy, Qz;
    Vec3d decomposedEuler = RQDecomp3x3(
        rotationEx.getRotationMatrix(), 
        mtxR, mtxQ, Qx, Qy, Qz);
    angle pitchAPI = decomposedEuler(0);
    angle yawAPI = decomposedEuler(1);
    angle rollAPI = decomposedEuler(2);
    result = format(kFormat, "API", pitchAPI, yawAPI, rollAPI);
    textPosition = Point(5, image.rows - 10);
    putTextInternal(image, result, textPosition);
}


// Callback functions for trackbars
void onEulerTrackbar(int value, void* ptr) {
    double* eulerAnglePtr = static_cast<double*>(ptr);
    *eulerAnglePtr = deg2rad(value);
}

void setAllTrackBarPos(int pos) {
    setTrackbarPos(trackBarX, kWindowName, pos);
    setTrackbarPos(trackBarY, kWindowName, pos);
    setTrackbarPos(trackBarZ, kWindowName, pos);
}

int main() {
    // Create a blank image
    Mat image(kWindowSize.height, kWindowSize.width, CV_8UC3, Scalar(255, 255, 255));

    // Initial angles
    double yaw = 0, pitch = 0, roll = 0;

    // Create a window
    namedWindow(kWindowName);

    // Create trackbars
    // X <-> Pitch
    // Y <-> Yaw
    // Z <-> Roll
    createTrackbar(trackBarX, kWindowName, 0, 360, onEulerTrackbar, &pitch);
    createTrackbar(trackBarY, kWindowName, 0, 360, onEulerTrackbar, &yaw);
    createTrackbar(trackBarZ, kWindowName, 0, 360, onEulerTrackbar, &roll);
    setAllTrackBarPos(0);
    CRotationExperiment rotationEx;
    rotationEx.setComposeOrder(rotationOrder::XYZ);
    rotationEx.setDecomposeOrder(rotationOrder::XYZ);
    bool debugFlag = false;

    while (true) {
        rotationEx.setPitch(pitch);
        rotationEx.setYaw(yaw);
        rotationEx.setRoll(roll);

        // Project 3D points onto 2D image
        rotationEx.drawProjectedImage(points3D, image);

        if (debugFlag)
        {
            // draw debug info
            drawDebug(image, rotationEx);
        }

        // Display the image
        imshow(kWindowName, image);

        char c = waitKey(1);
        bool exitFlag = false;

        switch (c)
        {
        case ' ':
            debugFlag = debugFlag ^ true;
            break;
        case 'q':
        case 'Q':
        case 27:
            exitFlag = true;
            break;
        case 'r':
            setAllTrackBarPos(0);
            break;
        case '1':
            rotationEx.setComposeOrder(rotationOrder::XYZ);
            break;
        case '2':
            rotationEx.setComposeOrder(rotationOrder::XZY);
            break;
        case '3':
            rotationEx.setComposeOrder(rotationOrder::YXZ);
            break;
        case '4':
            rotationEx.setComposeOrder(rotationOrder::YZX);
            break;
        case '5':
            rotationEx.setComposeOrder(rotationOrder::ZXY);
            break;
        case '6':
            rotationEx.setComposeOrder(rotationOrder::ZYX);
            break;
        case SHIFT_1:
            rotationEx.setDecomposeOrder(rotationOrder::XYZ);
            break;
        case SHIFT_2:
            rotationEx.setDecomposeOrder(rotationOrder::XZY);
            break;
        case SHIFT_3:
            rotationEx.setDecomposeOrder(rotationOrder::YXZ);
            break;
        case SHIFT_4:
            rotationEx.setDecomposeOrder(rotationOrder::YZX);
            break;
        case SHIFT_5:
            rotationEx.setDecomposeOrder(rotationOrder::ZXY);
            break;
        case SHIFT_6:
            rotationEx.setDecomposeOrder(rotationOrder::ZYX);
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
