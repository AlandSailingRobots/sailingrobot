/****************************************************************************************
 *
 * File:
 * 		FreeSpaceDetectionTest.cpp
 *
 * Purpose:
 *		Integration test for obstacle detection/free space detection
 *      with the camera.
 *
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp> 
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/video.hpp>

#include <vector>
#include <thread>

#include "SystemServices/SysClock.h"
#include "SystemServices/Timer.h"
#include "SystemServices/Logger.h"

using namespace std;
using namespace cv;

#define CAMERA_DEVICE_ID 0 // 0 = default webcam
#define DETECTOR_LOOP_TIME 250 // in ms (250 * 20 ms = 5s)
#define MAX_COMPASS_FRAME_TIMEFRAME 10 // in ms

// these are retrieved by trial and error, to get the roi containing the thermal imager data
const int lowFrameX = 68; //29
const int widthFrame = 585; //257
const int lowFrameY = 98; //30
const int heightFrame = 381;  //195

/*********************************************
 * Mean shift parameters
 *********************************************
 */
// With params at (10,25,2) we keep the horizon line, boat detection, clear waves
// and maintain an acceptable processing speed. More details can be kept by changing
// the values but the best way to do it increase greatly the processing time. 
// Note that boat tracks and reflection on the water are sometimes detected.
int spatialRad = 12; // Influence details on edge, and slow down the processing
                     // dramatically if set > 12
int colorRad = 25;   // Mainly influence the details on edge, at 10 some waves are kept
                     // and at 25 there is no waves, but less edges on the horizon.
int maxPyrLevel = 2; 
Mat meanshiftBaseFrame;

struct Compass
{
    float roll = 0;
    float heading = 0;
    int tmsp = 0;
} m_compass_data;


Mat dilateReconstruction(Mat imgReference, Mat imgMarker, Mat kernel)
{
    Mat imgRec, imgDil, imgResult;
    imgRec = imgMarker.clone();   
    while (cv::countNonZero(imgRec != imgResult) != 0)  // if the two matrices are equal, we end the loop
    {
        imgResult = imgRec.clone();
        dilate(imgResult, imgDil, kernel);
        imgRec = min(imgDil, imgReference);

    }
    return imgResult;
}

int main()
{
    Logger::init("FreeSpaceDetectionTest.log");


    VideoCapture m_capture(0); // Opens the camera handle
    if (m_capture.isOpened() == false) //  To check if object was associated to webcam successfully
    {
        Logger::error("Camera not available");
        // skipping return for single frame test
        return -1;
    }

    Mat imgFullSize; // Input raw image

    Mat imgOriginal; // Input raw image
    Mat hsvImg; // HSV converted image
    // Image containers
    Mat frameGrayScale, roi, dst, cdst;

    // Noise removal kernel for the filters
    Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(2,12));

    // Save detected lines
    std::vector<Vec4i> lines;
    std::vector<Point> locations;   // output, locations of non-zero pixels

    // For green dot detection
    std::vector<Vec3f> circles;

    // Tilt correction
    Mat rot;
    Rect imageBox;

    // Set up frame size
    // skipped for single frame test
    m_capture >> imgFullSize;
    Rect thermalImagerArea(lowFrameX, lowFrameY, widthFrame, heightFrame);
    imgOriginal = imgFullSize(thermalImagerArea).clone();
    Point2f center(imgOriginal.cols/2.0, imgOriginal.rows/2.0);


    for(;;)
    {
        m_capture >> imgFullSize; 

        if (imgFullSize.empty()) { // if frame read unsuccessfully
            Logger::error("video input frame not readable");
            break;
        }
        imgOriginal = imgFullSize(thermalImagerArea).clone();

        /*
         * -----------------------------------------------------------------
         * Correct tilted image
         *-----------------------------------------------------------------
         */
        // Check if compass data is up to date
        if(SysClock::unixTime() - m_compass_data.tmsp < MAX_COMPASS_FRAME_TIMEFRAME)
        {
            // Create the rotation matrix
            rot = getRotationMatrix2D(center, m_compass_data.roll, 1.0);
            // Determine bounding rectangle
            imageBox = RotatedRect(center, imgOriginal.size(), m_compass_data.roll).boundingRect();
            // Adjust transformation matrix
            rot.at<double>(0,2) += imageBox.width/2.0 - center.x;
            rot.at<double>(1,2) += imageBox.height/2.0 - center.y;
            // Perform rotation
            warpAffine(imgOriginal, imgOriginal, rot, imageBox.size());
        }

        /*
         * -----------------------------------------------------------------
         * Convert image to HSV color space
         *-----------------------------------------------------------------
         */
        // Convert Original Image to HSV Thresh Image
        cvtColor(imgOriginal, hsvImg, CV_BGR2HSV);
        // Save temporary result for debugging purpose
        imwrite("imageHSV.jpg", hsvImg);

        /*
         * -----------------------------------------------------------------
         * Find green circle indicating camera recalibration
         *-----------------------------------------------------------------
         */
        // Find green pixels
        inRange(hsvImg, Scalar(45, 100, 100), Scalar(75, 255, 255), hsvImg);

        // Avoid false positives with a general blur
        GaussianBlur(hsvImg, hsvImg, Size(9, 9), 2, 2);

        // Find circles
        HoughCircles(hsvImg, circles, CV_HOUGH_GRADIENT, 1, hsvImg.rows/8, 85, 15, 0, 0);

        if(circles.size() != 0)
        {
            // A green circle has been detected, he thermal camera is recalibrating and frames may be unreliable and corrupted, sleep thread
            Logger::info("Camera is recalibrating, thread will sleep for 3 seconds");
            std::this_thread::sleep_for(3s);
            continue;
        }

        /*
         * -----------------------------------------------------------------
         * Denoising
         *-----------------------------------------------------------------
         */
        /** @todo Tune noise elimination filters parameters */

        /*//Convert image to grayscale
        cvtColor( imgOriginal, frameGrayScale, CV_BGR2GRAY );

        // Apply a Gaussian Filter to clear out general noise
        GaussianBlur( frameGrayScale, frameGrayScale, Size(5, 5), 4.0, 4.0 );

        // Remove spot noise
        medianBlur(frameGrayScale, frameGrayScale, 3);
        */
        pyrMeanShiftFiltering( imgOriginal, frameGrayScale, spatialRad, colorRad, maxPyrLevel );

        cvtColor(frameGrayScale, frameGrayScale, COLOR_RGB2GRAY); 

        // Remove small objects
        Mat erodedFrame;
        erode( frameGrayScale, erodedFrame, kernel_ero );
        frameGrayScale = dilateReconstruction(erodedFrame, frameGrayScale, kernel_ero);

        /*
         * -----------------------------------------------------------------
         * Find the horizon and set up the ROI (region of interest)
         * to the image surface beneath
         *-----------------------------------------------------------------
         */

        // Attempt with Canny or Laplacian operator
        
        Canny(frameGrayScale, dst, 16, 42, 3);

        cvtColor(dst, cdst, COLOR_GRAY2BGR);
        

        HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
        double theta1, theta2, hyp;

        // Horizon
        Vec4i max_l;
        double max_dist = -1.0;

        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            theta1 = (l[3]-l[1]);
            theta2 = (l[2]-l[0]);
            hyp = hypot(theta1,theta2);

            Point p1, p2;
            p1=Point(l[0], l[1]);
            p2=Point(l[2], l[3]);

            //calculate angle in degrees
            float angle = atan2(p1.y - p2.y, p1.x - p2.x)*180/CV_PI;

            // +/- 45 deg max inclination and min 1/3 of the frame width size (tilt correction may not always work)
            if(angle < 135 || angle > 225 || hyp < widthFrame/3.0)
            {
                // Skip this line
                continue;
            }

            // select the greatest line only
            if (max_dist < hyp)
            {
                max_l = l;
                max_dist = hyp;
            }
        }

        // show lines found
        line( cdst, Point(max_l[0], max_l[1]), Point(max_l[2], max_l[3]), Scalar(255,0,0), 3, LINE_AA);
        // Save intermediary result for debugging purposes
        imwrite("findLinesImg.jpg", cdst);

        /*
         * -----------------------------------------------------------------
         * Define ROI (Region of Interest)
         *-----------------------------------------------------------------
         */
        Rect rect(Point(0, min(max_l[1], max_l[3])), Point(widthFrame,heightFrame));

        if( rect.area() > 0 )
        {
            roi = dst(rect);
        }
        // if the horizon was not found
        else
        {
            roi = dst;
        }
        // Save intermediary result for debugging purposes
        imwrite("roiImg1.jpg", roi);

        /*
         * -----------------------------------------------------------------
         * Colour the frame in B/W according to lines detected
         *-----------------------------------------------------------------
         */
        for (int j = roi.cols-1; j>=0; j--)
        {
            bool white = true;
            for (int i = roi.rows-1; i>=0; i--)
            {
                if (roi.at<unsigned char>(i,j) > 0)
                    white = false;

                if(white)
                    roi.at<unsigned char>(i,j)=255;
                else
                    roi.at<unsigned char>(i,j)=0;
            }
        }
        // Save intermediary result for debugging purposes
        imwrite("sideFillImg.jpg", roi);

        /*
         * -----------------------------------------------------------------
         * Colour the frame in B/W according to lines detected
         *-----------------------------------------------------------------
         */
        for (int col = roi.cols-1; col>=0; col--)
        {
            int row = roi.rows-1;
            do{
                row--;
            }while(roi.at<unsigned char>(row,col) != 255);

            // float bearing = col*webcamAngleApertureXPerPixel m_compass_data.heading;

            // collidableMgr->addVisualObstacle(row, bearing);
        }

        imwrite("roiImg2.jpg", roi);

    }

    Logger::error("exited loop");

    return 0;
}
