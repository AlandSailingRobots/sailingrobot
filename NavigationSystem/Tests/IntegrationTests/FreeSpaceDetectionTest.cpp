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
//#include "DataBase/DBHandler.h"
//#include "MessageBus/MessageTypes.h"
//#include "MessageBus/MessageBus.h"
//#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
//#include "WorldState/CollidableMgr/CollidableMgr.h"

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

//DBHandler dbHandler("../asr.db");
//MessageBus msgBus;
//CollidableMgr cMgr;
struct Compass
{
    float roll = 0;
    float heading = 0;
    int tmsp = 0;
} m_compass_data;

//void messageLoop() {
//    msgBus.run();
//}

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

//    cMgr.startGC();

//    std::thread thr(messageLoop);
//    thr.detach();
//    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    VideoCapture m_capture(0); // Opens the camera handle
    if (m_capture.isOpened() == false) //  To check if object was associated to webcam successfully
    {
        Logger::error("Camera not available");
        // skipping return for single frame test
        //return -1;
    }

    Mat imgFullSize; // Input raw image

    // Input single frame for tuning purpose
    String filename("scene00376.png");
    imgFullSize = imread(filename, IMREAD_COLOR);
    if(imgFullSize.empty()){
        cout << "Path incorrect or filename wrong, image not read!" << endl;
    return -1;
    }
    cout << "frame loaded" << endl;

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
    //m_capture >> imgFullSize;
    Rect thermalImagerArea(lowFrameX, lowFrameY, widthFrame, heightFrame);
    imgOriginal = imgFullSize(thermalImagerArea).clone();
    Point2f center(imgOriginal.cols/2.0, imgOriginal.rows/2.0);


    cout << "debug point: before for" << endl;
    for(;;)
    {
        //m_capture >> imgFullSize; skipped single frame test

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
        //Convert image to grayscale
        cvtColor( imgOriginal, frameGrayScale, CV_BGR2GRAY );

        // Apply a Gaussian Filter to clear out general noise
        GaussianBlur( frameGrayScale, frameGrayScale, Size(5, 5), 4.0, 4.0 );

        // Remove spot noise
        medianBlur(frameGrayScale, frameGrayScale, 3);

        // Remove small objects
        Mat erodedFrame;
        erode( frameGrayScale, erodedFrame, kernel_ero );
        // Test dilatereconstruct instead of dilate
        //dilate( frameGrayScale, frameGrayScale, kernel_ero );
        frameGrayScale = dilateReconstruction(erodedFrame, frameGrayScale, kernel_ero);

        /*
         * -----------------------------------------------------------------
         * Find the horizon and set up the ROI (region of interest)
         * to the image surface beneath
         *-----------------------------------------------------------------
         */

        // Attempt with Canny or Laplacian operator
        
        Canny(frameGrayScale, dst, 16, 42, 3);
        cout << "debug step: before edge detection" << endl;
        cout << "what is this: " << CV_64F << endl;
        //Laplacian(frameGrayScale, dst, 16);
        cout << "debug step: after edge detection" << endl;
        cvtColor(dst, cdst, COLOR_GRAY2BGR);
        

        // Attempt with Threshold+DistTransform+Watershed
        /*
        // Create binary image from source image
        Mat bw = frameGrayScale.clone();
        Mat src = frameGrayScale.clone();
        threshold(bw, bw, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
        // Save intermediary result for debugging purposes
        imwrite("binImg.jpg", bw);
        // Perform the distance transform algorithm
        Mat dist;
        distanceTransform(bw, dist, CV_DIST_L2, 3);
        // Normalize the distance image for range = {0.0, 1.0}
        // so we can visualize and threshold it
        normalize(dist, dist, 0, 1., NORM_MINMAX);
        // Save intermediary result for debugging purposes
        imwrite("distTransfImg.jpg", dist);
        // Threshold to obtain the peaks
        // This will be the markers for the foreground objects
        threshold(dist, dist, .4, 1., CV_THRESH_BINARY);
        // Dilate a bit the dist image
        Mat kernel1 = Mat::ones(3, 3, CV_8UC1);
        dilate(dist, dist, kernel1);
        imwrite("peaks.jpg", dist);
        // Create the CV_8U version of the distance image
        // It is needed for findContours()
        Mat dist_8u;
        dist.convertTo(dist_8u, CV_8U);
        // Find total markers
        vector<vector<Point> > contours;
        findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        // Create the marker image for the watershed algorithm
        Mat markers = Mat::zeros(dist.size(), CV_32SC1);
        // Draw the foreground markers
        for (size_t i = 0; i < contours.size(); i++)
            drawContours(markers, contours, static_cast<int>(i), Scalar::all(static_cast<int>(i)+1), -1);
        // Draw the background marker
        circle(markers, Point(5,5), 3, CV_RGB(255,255,255), -1);
        imwrite("markers.jpg", markers*10000);

        // Perform the watershed algorithm
        cout << "reached watershed step" << endl;
        watershed(src, markers);
        Mat mark = Mat::zeros(markers.size(), CV_8UC1);
        markers.convertTo(mark, CV_8UC1);
        bitwise_not(mark, mark);
    //    imshow("Markers_v2", mark); // uncomment this if you want to see how the mark
                                      // image looks like at that point
        // Generate random colors
        vector<Vec3b> colors;
        for (size_t i = 0; i < contours.size(); i++)
        {
            int b = theRNG().uniform(0, 255);
            int g = theRNG().uniform(0, 255);
            int r = theRNG().uniform(0, 255);
            colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
        }
        // Create the result image
        Mat dst = Mat::zeros(markers.size(), CV_8UC3);
        // Fill labeled objects with random colors
        for (int i = 0; i < markers.rows; i++)
        {
            for (int j = 0; j < markers.cols; j++)
            {
                int index = markers.at<int>(i,j);
                if (index > 0 && index <= static_cast<int>(contours.size()))
                    dst.at<Vec3b>(i,j) = colors[index-1];
                else
                    dst.at<Vec3b>(i,j) = Vec3b(0,0,0);
            }
        }
        // Visualize the final image
        imwrite("watershedResult.jpg", dst);

        */







        cout << "debug step: before HoughLines" << endl;
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

        break; //single frame test
    }

    Logger::error("exited loop");

    return 0;
}
