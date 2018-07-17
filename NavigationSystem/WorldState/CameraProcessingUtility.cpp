/****************************************************************************************
 *
 * File:
 * 		CameraProcessingUtility.cpp
 *
 * Purpose:
 *		Utility functions for image acquisition and processing.
 *      While running, this thread can be restarted with 'ESC', or
 *      shut down with 'q'.
 *
 *
 * Developer Notes:
 *      In case of lag when the program is running, pressing ESC will restart the
 *      thread and help a bit. Changing the value of "waitKey()" can do the job 
 *      if lags occur too many times.
 *    
 * TO DO: Add compass correction. Decrease computation time for the Raspberry
 *        (deleting the "imshow" functions can decrease by 0,15s, on the desk computer)
 *        Delete the includes and put it all in the corresponding .h file.
 ***************************************************************************************/

#include <vector>
#include <thread>
#include <map>
#include <unistd.h>

#include "../SystemServices/SysClock.h"
#include "../SystemServices/Timer.h"
#include "../Database/DBHandler.h"
#include "../SystemServices/Logger.h"
#include "../WorldState/CollidableMgr/CollidableMgr.h"
#include "CameraProcessingUtility.h"

#include <chrono> // testing execution speed

using namespace std;
using namespace cv;

#define WITH_GUI 1 // disable with 0
#define CAMERA_DEVICE_ID 0 // 0 = default webcam
#define DETECTOR_LOOP_TIME 250 // in ms (250 * 20 ms = 5s)
#define MAX_COMPASS_FRAME_TIMEFRAME 10 // in ms
#define NUMBER_OF_SEGMENTS 24 // make it close to an approximation of 1 degree
                              // per cols as the camera has a range from -12 to 12

struct Compass
{
    float roll = 0;
    float heading = 0;
    int tmsp = 0; // Updated nowhere in the code
} m_compass_data;

// Next values have to be selected by failings and retry, in comments are the values that should
// work for : regular webcam(output format should be documented) | 
// registered frame from the thermal camera | thermal camera video input

const int lowFrameX = 68; //0; //68 //29
const int widthFrame = 585;//640; // 585; //257
const int lowFrameY = 30;//0; //98; //30
const int heightFrame = 440; //500;//480; //381; //195

char c; // input for video display

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


Mat dilateReconstruction(Mat imgReference, Mat imgMarker, Mat kernel)
// Perform geodesic dilation (not sure of the name atm, might have to check this), 
// used for the opening by reconstruction
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


CameraProcessingUtility::CameraProcessingUtility(MessageBus& msgBus, DBHandler& dbhandler, CollidableMgr* collidableMgr)
  : ActiveNode(NodeID::CameraProcessingUtility, msgBus), m_cameraDeviceID(CAMERA_DEVICE_ID),
    m_detectorLoopTime(DETECTOR_LOOP_TIME), m_maxCompassFrameTimeframe(MAX_COMPASS_FRAME_TIMEFRAME),
    collidableMgr(collidableMgr), m_db(dbhandler), m_running(false) {

        Logger::init("FreeSpaceDetectionTest.log");
  } 

CameraProcessingUtility::~CameraProcessingUtility() {}


bool CameraProcessingUtility::init() {
    VideoCapture capture(0); // Opens the camera handle
    //VideoCapture capture("/home/sailbot/Docs/output_edit.mp4"); // For testing from video
    this->m_capture = capture;
    if (this->m_capture.isOpened() == false) //  To check if object was associated to video input successfully
    {
        Logger::error("Node: CameraProcessingUtility - Camera not available");
        // Shutdown if no camera found
        exit(EXIT_FAILURE);
    }
    Logger::info("Video capture initialized");
    return true;
}

void CameraProcessingUtility::start() {
    m_running = true;
    runThread(CameraProcessingUtilityThreadFunc);
}

void CameraProcessingUtility::stop() {
    m_running = false;
    m_capture.release();
    if (WITH_GUI)
    {
      destroyAllWindows();
    }
    stopThread(this);
}

void CameraProcessingUtility::CameraProcessingUtilityThreadFunc(ActiveNode* nodePtr) {
    CameraProcessingUtility* node = dynamic_cast<CameraProcessingUtility*> (nodePtr);

    Timer timer;
    timer.start();
    //Logger::info("Entering CameraProcessingUtilityThreadFunc loop");
    
    std::chrono::duration<double> elapsed_seconds;

   if (WITH_GUI)
    {
    namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.
    namedWindow( "Display roi", WINDOW_NORMAL );
    namedWindow( "Display distance", WINDOW_NORMAL );

    resizeWindow( "Display window", widthFrame - lowFrameX, heightFrame - lowFrameY );
    resizeWindow( "Display roi", widthFrame - lowFrameX, heightFrame - lowFrameY );
    resizeWindow( "Display distance", widthFrame - lowFrameX, heightFrame - lowFrameY );
    }


    while(node->m_running) {

      auto start = std::chrono::system_clock::now();
      node->freeSpaceProcessing();
      auto end = std::chrono::system_clock::now();
      elapsed_seconds = end-start;
      cout << "-------------------------- Image Processing Took: " << elapsed_seconds.count() << " Seconds" << endl;

      node->computeRelDistances();
      node->addCameraDataToCollidableMgr();
      
      timer.sleepUntil(0.25); // pause when thread is restarted
      timer.reset();
    }
    //Logger::info("Exiting CameraProcessingUtilityThreadFunc loop");
}

void CameraProcessingUtility::addCameraDataToCollidableMgr() {
    // float bearing = col*webcamAngleApertureXPerPixel m_compass_data.heading;
    int16_t bearing = 0; // Need to include compass in the process
    bearing = (int16_t)m_compass_data.heading;
    collidableMgr->addVisualField(m_relBearingToRelObstacleDistance, bearing);
    //Logger::info("Camera data added to CollidableMgr");
}

Mat CameraProcessingUtility::getRoi() {
    return m_freeSpaceFrame;
}

map<int16_t, uint16_t> CameraProcessingUtility::getRelDistances() {
    return m_relBearingToRelObstacleDistance;
}

void CameraProcessingUtility::freeSpaceProcessing() {
    
    this->m_capture >> m_imgFullSize;

    if (m_imgFullSize.empty())
    {
        Logger::info("Warning, frame is empty");
    }
    Mat imgOriginal; // Input raw image
    Mat hsvImg; // HSV converted image
    // Image containers
    Mat frameGrayScale, roi, dst, cdst;
    Mat erodedFrame;

    // Noise removal kernel for the filters
    // Using a vertical filter helps removing the waves during the open by reconstruction step
    // but we are assuming here that the waves are seen horizontally by the camera
    Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(2,12));

    // Save detected lines
    std::vector<Vec4i> lines;
    // output, locations of non-zero pixels
    std::vector<Point> locations;   

    // For green dot detection
    std::vector<Vec3f> circles;

    // Tilt correction
    Mat rot;
    Rect imageBox;

    // Set up frame size
    Rect thermalImagerArea(lowFrameX, lowFrameY, widthFrame, heightFrame);
    imgOriginal = m_imgFullSize(thermalImagerArea).clone();
    Point2f center(imgOriginal.cols/2.0, imgOriginal.rows/2.0);

    // Main processing part

    if (m_imgFullSize.empty()) { // if frame read unsuccessfully
        Logger::error("Video input frame not readable");
        //break;
    }

    imgOriginal = m_imgFullSize(thermalImagerArea).clone();

    if (WITH_GUI)
    {
      imshow( "Display window", imgOriginal );
    }
   

    /*
     * -----------------------------------------------------------------
     * Correct tilted image
     *-----------------------------------------------------------------
     */
    // Check if compass data is up to date (tmsp is not updated anywhere btw)
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
        //continue;
    }

    /*
     * -----------------------------------------------------------------
     * Find the horizon and set up the ROI (region of interest)
     * to the image surface beneath
     *-----------------------------------------------------------------
     */

    // Meanshift clustering, can add some filters before for denoising/tuning
    meanshiftBaseFrame = imgOriginal.clone();
    //GaussianBlur( meanshiftBaseFrame, meanshiftBaseFrame, Size(5, 5), 4.0, 4.0 );
    //medianBlur(meanshiftBaseFrame, meanshiftBaseFrame, 3);
    pyrMeanShiftFiltering( meanshiftBaseFrame, meanshiftBaseFrame, spatialRad, colorRad, maxPyrLevel );
    frameGrayScale = meanshiftBaseFrame.clone();
    cvtColor(frameGrayScale, frameGrayScale, COLOR_RGB2GRAY);

    erode( frameGrayScale, erodedFrame, kernel_ero );
    frameGrayScale = dilateReconstruction(erodedFrame, frameGrayScale, kernel_ero);

    // Using Canny filter for edges detection
    Canny(frameGrayScale, dst, 16, 42, 3);
    cvtColor(dst, cdst, COLOR_GRAY2BGR);
    
    // Detecting lines for horizon detection
    HoughLinesP(dst, lines, 2, 2*CV_PI/180, 10, 40, 25 ); //50,50,10|10,10,50
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
            //continue;
        }

        // select the greatest line only
        if (max_dist < hyp)
        {
            max_l = l;
            max_dist = hyp;
        }
    }

    // show lines found (appears blue on the image)
    line( cdst, Point(max_l[0], max_l[1]), Point(max_l[2], max_l[3]), Scalar(255,0,0), 3, LINE_AA);


    roi=dst;
    if (WITH_GUI)
    {
      imshow( "Display roi", cdst );
    }
    //imshow( "Display roi", cdst );

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

    // This image is what we want
    m_freeSpaceFrame = roi.clone();
    this->m_freeSpaceFrame = m_freeSpaceFrame; 
    //cout << to_string(this->m_freeSpaceFrame.type) << endl;
    
    if (WITH_GUI)
    {
        imshow( "Display distance", m_freeSpaceFrame );
    
        c=(char)waitKey(200); // pause of 20ms
        // Press ESC tor restart the thread, or Q to kill it.
        if(c==27) //ESC=27
        {
          Logger::info("ESC display window, restarting the thread");
          //break;
        }
        else if(c==113) //q=113
        {
          Logger::info("Warning, killing camera processing thread");
          m_running = false;
          m_capture.release();
          destroyAllWindows();
        }
        else
        {
        //waitKey(1); // process is kind of heavy depending on the current frame
                    // might need to increase the pause to smooth it a little
        }
    }
    else
    {
        usleep(100000);
    }

}

int CameraProcessingUtility::computeRelDistances() {
    int16_t n_cols = this->m_freeSpaceFrame.cols;
    int16_t n_rows = this->m_freeSpaceFrame.rows;
    collidableMgr->removeOldVisualField();

    vector<int16_t> n_whitePixelsVect(n_cols);
    for (int i=0; i < n_cols; i++)
    {
        for (int j=0; j < n_rows; j++)
        {
            // Notice that in Opencv, it's Point(col,row) but Mat(row,col)
            if (this->m_freeSpaceFrame.at<unsigned char>(j,i) > 127) // pixels are black or white so we choose an arbitrary value
                                                                     // for comparing 
            {
                n_whitePixelsVect.at(i)++;
                // might have to check there if the roi resulting from the processing
                // always keep the white pixels under the black ones, just to be sure
                // the result is coherent
            }
        }

    }
    // not really optimized to reloop over the cols, but that's a first shot

    std::lock_guard<std::mutex> guard(m_lock);
    int pixsPerSegment = n_cols/NUMBER_OF_SEGMENTS;
    uint16_t tmp_sum=0;
    for (int i=0; i<NUMBER_OF_SEGMENTS; i++)
    {
        for (int j=0; j<pixsPerSegment; j++)
        {
            tmp_sum += n_whitePixelsVect[i*pixsPerSegment+j];
        }
        // We insert the mean of the distance in a group of columns (called a segment here), scaled between 0 and 100
        // this->m_relBearingToRelObstacleDistance.insert(pair<int16_t ,uint16_t>(i-NUMBER_OF_SEGMENTS/2, 100.0*tmp_sum/(heightFrame*pixsPerSegment)));
        // Change from insert to operator[], as insert does not overwrite the value of an existing key, and operator[]
        // does call the insert function in case the key doesn't exist yet.
        this->m_relBearingToRelObstacleDistance[(i-NUMBER_OF_SEGMENTS/2)] = static_cast<unsigned short>(100.0 * tmp_sum / (heightFrame * pixsPerSegment));
        tmp_sum = 0;
    }
    return EXIT_SUCCESS;
}

void CameraProcessingUtility::processMessage(const Message* msg) 
{
    MessageType type = msg->messageType();
    switch(type)
    {
    case MessageType::CompassData:
        processCompassMessage(static_cast<const CompassDataMsg*>(msg));
        break;
    default:
        return;
    }
}

void CameraProcessingUtility::processCompassMessage(const CompassDataMsg* msg)
{
    std::lock_guard<std::mutex> lock_guard(m_lock);
    m_compass_data.heading = msg->heading();
    m_compass_data.roll = msg->roll();
}
