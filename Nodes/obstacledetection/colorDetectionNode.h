/****************************************************************************************
 *
 * File:
 * 		colorDetectionNode.h
 *
 * Purpose:
 *		A colorDetectionNode which uses opencv and color detection to detect obstacles
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "colorDetectionUtility.h"
#include "../ActiveNode.h"



class colorDetectionNode : public ActiveNode {
public:
    colorDetectionNode(MessageBus& msgBus,std::vector<std::string> colors_input);//Accepted values:red,orange,green,yellow,purple and blue
    //Possibility to live change ths HSV values live after to detect another color
	colorDetectionNode(MessageBus& msgBus,int m_numberOfCapturesPerDetection,
                        int port, int delay,std::vector<std::string> colors_input);

	~colorDetectionNode();

	///----------------------------------------------------------------------------------
	/// Initialises the connection with the camera and HSV default values
	///
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
	/// Processes DataRequest messages.
	///
	///----------------------------------------------------------------------------------
	void processMessage(const Message* msgPtr);

	///----------------------------------------------------------------------------------
 	/// This function starts the  colorDetectionNode thread
 	///
 	///----------------------------------------------------------------------------------
	void start();
private:

	static void colorDetectionThreadFunc(void* nodePtr);

    void changecolorTrackbar(void* nodePtr);
    static void update_trackbar_hsv_values( int val , void* nodePtr );
    static void get_trackbar_hsv_values( int val, void* nodePtr );
    static void get_on_click_hsv_pixel_values( int event, int x, int y, int val , void* nodePtr );
    void initWindowsAndTrackbars(void* nodePtr);

    cv::Mat     m_imgOriginal;
    int     m_hsvDiff;
    int     m_iLowH;
    int     m_iHighH;
    int     m_iLowS;
    int     m_iHighS;
    int     m_iLowV;
    int     m_iHighV;
    int     m_iColor;
    cv::Mat     m_trackBarHSV;
    int     m_numberOfColorsToTrack;
    std::vector<std::vector<int> >  m_hsvValues;

	bool 	m_Initialised;
    bool 	m_inputColorError;
    int     m_minAreaToDetect;
    int     m_maxAreaToDetect;
    int     m_numberOfCapturesPerDetection;
    int     m_delay; //In ms
    int     m_port;
    std::vector<cv::Scalar>      m_colorDrawing;
    cv::VideoCapture m_cap;
};
