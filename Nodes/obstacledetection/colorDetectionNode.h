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
	colorDetectionNode(MessageBus& msgBus,int port, int delay,std::vector<std::string> colors_input);

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

    void initWindowsAndTrackbars(void);
	static void colorDetectionThreadFunc(void* nodePtr);
    static void changecolorTrackbar(void);
    static void update_trackbar_hsv_values( int, void* );
    static void get_trackbar_hsv_values( int, void* );
    static void get_on_click_hsv_pixel_values( int event, int x, int y, int, void* );

    static cv::Mat     m_imgOriginal;
    static int     m_hsvDiff;
    static int     m_iLowH;
    static int     m_iHighH;
    static int     m_iLowS;
    static int     m_iHighS;
    static int     m_iLowV;
    static int     m_iHighV;
    static int     m_iColor;
    static cv::Mat     m_trackBarHSV;
    static int     m_numberOfColorsToTrack;
    static std::vector<std::vector<int> >  m_hsvValues;

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
