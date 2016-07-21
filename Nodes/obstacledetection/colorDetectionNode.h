/****************************************************************************************
 *
 * File:
 * 		colorDetectionNode.h
 *
 * Purpose:
 *		A colorDetetectionNode which uses opencv and color detection to detect obstacles
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "../ActiveNode.h"
#include "colorDetetectionUtility.h"


class colorDetetectionNode : public ActiveNode {
public:
    colorDetetectionNode(MessageBus& msgBus);
	colorDetetectionNode(MessageBus& msgBus,int port, int delay);

	~colorDetetectionNode();

	///----------------------------------------------------------------------------------
	/// Initialises the connection with the camera
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

	bool 	m_Initialised;

    Mat     m_imgOriginal;
    int     m_hsvDiff = 10;
    int     m_iLowH = 0;
    int     m_iHighH = 179;
    int     m_iLowS = 0;
    int     m_iHighS = 255;
    int     m_iLowV = 0;
    int     m_iHighV = 255;
    int     m_iColor = 0;
    Mat     m_trackBarHSV = Mat3b(100, 300, Vec3b(0,0,0));
    int     m_ minAreaToDetect = 2000;
    int     m_ maxAreaToDetect = 20000;
    int     m_numberOfCapturesPerDetection=5;
    int     m_delay = 5000;
    int     m_port;

    vector<string>      m_colors;
    vector<vector<int>  m_ hsvValues;
};
