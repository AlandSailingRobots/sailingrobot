#include "colorDetectionNode.h"
#include "SystemServices/Logger.h"
using namespace cv;
using namespace std;
#define DISPLAY_WINDOWS_AND_TRACKBARS 1

colorDetectionNode::colorDetectionNode(MessageBus& msgBus,std::vector<string> colors_input,int bottomPixelsToCrop)
	: ActiveNode(NodeID::ColorDetection, msgBus),m_hsvDiff(10),m_iLowH(0),
	m_iHighH(179),m_iLowS(0),m_iHighS(255),m_iLowV(0),m_iHighV(255),m_iColor(0),
	m_numberOfColorsToTrack(0),m_Initialised(false),m_minAreaToDetect(2000),
	m_maxAreaToDetect(20000),m_numberOfCapturesPerDetection(5),m_delay(1),m_port(0),m_bottomPixelsToCrop(bottomPixelsToCrop)
{
	vector<string> colors;
	m_trackBarHSV = Mat3b(100, 300, Vec3b(0,0,0));

	int numberOfColorInput=(int)colors_input.size();
	m_inputColorError=false;
	for(int i = 0; i< numberOfColorInput; i++){
        if(colors_input[i].compare("red")==0 ||colors_input[i].compare("orange")==0
		||colors_input[i].compare("yellow")==0 ||colors_input[i].compare("green")==0
		||colors_input[i].compare("blue")==0 ||colors_input[i].compare("purple")==0){
            colors.push_back(colors_input[i]);
            m_numberOfColorsToTrack++;
        }
        else{
			Logger::error("%sWrong color input, at least one color doesn't exists", __PRETTY_FUNCTION__);
			Logger::error("Error put valid colors as an input : ");
			Logger::error("red\norange\nyellow\ngreen\nblue\npurple");
			m_inputColorError=true;
        }
    }
    if(m_numberOfColorsToTrack==0){
		Logger::error("%sWrong color input, you have to give at least one color input", __PRETTY_FUNCTION__);
		Logger::error("Error put valid colors as an input : ");
		Logger::error("red\norange\nyellow\ngreen\nblue\npurple");
        m_inputColorError=true;
    }
	initHsvColors();
	m_hsvValues=findHsvTreshold(colors, m_colorDrawing);
}

colorDetectionNode::colorDetectionNode(MessageBus& msgBus,
	int m_numberOfCapturesPerDetection, int port, int delay,std::vector<string> colors_input,int bottomPixelsToCrop)
	: ActiveNode(NodeID::ColorDetection, msgBus),m_hsvDiff(10),m_iLowH(0),
	m_iHighH(179),m_iLowS(0),m_iHighS(255),m_iLowV(0),m_iHighV(255),m_iColor(0),
	m_numberOfColorsToTrack(0),m_Initialised(false),m_minAreaToDetect(2000),
	m_maxAreaToDetect(20000),m_numberOfCapturesPerDetection(m_numberOfCapturesPerDetection),
	m_delay(delay),m_port(port),m_bottomPixelsToCrop(bottomPixelsToCrop)
{

	vector<string> colors;
	m_trackBarHSV = Mat3b(100, 300, Vec3b(0,0,0));

	int numberOfColorInput=(int)colors_input.size();
	m_inputColorError=false;
	for(int i = 0; i< numberOfColorInput; i++){
        if(colors_input[i].compare("red")==0 ||colors_input[i].compare("orange")==0
		||colors_input[i].compare("yellow")==0 ||colors_input[i].compare("green")==0
		||colors_input[i].compare("blue")==0 ||colors_input[i].compare("purple")==0){
            colors.push_back(colors_input[i]);
            m_numberOfColorsToTrack++;
        }
        else{
			Logger::error("%sWrong color input, at least one color doesn't exists", __PRETTY_FUNCTION__);
			Logger::error("Error put valid colors as an input : ");
			Logger::error("red\norange\nyellow\ngreen\nblue\npurple");
			m_inputColorError=true;
        }
    }
    if(m_numberOfColorsToTrack==0){
		Logger::error("%sWrong color input, you have to give at least one color input", __PRETTY_FUNCTION__);
		Logger::error("Error put valid colors as an input : ");
		Logger::error("red\norange\nyellow\ngreen\nblue\npurple");
        m_inputColorError=true;
    }
	initHsvColors();
	m_hsvValues=findHsvTreshold(colors, m_colorDrawing);
}

bool colorDetectionNode::init(){
	VideoCapture cap(m_port);
	m_cap=cap; //capture the video from webcam
    if ( !m_cap.isOpened() )  // if not success, log an error and prevent the thread to start
    {
		Logger::error("%sCannot open the webcam, check usb port", __PRETTY_FUNCTION__);
        m_Initialised=false;
    }
	else{
		m_Initialised=true;

	}
	return m_Initialised;

}

void colorDetectionNode::start()
{
	if(m_Initialised && !m_inputColorError)
	{
		runThread(colorDetectionThreadFunc);
	}
	else if(!m_Initialised)
	{
		Logger::error("%s Cannot start colorDetectionNode thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
	}
	else if(m_inputColorError)
	{
		Logger::error("%sCannot start colorDetectionNode thread because of a wrong color input!", __PRETTY_FUNCTION__);
	}
}


/*Update the displayed color on the trackbar window*/
void colorDetectionNode::changecolorTrackbar(void* nodePtr){
	colorDetectionNode* node = (colorDetectionNode*)nodePtr;
	//Need to convert HSV to RGB
    Mat HSVmat=  Mat::zeros(1,1 , CV_8UC3 );
    Mat RGBmat=  Mat::zeros(1,1 , CV_8UC3 );
    cv::Vec3b pixel ;
    pixel[0] = (int)(node->m_iLowH+node->m_iHighH)/2;
    pixel[1] = (int)(node->m_iLowS+node->m_iHighS)/2;
    pixel[2] = (int)(node->m_iLowV+node->m_iHighV)/2;
    HSVmat.at<Vec3b>(0,0)=pixel;
    cvtColor(HSVmat, RGBmat,CV_HSV2RGB);
    pixel=RGBmat.at<Vec3b>(0,0);
    node->m_trackBarHSV.setTo(Scalar(pixel[2],pixel[1],pixel[0]));//m_trackBarHSV is in BGR
    imshow("Change Detection Values",node->m_trackBarHSV);
}

/*Set the thrackbar HSV position to the value of m_iLowH,m_iHighH ... etc*/
void colorDetectionNode::update_trackbar_hsv_values( int val , void* nodePtr )
{
	colorDetectionNode* node = (colorDetectionNode*)nodePtr;
    node->m_iLowH = node->m_hsvValues[node->m_iColor][0];
    node->m_iHighH =node-> m_hsvValues[node->m_iColor][3];
    node->m_iLowS = node->m_hsvValues[node->m_iColor][1];
	node->m_iHighS =node->m_hsvValues[node->m_iColor][4];
    node->m_iLowV = node->m_hsvValues[node->m_iColor][2];
    node->m_iHighV = node->m_hsvValues[node->m_iColor][5];
    setTrackbarPos("LowH","Change Detection Values",node->m_iLowH);
    setTrackbarPos("LowS","Change Detection Values",node->m_iLowS);
    setTrackbarPos("LowV","Change Detection Values",node->m_iLowV);
    setTrackbarPos("HighH","Change Detection Values",node->m_iHighH);
    setTrackbarPos("HighS","Change Detection Values",node->m_iHighS);
    setTrackbarPos("HighV","Change Detection Values",node->m_iHighV);
    node->changecolorTrackbar(nodePtr);
}

/*Update  m_iLowH,m_iHighH ... when the user change the trackbar position*/
void colorDetectionNode::get_trackbar_hsv_values( int val, void* nodePtr )
{
	colorDetectionNode* node = (colorDetectionNode*)nodePtr;
    node->m_hsvValues[node->m_iColor][0]=getTrackbarPos("LowH", "Change Detection Values");
    node->m_hsvValues[node->m_iColor][1]=getTrackbarPos("LowS", "Change Detection Values");
    node->m_hsvValues[node->m_iColor][2]=getTrackbarPos("LowV", "Change Detection Values");
    node->m_hsvValues[node->m_iColor][3]=getTrackbarPos("HighH", "Change Detection Values");
    node->m_hsvValues[node->m_iColor][4]=getTrackbarPos("HighS", "Change Detection Values");
    node->m_hsvValues[node->m_iColor][5]=getTrackbarPos("HighV", "Change Detection Values");
    node->changecolorTrackbar(nodePtr);
}

/*Update m_iLowH,m_iHighH ... when the user click on the image.The color to
detect will be the color of the clicked pixel*/
void colorDetectionNode::get_on_click_hsv_pixel_values( int event, int x, int y, int val , void* nodePtr )
{
	colorDetectionNode* node = (colorDetectionNode*)nodePtr;
    if( event == EVENT_LBUTTONDOWN ){
        Mat HSV;
        cvtColor(node->m_imgOriginal, HSV,CV_BGR2HSV);
        Vec3b pixel = HSV.at<Vec3b>(y,x);
        node->m_iLowH =(pixel[0]-node->m_hsvDiff);
        node->m_iHighH =(pixel[0]+node->m_hsvDiff);
        node->m_iLowS = (pixel[1]-node->m_hsvDiff);
        node->m_iHighS = (pixel[1]+node->m_hsvDiff);
        node->m_iLowV = (pixel[2]-node->m_hsvDiff);
        node->m_iHighV = (pixel[2]+node->m_hsvDiff);
        setTrackbarPos("LowH","Change Detection Values",node->m_iLowH);
        setTrackbarPos("LowS","Change Detection Values",node->m_iLowS);
        setTrackbarPos("LowV","Change Detection Values",node->m_iLowV);
        setTrackbarPos("HighH","Change Detection Values",node->m_iHighH);
        setTrackbarPos("HighS","Change Detection Values",node->m_iHighS);
        setTrackbarPos("HighV","Change DeteccolorDetectionNode::tion Values",node->m_iHighV);
        node->changecolorTrackbar(nodePtr);
    }
}
/*Create Windows and Trackbars*/
void colorDetectionNode::initWindowsAndTrackbars(void* nodePtr){
	colorDetectionNode* node = (colorDetectionNode*)nodePtr;

	//create a window called "Change Detection Values"
    namedWindow("Change Detection Values", CV_WINDOW_NORMAL);

	//Create trackbars in "Change Detection Values" window
    if(node->m_numberOfColorsToTrack>1){
        node->m_numberOfColorsToTrack=node->m_numberOfColorsToTrack-1;
        createTrackbar("Color", "Change Detection Values", &(node->m_iColor),node-> m_numberOfColorsToTrack,update_trackbar_hsv_values,nodePtr);
    }
    node->m_iLowH =node->m_hsvValues[0][0];
    node->m_iHighH = node->m_hsvValues[0][3];
    node->m_iLowS = node->m_hsvValues[0][1];
    node->m_iHighS = node->m_hsvValues[0][4];
    node->m_iLowV = node->m_hsvValues[0][2];
    node->m_iHighV = node->m_hsvValues[0][5];
    createTrackbar("m_minAreaToDetect", "Change Detection Values", &m_minAreaToDetect,m_maxAreaToDetect);
    createTrackbar("LowH", "Change Detection Values", &m_iLowH, 179, get_trackbar_hsv_values,nodePtr); //Hue (0 - 179)
    createTrackbar("HighH", "Change Detection Values", &m_iHighH, 179, get_trackbar_hsv_values,nodePtr);

    createTrackbar("LowS", "Change Detection Values", &m_iLowS, 255, get_trackbar_hsv_values,nodePtr); //Saturation (0 - 255)
    createTrackbar("HighS", "Change Detection Values", &m_iHighS, 255, get_trackbar_hsv_values,nodePtr);

    createTrackbar("LowV", "Change Detection Values", &m_iLowV, 255, get_trackbar_hsv_values,nodePtr);//Value (0 - 255)
    createTrackbar("HighV", "Change Detection Values", &m_iHighV, 255, get_trackbar_hsv_values,nodePtr);
    changecolorTrackbar(nodePtr);
    moveWindow("Change Detection Values", 0,0);

    namedWindow("Thresholded Image", CV_WINDOW_NORMAL);
    namedWindow("Detection", CV_WINDOW_NORMAL);
    resizeWindow("Thresholded Image", 500, 500);
    moveWindow("Thresholded Image", 700,0);
    resizeWindow("Detection", 500, 500);
    moveWindow("Detection", 1300,0);
}

void colorDetectionNode::colorDetectionThreadFunc(void* nodePtr)
{
	colorDetectionNode* node = (colorDetectionNode*)nodePtr;
	Logger::info("colorDetectionNode thread started");

    Mat imgThresholded;
    vector<Point2f> mc;
    vector<Rect> rotated_bounding_rects;
    vector<Mat> imgsThresholded (node->m_numberOfColorsToTrack);
    if(DISPLAY_WINDOWS_AND_TRACKBARS == 1){
		node->initWindowsAndTrackbars(nodePtr);
	}
    vector<vector<Rect> > rotated_bounding_rects_several_captures(node->m_numberOfColorsToTrack);
    vector<vector<Rect> > rotated_bounding_rects_merged_list(node->m_numberOfColorsToTrack);
    vector<vector<Point> > centers(node->m_numberOfColorsToTrack);
    std::vector<ObstacleData> obstacles;

    while (true)
    {
		
		//For each capture
        for(int h = 0; h<(int)(node->m_numberOfCapturesPerDetection);h++){
			// read a new frame from video
            bool bSuccess = node->m_cap.read(node->m_imgOriginal);
            if (!bSuccess) {
				Logger::warning("%sCannot read a frame from video stream!", __PRETTY_FUNCTION__);
				continue;
            }
            //Crop the part corresponding to the boat
            // Setup a rectangle to define your region of interest
            cv::Rect myROI(0, 0, node->m_imgOriginal.cols,node->m_imgOriginal.rows - node->m_bottomPixelsToCrop);
            // Crop the full image to that image contained by the rectangle myROI
            // Note that this doesn't copy the data
            node->m_imgOriginal = node->m_imgOriginal(myROI);

			//For each color to track find obstacles
            for(int i = 0; i<(int)node->m_hsvValues.size(); i++){
                imgThresholded=threshold(node->m_imgOriginal,node->m_hsvValues[i]);
                morphologicalOperations (imgThresholded);
                computeContoursCentersRectangles(imgThresholded,mc,rotated_bounding_rects,node->m_minAreaToDetect );
                rotated_bounding_rects_several_captures[i].insert(rotated_bounding_rects_several_captures[i].begin(),rotated_bounding_rects.begin(),rotated_bounding_rects.end());
                rotated_bounding_rects.erase(rotated_bounding_rects.begin(),rotated_bounding_rects.end());
                imgsThresholded[i]=imgThresholded;
            }
        }
		//For each obstacles found merge the obstacles close to each others
        for( int i = 0; i<(int)rotated_bounding_rects_several_captures.size(); i++ ){
            //supressSmallRectangles(rotated_bounding_rects_several_captures[i], m_minAreaToDetect);
            rotated_bounding_rects_merged_list[i] = compareRects(node->m_imgOriginal,rotated_bounding_rects_several_captures[i]);
            centers[i]=findRectanglesCenters(rotated_bounding_rects_merged_list[i]);

			//Display merged rectangles and centers found
            for(int j = 0; j <(int)rotated_bounding_rects_merged_list[i].size(); j++){
                rectangle(node->m_imgOriginal, rotated_bounding_rects_merged_list[i][j],node->m_colorDrawing[i] ,4, 8,0);
                circle( node->m_imgOriginal, centers[i][j], 10,node->m_colorDrawing[i] , 4, 8, 0 );
            }
        }

        computeObstaclesAnglePosition(node->m_imgOriginal, obstacles, rotated_bounding_rects_merged_list );
		ObstacleVectorMsg* msg = new ObstacleVectorMsg(NodeID::Lidar, NodeID::ColorDetection ,obstacles);
		node->m_MsgBus.sendMessage(msg);

        rotated_bounding_rects_several_captures.erase(rotated_bounding_rects_several_captures.begin(),rotated_bounding_rects_several_captures.end());
        rotated_bounding_rects_several_captures.resize(node->m_numberOfColorsToTrack);
        obstacles.erase(obstacles.begin(),obstacles.end());

		if(DISPLAY_WINDOWS_AND_TRACKBARS == 1){
			imshow("Thresholded Image", imgsThresholded[node->m_iColor]);
			cvWaitKey(1);
			imshow("Detection", node->m_imgOriginal);
			cvWaitKey(1);
			setMouseCallback( "Detection", get_on_click_hsv_pixel_values, nodePtr );
		}
		// Controls how often we pump out messages
		std::this_thread::sleep_for(std::chrono::milliseconds(node->m_delay));
    }

}

void colorDetectionNode::processMessage(const Message* msgPtr)
{

}
