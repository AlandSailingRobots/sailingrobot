#include "detectUtility.h"
using namespace cv;
using namespace std;

Mat imgOriginal;
int hsvDiff = 10;
int iLowH = 0;
int iHighH = 179;
int iLowS = 0;
int iHighS = 255;
int iLowV = 0;
int iHighV = 255;
int iColor = 0;
Mat trackBarHSV = Mat3b(100, 300, Vec3b(0,0,0));
vector<string> colors;
vector<vector<int> > hsvValues;
int minAreaToDetect = 2000;
int maxAreaToDetect = 20000;
int numberOfCapturesPerDetection=5;
int delay = 5000;

void printError(){
    cout << "Error put valid colors as an input." << endl;
    cout <<"Valid colors : "<<endl;
    cout << "red" << endl;
    cout <<"orange" <<endl;
    cout << "yellow" << endl;
    cout <<"green" <<endl;
    cout << "blue" << endl;
    cout <<"purple" <<endl;
}
void changecolorTrackbar(){
    Mat HSVmat=  Mat::zeros(1,1 , CV_8UC3 );
    Mat RGBmat=  Mat::zeros(1,1 , CV_8UC3 );
    cv::Vec3b pixel ;
    pixel[0] = (int)(iLowH+iHighH)/2;
    pixel[1] = (int)(iLowS+iHighS)/2;
    pixel[2] = (int)(iLowV+iHighV)/2;
    HSVmat.at<Vec3b>(0,0)=pixel;
    cvtColor(HSVmat, RGBmat,CV_HSV2RGB);
    pixel=RGBmat.at<Vec3b>(0,0);
    trackBarHSV.setTo(Scalar(pixel[2],pixel[1],pixel[0]));
    imshow("Change Detection Values",trackBarHSV);

}

void update_trackbar_hsv_values( int, void* )
{
    iLowH = hsvValues[iColor][0];
    iHighH = hsvValues[iColor][3];
    iLowS = hsvValues[iColor][1];
    iHighS = hsvValues[iColor][4];
    iLowV = hsvValues[iColor][2];
    iHighV = hsvValues[iColor][5];
    setTrackbarPos("LowH","Change Detection Values",iLowH);
    setTrackbarPos("LowS","Change Detection Values",iLowS);
    setTrackbarPos("LowV","Change Detection Values",iLowV);
    setTrackbarPos("HighH","Change Detection Values",iHighH);
    setTrackbarPos("HighS","Change Detection Values",iHighS);
    setTrackbarPos("HighV","Change Detection Values",iHighV);
    changecolorTrackbar();

}

void get_trackbar_hsv_values( int, void* )
{
    hsvValues[iColor][0]=getTrackbarPos("LowH", "Change Detection Values");
    hsvValues[iColor][1]=getTrackbarPos("LowS", "Change Detection Values");
    hsvValues[iColor][2]=getTrackbarPos("LowV", "Change Detection Values");
    hsvValues[iColor][3]=getTrackbarPos("HighH", "Change Detection Values");
    hsvValues[iColor][4]=getTrackbarPos("HighS", "Change Detection Values");
    hsvValues[iColor][5]=getTrackbarPos("HighV", "Change Detection Values");
    changecolorTrackbar();
}
static void get_on_click_hsv_pixel_values( int event, int x, int y, int, void* )
{
    if( event == EVENT_LBUTTONDOWN ){
        Mat HSV;
        cvtColor(imgOriginal, HSV,CV_BGR2HSV);
        Vec3b pixel = HSV.at<Vec3b>(y,x);
        iLowH =(pixel[0]-hsvDiff);
        iHighH =(pixel[0]+hsvDiff);
        iLowS = (pixel[1]-hsvDiff);
        iHighS = (pixel[1]+hsvDiff);
        iLowV = (pixel[2]-hsvDiff);
        iHighV = (pixel[2]+hsvDiff);
        setTrackbarPos("LowH","Change Detection Values",iLowH);
        setTrackbarPos("LowS","Change Detection Values",iLowS);
        setTrackbarPos("LowV","Change Detection Values",iLowV);
        setTrackbarPos("HighH","Change Detection Values",iHighH);
        setTrackbarPos("HighS","Change Detection Values",iHighS);
        setTrackbarPos("HighV","Change Detection Values",iHighV);
        changecolorTrackbar();
    }
}
void initWindowsAndTrackbars(int numberOfColorsToTrack){
    namedWindow("Change Detection Values", CV_WINDOW_NORMAL); //create a window called "Change Detection Values"

    if(numberOfColorsToTrack>1){
        numberOfColorsToTrack=numberOfColorsToTrack-1;
        createTrackbar("Color", "Change Detection Values", &iColor, numberOfColorsToTrack,update_trackbar_hsv_values);
    }
    iLowH = hsvValues[0][0];
    iHighH = hsvValues[0][3];
    iLowS = hsvValues[0][1];
    iHighS = hsvValues[0][4];
    iLowV = hsvValues[0][2];
    iHighV = hsvValues[0][5];
    createTrackbar("minAreaToDetect", "Change Detection Values", &minAreaToDetect,maxAreaToDetect);

    //Create trackbars in "Change Detection Values" window
    createTrackbar("LowH", "Change Detection Values", &iLowH, 179, get_trackbar_hsv_values); //Hue (0 - 179)
    createTrackbar("HighH", "Change Detection Values", &iHighH, 179, get_trackbar_hsv_values);

    createTrackbar("LowS", "Change Detection Values", &iLowS, 255, get_trackbar_hsv_values); //Saturation (0 - 255)
    createTrackbar("HighS", "Change Detection Values", &iHighS, 255, get_trackbar_hsv_values);

    createTrackbar("LowV", "Change Detection Values", &iLowV, 255, get_trackbar_hsv_values);//Value (0 - 255)
    createTrackbar("HighV", "Change Detection Values", &iHighV, 255, get_trackbar_hsv_values);
    changecolorTrackbar();
    moveWindow("Change Detection Values", 0,0);

    namedWindow("Thresholded Image", CV_WINDOW_NORMAL);
    namedWindow("Detection", CV_WINDOW_NORMAL);
    resizeWindow("Thresholded Image", 500, 500);
    moveWindow("Thresholded Image", 700,0);
    resizeWindow("Detection", 500, 500);
    moveWindow("Detection", 1300,0);
}

int main( int argc, char** argv )
{
    initHsvColors();
    vector<Scalar> colorDrawing;
    int numberOfColorsToTrack = 0;
    Mat imgThresholded;
    vector<Point2f> mc;
    vector<Rect> rotated_bounding_rects;

    for(int i = 1; i< argc; i++){
        string input(argv[i]);
        if(input.compare("red")==0 ||input.compare("orange")==0 ||input.compare("yellow")==0 ||input.compare("green")==0 ||input.compare("blue")==0 ||input.compare("purple")==0){
            colors.push_back(argv[i]);
            numberOfColorsToTrack++;
        }
        else{
            printError();
            return -1;
        }
    }
    if(numberOfColorsToTrack==0){
        printError();
        return -1;
    }
    vector<Mat> imgsThresholded (numberOfColorsToTrack);
    hsvValues=findHsvTreshold(colors, colorDrawing);
    initWindowsAndTrackbars(numberOfColorsToTrack);
    vector<vector<Rect> > rotated_bounding_rects_several_captures(numberOfColorsToTrack);
    vector<vector<Rect> > rotated_bounding_rects_merged_list(numberOfColorsToTrack);
    vector<vector<Point> > centers(numberOfColorsToTrack);
    std::vector<ObstacleData> obstacles;

    VideoCapture cap(1); //capture the video from webcam
    if ( !cap.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }
    while (true)
    {
        for(int h = 0; h<numberOfCapturesPerDetection;h++){
            bool bSuccess = cap.read(imgOriginal); // read a new frame from video
            if (!bSuccess) { //if not success, break loop
                cout << "Cannot read a frame from video stream" << endl;
                break;
            }
            for(int i = 0; i<(int)hsvValues.size(); i++){
                imgThresholded=threshold(imgOriginal,hsvValues[i]);
                morphologicalOperations (imgThresholded);
                computeContoursCentersRectangles(imgThresholded,mc,rotated_bounding_rects,minAreaToDetect );
                //drawCentersRectangles(imgOriginal, mc,rotated_bounding_rects, colorDrawing[i]);
                rotated_bounding_rects_several_captures[i].insert(rotated_bounding_rects_several_captures[i].begin(),rotated_bounding_rects.begin(),rotated_bounding_rects.end());
                rotated_bounding_rects.erase(rotated_bounding_rects.begin(),rotated_bounding_rects.end());
                imgsThresholded[i]=imgThresholded;
            }
        }
        for( int i = 0; i<(int)rotated_bounding_rects_several_captures.size(); i++ ){
            //supressSmallRectangles(rotated_bounding_rects_several_captures[i], minAreaToDetect);
            rotated_bounding_rects_merged_list[i] = compareRects(imgOriginal,rotated_bounding_rects_several_captures[i]);
            centers[i]=findRectanglesCenters(rotated_bounding_rects_merged_list[i]);
            for(int j = 0; j < rotated_bounding_rects_merged_list[i].size(); j++){
                rectangle(imgOriginal, rotated_bounding_rects_merged_list[i][j],colorDrawing[i] ,4, 8,0);
                circle( imgOriginal, centers[i][j], 10,colorDrawing[i] , 4, 8, 0 );
            }
        }
        cout << rotated_bounding_rects_merged_list.size() << endl;
        computeObstaclesAnglePosition( imgOriginal, obstacles, rotated_bounding_rects_merged_list );
        printObstacleVector(obstacles);
        rotated_bounding_rects_several_captures.erase(rotated_bounding_rects_several_captures.begin(),rotated_bounding_rects_several_captures.end());
        rotated_bounding_rects_several_captures.resize(numberOfColorsToTrack);
        obstacles.erase(obstacles.begin(),obstacles.end());

        imshow("Thresholded Image", imgsThresholded[iColor]); //show the thresholded image
        imshow("Detection", imgOriginal); //show the original image
        setMouseCallback( "Detection", get_on_click_hsv_pixel_values, 0 );
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
        waitKey(delay);
    }

    return 0;
}
