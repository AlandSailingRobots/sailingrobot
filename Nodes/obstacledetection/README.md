# Color Detection For Collision Avoidance

This program is a color detection program for collision avoidance.
This program is the active node version of the code available at https://github.com/AlandSailingRobots/Color_Detection 

## Parameters and adjustments

Check the code and readme available here : https://github.com/AlandSailingRobots/Color_Detection.
Settle the values of the colors to detect and save them in void initHsvColors() in colorDetectionUtility.cpp
Don't forgot to change float webcamAngleApertureX and float webcamAngleApertureY if needed.

## Initialize and start the node in main.cpp

You use either :
 colorDetectionNode(MessageBus& msgBus,std::vector<std::string> colors_input,int bottomPixelsToCrop);
 
Or :
 colorDetectionNode(MessageBus& msgBus,int numberOfCapturesPerDetection,
                        int port, int delay,std::vector<std::string> colors_input,int bottomPixelsToCrop);

Create a std::vector<std::string> colors_input with the colors you settled earlier.
If you need to crop the bottom part of the image to not detect the boat adjust int bottomPixelsToCrop;

	*int numberOfCapturesPerDetection : at each loop iteration the program will take numberOfCapturesPerDetection
		pictures and compute the position of the detected obstacles with
        a mean on those pictures.
        
	*int port :set to 0 for the inner camera of the computer and to 1 for an outer camera.
	
	*int delay : Delay between each loop iteration.
	
	*m_minAreaToDetect(2000) : is always set to 2000 in colorDetectionNode.cpp. If needed
			don't forget to adjust and change this.
	
	*m_maxAreaToDetect(20000) : is always set to 20000 in colorDetectionNode.cpp. If needed
			don't forget to adjust and change this.
 
