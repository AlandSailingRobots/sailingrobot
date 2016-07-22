#include "Utility.h"
#include <algorithm>
#include <cmath>


int Utility::combineBytes(uint8_t MSB, uint8_t LSB)
{
	int combined = 0;

	combined = MSB;
	combined = (MSB << 8) + LSB;

	return combined;
}

int Utility::combineBytesSigned(uint8_t MSB, uint8_t LSB)
{
	int16_t combined = 0;

	combined = MSB;
	combined = (MSB << 8) + LSB;

	return int(combined);
}

bool Utility::IsOutOfRange (float f) {
	return (f < 90 || f > 270);
}

std::vector<float> Utility::fixAngles(std::vector<float> v){
	unsigned int counter =  count_if(v.begin(), v.end(), Utility::IsOutOfRange);
	if (counter > (v.size()/2)) {
		std::vector<float> newV(v);
		for (float& f : newV) {
			if (f > 180 && f < 360) {
				f -= 360;
			}
		}
		return newV;
	}
	return v;
}

float Utility::getMedianValue(std::vector<float> v) {
	if(v.empty()) {
		return 0;
	}
	std::vector<float> fixedV = Utility::fixAngles(v);
	sort(fixedV.begin(), fixedV.begin() + fixedV.size());
	unsigned int middle = (int) fixedV.size()/2;
	float middleValue = 0;
	if (fixedV.size() % 2 == 1) {
		middleValue = fixedV.at(middle);
	}
	else {
		if (fixedV.size() > middle) {
			middleValue = (fixedV.at(middle-1) + fixedV.at(middle)) / 2;
		}
		else {
			middleValue = fixedV.at(middle);
		}
	}
	if(middleValue < 0 ){
		middleValue+=360;
	}
	return middleValue;
}

float Utility::mean(std::vector<float> values)
{
	if (values.size() < 1) return 0;

	float sum = 0;

	for(std::vector<float>::iterator it = values.begin();
		it != values.end(); ++it)
	{
		sum += *it;
	}

	return sum / values.size();
}

/*
 * uses formula for calculating mean of angles
 * https://en.wikipedia.org/wiki/Mean_of_circular_quantities
 */
float Utility::meanOfAngles(std::vector<float> anglesInDegrees)
{
	if (anglesInDegrees.size() < 1) return 0;

	std::vector<float> xx, yy;
	float x, y;

	// convert all angles to cartesian coordinates
	for(std::vector<float>::iterator it = anglesInDegrees.begin();
		it != anglesInDegrees.end(); ++it)
	{
		polarToCartesian(*it, x, y);
		xx.push_back(x);
		yy.push_back(y);
	}

	// use formula
	float meanAngleRadians = atan2(mean(yy), mean(xx));
	// atan2 produces results in the range (−π, π],
	// which can be mapped to [0, 2π) by adding 2π to negative results
	if (meanAngleRadians < 0) meanAngleRadians += 2*M_PI;

	return meanAngleRadians * 180/M_PI;
}

int Utility::sgn(double value)
{
	if(value == 0) return 0;
	if(value < 0) return -1;
	if(value > 0) return 1;

	return 0;
}

void Utility::polarToCartesian(float degrees, float& x, float& y)
{
	x = cos(degrees * M_PI/180);
	y = sin(degrees * M_PI/180);
}

bool Utility::isAngleInSector(double angle, double sectorAngle1, double sectorAngle2)
{
	double start = 0;
	double end = limitAngleRange(sectorAngle2 - sectorAngle1);
	double toCheck = limitAngleRange(angle - sectorAngle1);

	bool angleIsInSector = false;
	if (toCheck >= start && toCheck <= end)
		angleIsInSector = true;

	return angleIsInSector;
}


double Utility::angleDifference(double angle1, double angle2)
{
	const double fullRevolution = 360;

	double diff = std::abs(limitAngleRange(angle1) - limitAngleRange(angle2));
	if (diff > fullRevolution/2) diff = fullRevolution - diff;

	return diff;
}


double Utility::limitAngleRange(double angle)
{
	const double fullRevolution = 360;
	const double minAngle = 0;

	while (angle < minAngle)
		angle += fullRevolution;

	while (angle >= minAngle + fullRevolution)
		angle -= fullRevolution;

	return angle;
}

double Utility::limitRadianAngleRange(double angle)
{
	const double fullRevolution = 2 * M_PI;
	const double minAngle = 0;

	while (angle < minAngle)
		angle += fullRevolution;

	while (angle >= minAngle + fullRevolution)
		angle -= fullRevolution;

	return angle;
}


double Utility::degreeToRadian(double degrees)
{
	return degrees * M_PI / 180;
}


double Utility::radianToDegree(double radians)
{
	return radians / M_PI * 180;
}

int Utility::addDeclinationToHeading(int heading, int declination) {
	return static_cast<int> (Utility::limitAngleRange(heading + declination) + 0.5);
}

double Utility::directionAdjustedSpeed(double gpsHeading,double compassHeading,double gpsSpeed) {
	double speed = 0;

		if (Utility::angleDifference(gpsHeading,compassHeading) < 90)
		{
			speed = gpsSpeed;
		}

		return speed;
}
/*
 * uses formula for calculating true Wind Direction
 * https://en.wikipedia.org/wiki/Apparent_wind
 */
double Utility::calculateTrueWindDirection(const SystemStateModel& systemStateModel , double heading){

	//double knots = 1.94384;
	double apparentWindSpeed = systemStateModel.windsensorModel.speed; //* knots; // Converting m/s to knots
	double apparentWindAngle = systemStateModel.windsensorModel.direction;
	double boatSpeed = systemStateModel.gpsModel.speed;

	if (apparentWindAngle < 0.001 ){
		apparentWindAngle = 0.001;
	} else if ( apparentWindAngle > 359.999 ){
		apparentWindAngle = 359.999 ;
	}

	if(apparentWindSpeed < 0.001){
		return heading;
	}

	double trueWindSpeed = sqrt((apparentWindSpeed * apparentWindSpeed) + (boatSpeed * boatSpeed)
						 - (2 * boatSpeed * apparentWindSpeed * cos(apparentWindAngle/180*M_PI)));

	double alpha = acos((apparentWindSpeed * cos(apparentWindAngle/180*M_PI) - boatSpeed)
					/ trueWindSpeed)* 180/M_PI;

	double twd = 0;
	if (apparentWindAngle > 180){
		twd = Utility::limitAngleRange(heading - alpha);
	} else {
		twd = Utility::limitAngleRange(heading + alpha);
	}

	return twd;
}

double Utility::calculateTrueWindSpeed(const SystemStateModel& systemStateModel , double heading)
{
	//double knots = 1.94384;
	double apparentWindSpeed = systemStateModel.windsensorModel.speed; //* knots; // Converting m/s to knots
	double apparentWindAngle = systemStateModel.windsensorModel.direction;
	double boatSpeed = systemStateModel.gpsModel.speed;

	if (apparentWindAngle < 0.001 ){
		apparentWindAngle = 0.001;
	} else if ( apparentWindAngle > 359.999 ){
		apparentWindAngle = 359.999 ;
	}

	if(apparentWindSpeed < 0.001){
		return heading;
	}

	double trueWindSpeed = sqrt((apparentWindSpeed * apparentWindSpeed) + (boatSpeed * boatSpeed)
						 - (2 * boatSpeed * apparentWindSpeed * cos(apparentWindAngle/180*M_PI)));

	return trueWindSpeed;
}

double Utility::getTrueWindDirection(SystemStateModel systemStateModel, std::vector<float> &twdBuffer, const unsigned int twdBufferMaxSize)
{
	double twd = calculateTrueWindDirection(systemStateModel, systemStateModel.compassModel.heading);
	twdBuffer.push_back(twd);// new wind calculation
	//twdBuffer.push_back(systemStateModel.gpsModel.heading + systemStateModel.windsensorModel.direction);// old wind calculation

	while (twdBuffer.size() > twdBufferMaxSize) {
		twdBuffer.erase(twdBuffer.begin());
	}

	return meanOfAngles(twdBuffer);
}


void Utility::calculateApparentWind(const SystemStateModel systemStateModel, const double heading,
	           const double trueWindDirection,double &apparentWindSpeed, double &apparentWindDirection)
{ //referenced from "Modeling, control and state-estimation for an autonomous sailboat" by Jon Melin
  /* the trueWindDirection is the origin of the wind but the computation suppose its the direction so we change it during the function*/

  double trueWindDirection_ = trueWindDirection+M_PI;
	std::array<double, 2> wcaw = { calculateTrueWindSpeed(systemStateModel, heading) * cos(trueWindDirection_ - heading) - systemStateModel.gpsModel.speed, //wcaw[0]
									calculateTrueWindSpeed(systemStateModel, heading) * sin(trueWindDirection_ - heading)}; //wcaw[1] Cartesian Apparent Wind

	apparentWindSpeed = sqrt(pow(wcaw[0], 2) + pow(wcaw[1], 2));
	apparentWindDirection = -atan2(wcaw[0], wcaw[1])*180/M_PI;
}

double Utility::getApparentWindSpeed(const SystemStateModel systemStateModel, const double heading, const double trueWindDirection)
{
	double apparentWindSpeed,apparentWindDirection;
	calculateApparentWind(systemStateModel, heading, trueWindDirection,apparentWindSpeed,apparentWindDirection);
	return apparentWindSpeed;
}

double Utility::getApparentWindDirection(const SystemStateModel systemStateModel, const double heading, const double trueWindDirection)
{
	double apparentWindSpeed,apparentWindDirection;
	calculateApparentWind(systemStateModel, heading, trueWindDirection,apparentWindSpeed,apparentWindDirection);
	return apparentWindDirection;
}
