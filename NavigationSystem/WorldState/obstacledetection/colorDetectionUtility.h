
#ifndef DETECTUTILITY
#define DETECTUTILITY

#include <cmath>
#include <cstdio>
#include <iostream>
#include <opencv2/objdetect/objdetect.hpp>
#include <string>
#include <vector>
#include "../Messages/ObstacleVectorMsg.h"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

bool is_valid_double(double x);

bool find_if_close(std::vector<cv::Point> cnt1, std::vector<cv::Point> cnt2, float dist_is_close);

std::vector<cv::Point> merge2Contours(std::vector<cv::Point> cnt1,
                                      std::vector<cv::Point> cnt2,
                                      cv::Rect& rotated_bounding_rect);

std::vector<std::vector<cv::Point>> mergeAllContours(std::vector<std::vector<cv::Point>> contours,
                                                     float const& dist_is_close,
                                                     cv::Rect& rotated_bounding_rect,
                                                     std::vector<cv::Rect>& rotated_bounding_rects);

void printContour(std::vector<std::vector<cv::Point>> ct);
std::vector<std::vector<int>> findHsvTreshold(std::vector<std::string> colors,
                                              std::vector<cv::Scalar>& colorDrawing);
cv::Mat threshold(cv::Mat const& imgOriginal, std::vector<int> const& color);
void morphologicalOperations(cv::Mat& imgThresholded);
void computeContoursCentersRectangles(cv::Mat const& imgThresholded,
                                      std::vector<cv::Point2f>& mc,
                                      std::vector<cv::Rect>& rotated_bounding_rects,
                                      int minAreaToDetect);
void drawCentersRectangles(cv::Mat& imgOriginal,
                           std::vector<cv::Point2f> const& mc,
                           std::vector<cv::Rect> const& rotated_bounding_rects,
                           cv::Scalar const& aColorDrawing);
void initHsvColors();
void createTrackbarHSV(int numberOfColorsToTrack);
std::vector<cv::Rect> compareRects(cv::Mat& imgOriginal,
                                   std::vector<cv::Rect> const& rotated_bounding_rects);
std::vector<cv::Point> findRectanglesCenters(std::vector<cv::Rect> const& rects);
void supressSmallRectangles(std::vector<cv::Rect>& rects, int minAreaRectangle);
void computeObstaclesAnglePosition(
    cv::Mat const& imgOriginal,
    std::vector<ObstacleData>& Obstacles,
    std::vector<std::vector<cv::Rect>> rotated_bounding_rects_several_captures);
void printObstacleVector(std::vector<ObstacleData> v);
#endif
