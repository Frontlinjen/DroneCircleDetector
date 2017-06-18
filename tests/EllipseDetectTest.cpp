#include <gtest/gtest.h>
#include "ring_detector/EllipseDetector.h"
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <string>
#include "ring_detector/kdtree.h"
#include <cstdlib>
#include <iostream>
#include <opencv2/core/types.hpp>

TEST(EllipseTest, canDetectEllipse){
    cv::Mat image = cv::imread("/home/flamingo/Desktop/ellipser.png", 0);
    cv::namedWindow("test1", CV_WINDOW_NORMAL);
    if(image.empty()){
        std::cout << "!!! Failed imread(): image not found" << std::endl;
    }

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(image, lines, 1, (CV_PI/180.0)*15, 40, 10, 10);

    std::vector<Line> lineSegments[4];
    kdTree::PointContainer startPoints[4];
    EllipseDetector::generateLines(lines, lineSegments, startPoints);

    //Tester hvilke linjer der findes i billedet, og om de passer med vores generated lines
   // std::cout << "Number of Hough lines: " << lines.size() << std::endl;
    //std::cout << "Number of generated lines: " << lineSegments[0].size() + lineSegments[1].size() + lineSegments[2].size() + lineSegments[3].size() << std::endl;

    //Tester hvilke arcs der findes
    std::vector<Arc>* extractedArcs;
    extractedArcs = EllipseDetector::extractArcs(lineSegments, startPoints);

    //std::cout << "Number of extractedArcs: " << extractedArcs.size() << " should be less than generateLines(" << generateLines.size() << ")" << std::endl;
    for(std::vector<Arc>::iterator i = extractedArcs->begin() ; i != extractedArcs->end(); ++i){
        for(std::vector<Line>::iterator j = i->lines->begin() ; j != i->lines->end(); ++j){
        	cv::Point pt1(j->start.x, j->start.y);
            cv::Point pt2(j->end.x, j->end.y);
            cv::line(image, pt1, pt2, cv::Scalar(255,0,0), 1, cv::LINE_8);
        }
    }
    cv::namedWindow("ExtractedArcs", CV_WINDOW_NORMAL);

    //Tester hvilke extended arcs der findes
    std::vector<ExtendedArc> extendedArcs;
    extendedArcs = EllipseDetector::extractExtendedArcs(extractedArcs);

    //std::cout << "Number of extendedArcs: " << extendedArcs.size() << " should be less than extractedArcs (" << extractedArcs.size() << ")" << std::endl;
    for(std::vector<ExtendedArc>::iterator i = extendedArcs.begin() ; i != extendedArcs.end(); ++i){
        for(std::vector<Arc*>::iterator j = i->arcs.begin() ; j != i->arcs.end(); ++j){
            for(std::vector<Line>::iterator k = (*j)->lines->begin() ; k != (*j)->lines->end(); ++k){
            	cv::Point pt1(k->start.x, k->start.y);
            	cv::Point pt2(k->end.x, k->end.y);
                cv::line(image, pt1, pt2, cv::Scalar(255,0,0), 1, cv::LINE_8);
            }
        }
    }
    cv::namedWindow("ExtendedArcs", CV_WINDOW_NORMAL);

    //Tester hvilke ellipser der findes

    cv::namedWindow("Ellipses", CV_WINDOW_NORMAL);

    //Vi ser om testen virker eller fejler, og viser vores billede
    EXPECT_EQ("Hvad vi vil have", "hvad vores resultat er");
    cv::imshow("test1", image);
    cv::imshow("ExtractedArcs", image);
    cv::imshow("ExtendedArcs", image);
}

main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



