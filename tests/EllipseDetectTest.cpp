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

void printImage(const cv::Mat& in){

    cv::Mat image = cv::imread("/home/flamingo/Desktop\Ellipse.png", 0);
    cv::namedWindow("loadingTest", CV_WINDOW_NORMAL);
    if(image.empty()){
        std::cout << "!!! Failed imread(): image not found" << std::endl;
    }

    cv::imshow("loadingTest", in);
    cv::waitKey(0);
}


TEST(EllipseTest, canDetectEllipse){
    cv::Mat image = cv::imread("/home/flamingo/Desktop\Ellipse.png", 0);
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
    std::vector<Arc> extractedArcs;
    extractedArcs = EllipseDetector::extractArcs(lineSegments, startPoints);

    //std::cout << "Number of extractedArcs: " << extractedArcs.size() << " should be less than generateLines(" << generateLines.size() << ")" << std::endl;
    for(int i = 0; i < extractedArcs.size(); i++){
        for(int j = 0; j < extractedArcs[i].lines->size(); j++);
            cv::Point pt1(extractedArcs[i].lines[j].start.x, extractedArcs[i].lines[j].start.y);
            cv::Point pt2(extractedArcs[i].lines[j].end.x, extractedArcs[i].lines[j].end.y);
            cv::line(image, pt1, pt2, cv::Scalar(255,0,0), 1, cv::LINE_8);
    }
    cv::namedWindow("ExtractedArcs", CV_WINDOW_NORMAL);

    //Tester hvilke extended arcs der findes
    std::vector<ExtendedArc> extendedArcs;
    extendedArcs = EllipseDetector::extractExtendedArcs(extractedArcs);

    //std::cout << "Number of extendedArcs: " << extendedArcs.size() << " should be less than extractedArcs (" << extractedArcs.size() << ")" << std::endl;
    /*for(int i = 0; i < extendedArcs.size(); i++){
        for(int j = 0; j < extendedArcs.arcs.size(); j++){
            for(int k = 0; k < extendedArcs.arcs[j].lines.size(); k++){
                cv::Point pt1(extendedArcs[i].arcs[j]->lines[k].start.x, extendedArcs[i].arcs[j]->lines[k].start.y);
                cv::Point pt2(extendedArcs[i].arcs[j]->lines[k].end.x, extendedArcs[i].arcs[j]->lines[k].end.y);
                cv::line(image, pt1, pt2, cv::Scalar(255,0,0), 1, cv::LINE_8);
            }
        }
    }*/
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



