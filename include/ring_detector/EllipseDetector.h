#include <vector>
#include <opencv2/core/core.hpp>
#include "ring_detector/kdtree.h"

struct Vec2{
	int x;
	int y;
};

struct Ellipse{
	float centerX;
	float centerY;
	float a;
	float b;
	float rotation;
};

struct Line{
	Vec2 start;
	Vec2 mid;
	Vec2 end;
	float angle;
};


struct Arc{
	std::vector<Line> * lines;
	float centerX;
	float centerY;
	float radius;

	float absoluteDistance(Arc& other){
		Line& line1 = lines->back();
		Line& line2 = other.lines->front();
		int deltaX = line2.start.x - line1.end.x;
		int deltaY = line2.start.y - line1.end.y;
		return pow(deltaX, 2) + pow(deltaY, 2);
	}

	float relativeDistance(Arc& other){
		Vec2 A;
		Vec2 B;
		Vec2 AB;

		A.x = lines->back().end.x - lines->front().start.x;
		A.y = lines->back().end.y - lines->front().start.y;

		B.x = other.lines->back().end.x - other.lines->front().start.x;
		B.y = other.lines->back().end.y - other.lines->front().start.y;

		AB.x = other.lines->front().start.x - lines->front().start.x;
		AB.y = other.lines->front().start.y - lines->front().start.y;

		return (pow(AB.x, 2) + pow(AB.y, 2))/(pow(A.x, 2) + pow(A.y, 2));
	}

	void gapAngle(Arc& other, float* container){
		Line& last = lines->back();
		Line& first = other.lines->front();
		Vec2 G;
		G.x = last.start.x - first.end.x;
		G.y = last.start.y - first.end.y;

		Vec2 lastVec;
		lastVec.x = last.end.x - last.start.x;
		lastVec.y = last.end.y - last.start.y;

		Vec2 firstVec;
		firstVec.x = first.end.x - first.start.x;
		firstVec.y = first.end.y - first.start.y;


		float dotp1 = firstVec.x * G.x + firstVec.y * G.y;
		float dotp2 = lastVec.x * G.x + lastVec.y * G.y;

		float lenFirst = sqrt(pow(firstVec.x, 2) + pow(firstVec.y, 2));
		float lenLast = sqrt(pow(lastVec.x, 2) + pow(lastVec.y, 2));
		float lenG = sqrt(pow(G.x, 2) + pow(G.y, 2));

		container[0] = acos(dotp1 / (lenFirst + lenG));
		container[1] = acos(dotp2 / (lenLast + lenG));
	}
};


struct ExtendedArc{
	Arc* arcs[3];
	Vec2 position;
	float a;
	float b;
	float alpha;
};



class EllipseDetector{
  typedef std::vector<cv::Vec4i> LineContainer;

 public:
  static std::vector<Arc> detect(const LineContainer  lines);
 protected:
  static void generateLines(const LineContainer& lines, std::vector<Line> (& lineSegments)[4], kdTree::PointContainer (& startPoints)[4]);
  static std::vector<Arc> extractArcs(std::vector<Line> (& lineSegments)[4], kdTree::PointContainer (& startPoints)[4]);
};
