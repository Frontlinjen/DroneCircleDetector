#include "ring_detector/EllipseDetector.h"
#include <cmath>
#include "ring_detector/kdtree.h"
#include "ring_detector/ThomasAlgorithm.h"
#define TO_DEG(x) (x*(180/M_PI))
struct Vec2{
	int x;
	int y;
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
		container[0] = acos(dotp1);
		container[1] = acos(dotp2);
	}

	void innerAngle(Arc& other, float* container){

	}



};

std::vector<Arc> EllipseDetector::detect(const LineContainer lines){
	if(lines.size() == 0)
		return std::vector<Arc>();

	std::vector<Line> lineSegments[4];
	kdTree::PointContainer startPoints[4];
	{
		LineContainer::iterator itr = lines.begin();
		//Ls
		while(itr != lines.end()){
			float slope;
			Line l;
			kdLeaf leaf;
			l.start.x = (*itr)[0];
			l.start.y = (*itr)[1];
			l.end.x = (*itr)[2];
			l.end.y = (*itr)[3];
			l.mid.x = l.end.x - l.start.x;
			l.mid.y = l.end.y - l.start.y;
			l.angle = atan2(l.end.y - l.start.y, l.end.x - l.start.x);
			leaf.p = kdPoint { l.start.x, l.start.y };
			leaf.userdata = static_cast<void*>(&lineSegments.back());
			slope = (l.end.y - l.start.y) / (l.end.x - l.start.x)
					if(slope <= 0,5 && slope >= -0,5){
						lineSegments[0].push_back(l);
						startPoints[0].push_back(leaf);
					}
					else if(slope > 0,5 && slope <= 1){
						lineSegments[3].push_back(l);
						startPoints[3].push_back(leaf);
					}
					else if(slope < -0,5 && slope > -1){
						lineSegments[2].push_back(l);
						startPoints[2].push_back(leaf);
					}
					else{
						slope = (l.end.x - l.start.x)/(l.end.y - l.start.x)
						if(slope <= 0,5 && slope >= -0,5){
							lineSegments[1].push_back(l);
							startPoints[1].push_back(leaf);
						}
						else if(slope > 0,5 && slope <= 1){
							lineSegments[2].push_back(l);
							startPoints[2].push_back(leaf);
						}
						else if(slope < -0,5 && slope > -1){
							lineSegments[3].push_back(l);
							startPoints[3].push_back(leaf);
						}
					}
			++itr;
		}
	}
	kdTree tree;

	const float errLine = 45.0f;
	const int maxDistance = 20;
	std::vector<Arc> arcs;
	for(int i = 0; i < 4; i++){
		tree.buildTree(startPoints[i]);
		for(std::vector<Line>::iterator startLine = lineSegments[i].begin(); startLine != lineSegments[i].end(); ++startLine){

			//LA
			std::vector<Line>* arcCandidate = new std::vector<Line>();

			while(startLine != NULL){
				arcCandidate->push_back(*startLine);

				Rect r;
				r.center.x = startLine->end.x + 2;
				r.center.y = startLine->end.y;
				r.width = maxDistance;
				r.height = maxDistance;

				std::vector<kdLeaf> searchResult = tree.InRange(r);

				//Set it to NULL, so we know if a new candidate has been found
				startLine = NULL;
				for(std::vector<kdLeaf>::iterator cItr = searchResult.begin(); cItr != searchResult.end() ; ++cItr){
					Line * candidate = static_cast<Line*>(cItr->userdata);

					float intersectingAngle = std::abs(startLine->angle - candidate->angle);
					if(intersectingAngle > 0 && intersectingAngle < 45){
						//Calculating error
						std::vector<float> xs;
						std::vector<float> ys;

						for(std::vector<Line>::iterator candidateLine = arcCandidate->begin();candidateLine != arcCandidate->end(); ++candidateLine){
							xs.push_back(candidateLine->start.x);
							xs.push_back(candidateLine->end.x);
							ys.push_back(candidateLine->start.y);
							ys.push_back(candidateLine->end.y);
						}
						xs.push_back(candidate->start.x);
						xs.push_back(candidate->end.x);
						ys.push_back(candidate->start.y);
						ys.push_back(candidate->end.y);

						ring r = ThomasAlgorithm::thomasAlgorithm(xs, ys);

						float estimatedAngle =  TO_DEG(std::atan2(candidate->mid.x - r.centerX, r.centerY - candidate->mid.y));
						if(std::abs(candidate->angle - estimatedAngle) < errLine){
							startLine = candidate; //Start iteration using new candidate
							break;
						}
					}
				}
				//Save if we have more than one candidate
				if(startLine==NULL && arcCandidate->size() > 1){
					Arc a;
					a.lines = arcCandidate;
					std::vector<float> xs;
					std::vector<float> ys;
					for(std::vector<Line>::iterator candidate = arcCandidate->begin(); candidate != arcCandidate->end(); ++candidate){
						xs.push_back(candidate->start.x);
						ys.push_back(candidate->start.y);
						xs.push_back(candidate->end.x);
						ys.push_back(candidate->end.y);
					}
					ring r = ThomasAlgorithm::thomasAlgorithm(xs, ys);
					a.centerX = r.centerX;
					a.centerY = r.centerY;
					a.radius = r.radius;
					arcs.push_back(a);
				}
			}
		}
	}

	//Compute extended arcs:
	for(std::vector<Arc>::iterator startArc = arcs.begin(); startArc != arcs.end(); ++startArc){
		//TODO absolute distance
		//Gx < Darc
		//Gy < Darc
		//Darc bestemmer vi selv. Gx er x-værdien for vektoren mellem slutpunktet for sidste line i testarc og startpunktet for første line i candidateArc


		//TODO relative distance
		//drel = |AB|/|A| > dmin
		//AB er vektoren der forbinder arcsnes startpunkter. A og B peger fra start til end på hver arc.

		//TODO Gap angles
		//ThetaGap,a = Vinklen mellem(La og G)
		//ThetaGap,b = Vinklen mellem(Lb og G
		//La er sidste linje i arc a (testarc), og Lb er første linje i arc b (kandidat arc).
		//G forbinder slut i testarc med start i kandidat arc.

		//TODO Inner angles
		//

		//TODO Tangent Error
		//TODO Line
	}




}
