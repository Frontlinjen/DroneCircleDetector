#include "ring_detector/EllipseDetector.h"
#include <cmath>
#include "ring_detector/ThomasAlgorithm.h"
#define TO_DEG(x) (x*(180/M_PI))

bool TestInnerAngles(Arc& first, Arc& second, Arc& third){

	const int kContainerCount = 3;
	std::vector<Line>* lines[kContainerCount];
	lines[0] = first.lines;
	lines[1] = second.lines;
	lines[2] = third.lines;

	Line * current;
	for(unsigned int i = 0; i < kContainerCount; ++i){
		for(std::vector<Line>::iterator itr = lines[i]->begin(); itr!= lines[i]->end(); ++itr){
			Vec2 normalVec;
			normalVec.x = -itr->start.y;
			normalVec.y = itr->start.x;
			for(unsigned int i = i; i < kContainerCount; ++i){
				for(std::vector<Line>::iterator itr2 = (itr+1); itr2!= lines[i]->end(); ++itr2){
					Vec2 vecIJ, normalVec2;
					vecIJ.x = itr2->start.x - itr->start.x;
					vecIJ.y = itr2->start.y - itr->start.y;
					normalVec2.x = -itr2->start.y;
					normalVec2.y = itr2->start.x;

					float dotp1, dotp2;
					dotp1 = normalVec.x * vecIJ.x + normalVec.y * vecIJ.y;
					dotp2 = normalVec2.x * vecIJ.x + normalVec2.y * vecIJ.y;

					if(!(dotp1 > 0 && dotp2 > 0)){
						return false;
					}
				}
			}
		}
	}

}

void EllipseDetector::generateLines(const LineContainer& lines, std::vector<Line>(& lineSegments)[4], kdTree::PointContainer(& startPoints)[4]){
	LineContainer::const_iterator itr = lines.begin();
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
		slope = (l.end.y - l.start.y) / (l.end.x - l.start.x);
		if(slope <= 0,5 && slope >= -0,5){
			lineSegments[0].push_back(l);
			leaf.userdata = static_cast<void*>(&lineSegments[0].back());
			startPoints[0].push_back(leaf);
		}
		else if(slope > 0,5 && slope <= 1){
			lineSegments[3].push_back(l);
			leaf.userdata = static_cast<void*>(&lineSegments[3].back());
			startPoints[3].push_back(leaf);
		}
		else if(slope < -0,5 && slope > -1){
			lineSegments[2].push_back(l);
			leaf.userdata = static_cast<void*>(&lineSegments[2].back());
			startPoints[2].push_back(leaf);
		}
		else{
			slope = (l.end.x - l.start.x)/(l.end.y - l.start.x);
			if(slope <= 0,5 && slope >= -0,5){
				lineSegments[1].push_back(l);
				leaf.userdata = static_cast<void*>(&lineSegments[1].back());
				startPoints[1].push_back(leaf);
			}
			else if(slope > 0,5 && slope <= 1){
				lineSegments[2].push_back(l);
				leaf.userdata = static_cast<void*>(&lineSegments[2].back());
				startPoints[2].push_back(leaf);
			}
			else if(slope < -0,5 && slope > -1){
				lineSegments[3].push_back(l);
				leaf.userdata = static_cast<void*>(&lineSegments[3].back());
				startPoints[3].push_back(leaf);
			}
		}
		++itr;
}}

std::vector<Arc> EllipseDetector::extractArcs(std::vector<Line> (& lineSegments)[4], kdTree::PointContainer (& startPoints)[4]){
		kdTree tree;
		const float errLine = 45.0f;
		const int maxDistance = 20;
		std::vector<Arc> arcs;
		for(int i = 0; i < 4; i++){
			tree.buildTree(startPoints[i]);
			for(Line* startLine = &(lineSegments[i].front()); startLine != &(lineSegments[i].back()) + 1; ++startLine){
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
		return arcs;
}

std::vector<Arc> EllipseDetector::detect(const LineContainer lines){
	if(lines.size() == 0)
		return std::vector<Arc>();

	std::vector<Line> lineSegments[4];
	kdTree::PointContainer startPoints[4];
	generateLines(lines, lineSegments, startPoints);

	std::vector<Arc> arcs = extractArcs(lineSegments, startPoints);
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
		//TODO Line           BEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAM
	}




}
