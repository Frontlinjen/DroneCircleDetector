#include "ring_detector/EllipseDetector.h"
#include <cmath>
#include "ring_detector/ThomasAlgorithm.h"
#include <opencv2/imgproc.hpp>
#define TO_DEG(x) (x*(180/M_PI))

bool TestLineBeam(Arc& first, Arc& third, Ellipse& el){
	const int maxDist = 10;
	Vec2 mi;
	mi.x = 0,5 * (third.lines->back().mid.x - first.lines->at(0).mid.x);
	mi.y = 0,5 * (third.lines->back().mid.y - first.lines->at(0).mid.y);

	Line firstLine, lastLine;

	firstLine = first.lines->at(0);
	lastLine = third.lines->back();

	float slopeFirst = tan(firstLine.angle);
	float bFirst = firstLine.end.y - slopeFirst * firstLine.end.x;

	float slopeLast = tan(lastLine.angle);
	float bLast = lastLine.end.y - slopeLast * lastLine.end.x;

	Vec2 ti;
	ti.x = (bLast - bFirst) / (slopeFirst - slopeLast);
	ti.y = slopeFirst * ti.x + bFirst;

	float sloapBeam = (ti.y - mi.y) / (ti.x - mi.x);
	float bBeam = ti.y - sloapBeam * ti.x;

	float temp1, temp2;
	temp1 = abs(sloapBeam * el.centerX + bBeam - el.centerY);
	temp2 = sqrt(pow(sloapBeam, 2) + 1);

	float dist = temp1 / temp2;

	return dist <= maxDist;
}

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
	return true;
}

bool TestTangentError(const Arc& first, const Arc& second, const Arc& third, Ellipse& el){
	const int maxAngle = 10;
	const std::vector<Line>* lines[3];
	lines[0] = first.lines;
	lines[1] = second.lines;
	lines[2] = third.lines;

	//Collect all points
	for(size_t i = 0; i < 3 ; ++i){
		for(std::vector<Line>::const_iterator itr1 = lines[i]->begin(); itr1 != lines[i]->end(); ++itr1){
			float estAngle = atan2((-pow(el.b,2) * itr1->mid.x), (pow(el.a, 2) * itr1->mid.y)) * 180 / M_PI;
			float vecAngle = itr1->angle;
			if(vecAngle - estAngle >= maxAngle){
				return false;
			}
		}
	}
	return true;
}

Ellipse fitEllipse(const Arc& first, const Arc& second, const Arc& third){
	std::vector<cv::Point> points;
	const Arc* arcs[3];
	arcs[0] = &first;
	arcs[1] = &second;
	arcs[2] = &third;
	for(uint8_t i = 0; i < 3; ++i){
		for(std::vector<Line>::const_iterator itr = arcs[i]->lines->begin();itr != arcs[i]->lines->end(); ++itr){
			points.emplace_back(itr->start.x, itr->start.y);
			points.emplace_back(itr->mid.x, itr->mid.y);
			points.emplace_back(itr->end.x, itr->end.y);
		}
	}
	cv::RotatedRect rBox = cv::fitEllipse(points);
	Ellipse ell;
	ell.centerX = rBox.center.x;
	ell.centerY = rBox.center.y;
	ell.a = rBox.size.width / 2;
	ell.b = rBox.size.height / 2;
	ell.rotation = rBox.angle;

	return ell;
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

std::vector<Arc>* EllipseDetector::extractArcs(std::vector<Line> (& lineSegments)[4], kdTree::PointContainer (& startPoints)[4]){
		kdTree tree;
		const float errLine = 45.0f;
		const int maxDistance = 20;
		std::vector<Arc>* arcs = new std::vector<Arc>[4];
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
						arcs[i].push_back(a);
					}
				}
			}
		}
		return arcs;
}

std::vector<ExtendedArc> EllipseDetector::extractExtendedArcs(std::vector<Arc>* arcs){
	const float tolerance = 0.0f;
	const float mGap = 0.0f;
	std::vector<ExtendedArc> extArcs;
	for(uint8_t i=0;i < 4;++i){
		std::vector<Arc>* preArcs = &arcs[(i - 1 + 4)%4];
		std::vector<Arc>* currentArcs = &arcs[i];
		std::vector<Arc>* postArcs = &arcs[(i + 1 + 4)%4];
		for(std::vector<Arc>::iterator current = currentArcs->begin(); current != currentArcs->end(); ++current){
			while(true){
				std::vector<Arc>::iterator previous = preArcs->begin();
				std::vector<Arc>::iterator next = postArcs->begin();

				//TODO: Investigate if this is correct way of checking it
				if(previous->absoluteDistance(*current) < tolerance && next->absoluteDistance(*current) < tolerance){
					if(previous->relativeDistance(*current) < tolerance && next->relativeDistance(*current) < tolerance){
						float AB[2];
						float BC[2];
						previous->gapAngle(*current, AB);
						next->gapAngle(*current, BC);
							if(AB[0] < mGap && AB[1] < mGap && BC[0] < mGap && BC[1] < mGap ){
								if(TestInnerAngles(*previous, *current, *next)){
									//Estimating ellipse..
									Ellipse ellipse = fitEllipse(*previous, *current, *next);
									if(TestTangentError(*previous, *current, *next, ellipse)){
										if(TestLineBeam(*previous, *next, ellipse)){
											ExtendedArc newArc;
											newArc.a = ellipse.a;
											newArc.b = ellipse.b;
											newArc.alpha = ellipse.rotation;
											newArc.position = { ellipse.centerX, ellipse.centerY };
											newArc.arcs[0] = &(*previous);
											newArc.arcs[1] = &(*current);
											newArc.arcs[2] = &(*next);
											extArcs.push_back(newArc);
										}
									}
								}
							}
					}
				}
			}
		}
	}
	return extArcs;
}

std::vector<Arc> EllipseDetector::detect(const LineContainer lines){
	if(lines.size() == 0)
		return std::vector<Arc>();

	std::vector<Line> lineSegments[4];
	kdTree::PointContainer startPoints[4];
	generateLines(lines, lineSegments, startPoints);

	std::vector<Arc>* arcs = extractArcs(lineSegments, startPoints);


	//Compute extended arcs:
	for(std::vector<Arc>::iterator startArc = arcs->begin(); startArc != arcs->end(); ++startArc){
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


