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

bool TestTangentError(Arc** const first, const size_t arcs, Ellipse el){
	const int maxAngle = 10;
	const std::vector<Line>* lines[arcs];
	for(size_t i = 0 ; i < arcs ; ++i){
		lines[i] = (*first[i]).lines;
	}

	//Collect all points
	for(size_t i = 0; i < arcs ; ++i){
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

Ellipse fitEllipse(Arc** const first, const size_t arcs){
	std::vector<cv::Point> points;
	const std::vector<Line>* lines[arcs];
	for(size_t i = 0; i < arcs; ++i){
		lines[i] = (*first[i]).lines;
	}
	for(uint8_t i = 0; i < 3; ++i){
		for(std::vector<Line>::const_iterator itr = first[i]->lines->begin();itr != first[i]->lines->end(); ++itr){
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
									Arc* currentArcs[3];
									currentArcs[0] = &(*previous);
									currentArcs[1] = &(*current);
									currentArcs[2] = &(*next);
									//Estimating ellipse..
									Ellipse ellipse = fitEllipse(currentArcs, 3);
									if(TestTangentError(currentArcs, 3, ellipse)){
										if(TestLineBeam(*previous, *next, ellipse)){
											ExtendedArc newArc;
											newArc.a = ellipse.a;
											newArc.b = ellipse.b;
											newArc.alpha = ellipse.rotation;
											newArc.position = { ellipse.centerX, ellipse.centerY };
											newArc.arcs.push_back(&(*previous));
											newArc.arcs.push_back(&(*current));
											newArc.arcs.push_back(&(*next));
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

bool EllipseDetector::canBeMerged(std::vector<Arc*> arcs){
		const int max = 5;
		Ellipse e = fitEllipse(arcs.data(), arcs.size());
		if(!TestTangentError(arcs.data(), arcs.size(), e) || !TestLineBeam(*(arcs.front()), *(arcs.back()), e)){
			return false;
		}
		else {
			for(std::vector<Arc*>::iterator itr2 = arcs.begin(); itr2 != arcs.end() ; ++itr2){
				for(std::vector<Line>::iterator line = (*itr2)->lines->begin(); line != (*itr2)->lines->end(); ++itr2){
					float value, value2;
					value = pow((line->start.x / e.a), 2) + pow((line->start.y / e.b), 2) - 1;
					value2 = pow((line->end.x / e.a), 2) + pow((line->end.y / e.b), 2) - 1;
					if(value > max || value2 > max){
						return false;
					}
				}
			}
		}
	return true;
}

std::vector<Ellipse> EllipseDetector::detect(const LineContainer lines){
	if(lines.size() == 0)
		return std::vector<Ellipse>();

	std::vector<Line> lineSegments[4];
	kdTree::PointContainer startPoints[4];
	generateLines(lines, lineSegments, startPoints);


	std::vector<Arc>* arcs = extractArcs(lineSegments, startPoints);

	std::vector<ExtendedArc> extendedArcs = extractExtendedArcs(arcs);

	std::vector<ExtendedArc> almostEllipse;

	//Find ellipses....
	for(std::vector<ExtendedArc>::iterator itr = extendedArcs.begin(); itr != extendedArcs.end() ; ++itr){
		std::vector<int> indices;
		std::vector<Arc*>::iterator resetPos = itr->arcs.end();
		for(std::vector<ExtendedArc>::iterator itr2 = itr + 1; itr2 != extendedArcs.end() ; ++itr2){
				if(itr->arcs.back() == itr2->arcs.front()){
					itr->arcs.push_back(itr2->arcs[1]);
					itr->arcs.push_back(itr2->arcs[2]);
				}else if(itr->arcs.back() == itr2->arcs[1]){
					itr->arcs.push_back(itr2->arcs[2]);
				}
				if(resetPos != itr->arcs.end()){
					if(!canBeMerged(itr->arcs)){
						itr->arcs.erase(resetPos, itr->arcs.end());
					}
					else
					{
						extendedArcs.erase(itr2);
					}
				}
		}
		if(itr->arcs.size() > 3){
			almostEllipse.push_back(*itr);
			extendedArcs.erase(itr);
		}
	}
	for(std::vector<ExtendedArc>::iterator itr = extendedArcs.begin(); itr != extendedArcs.end() ; ++itr){
		for(std::vector<ExtendedArc>::iterator itr2 = almostEllipse.begin(); itr2 != almostEllipse.end() ; ++itr2){
			std::vector<Arc*>::iterator resetPos = itr2->arcs.end();
			itr2->arcs.insert(itr2->arcs.begin(), itr->arcs.begin(), itr->arcs.end());
			if(!canBeMerged(itr2->arcs)){
				itr2->arcs.erase(resetPos, itr2->arcs.end());
			}else
			{
				extendedArcs.erase(itr);
			}
		}
	}

	for(std::vector<ExtendedArc>::iterator itr = extendedArcs.begin(); itr != extendedArcs.end() ; ++itr){
		for(std::vector<ExtendedArc>::iterator itr2 = itr+1; itr2 != extendedArcs.end() ; ++itr2){
			float centerCheck, axis1Check, axis2Check;
			centerCheck = sqrt(pow((itr->position.x - itr2->position.x), 2) + pow((itr->position.y - itr->position.y), 2));
			axis1Check = itr->a / itr2->a;
			axis2Check = itr->b / itr2->b;

			if(centerCheck < 2 && axis1Check > 0,75 && axis2Check > 0,75){
				itr->arcs.insert(itr->arcs.end(), itr2->arcs.begin(), itr2->arcs.end());
				almostEllipse.push_back(*itr);
				extendedArcs.erase(itr);
				extendedArcs.erase(itr2);
			}
		}
	}
	const float circumfanceTolerance = 100;
	std::vector<Ellipse> ellipses;
	for(std::vector<ExtendedArc>::iterator itr = almostEllipse.begin(); itr != almostEllipse.end() ; ++itr){
		Ellipse e = fitEllipse(itr->arcs.data(), itr->arcs.size());
		float cj = M_PI*(1.5*(e.a + e.b) - sqrt(e.a*e.b));
		if(cj > circumfanceTolerance){
			ellipses.push_back(e);
		}
	}
	return ellipses;
}


