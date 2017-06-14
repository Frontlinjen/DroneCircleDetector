#include "ring_detector/kdtree.h"
#include <algorithm>

bool compareY(const kdLeaf& first, const kdLeaf& second){
  return first.p.y < second.p.y;
}

bool compareX(const kdLeaf& first, const kdLeaf& second){
  return first.p.x < second.p.x;
}



void kdTree::buildTree(PointContainer &points){
  root =  visitNode(points.data(), points.size(), 0);
}

kdNode* kdTree::visitNode(kdLeaf* points, std::size_t length, unsigned int depth){
  kdNode* newNode = new kdNode();
  newNode->type = kdNode::BRANCH;
  if(length == 1){
	kdLeaf& p = *points;
    newNode->type = kdNode::LEAF;
    newNode->leaf = p;
  } else if(depth%2==0){
    unsigned int index = estimateMedian(points, length, compareX);//Split according to X
    unsigned int median = points[index].p.x;
    newNode->branch.location = median;
    newNode->branch.left = visitNode(points, index+1, depth+1);
    newNode->branch.right = visitNode(points+index+1, length-index-1, depth+1);
  }
  else{
    unsigned int index = estimateMedian(points, length, compareY);//Split according to X
    unsigned int median = points[index].p.y;
    newNode->branch.location = median;
    newNode->branch.left = visitNode(points, index+1, depth+1);
    newNode->branch.right = visitNode(points+index+1, length-index-1, depth+1);
  }
  return newNode;
}

std::vector<kdLeaf> kdTree::InRange(Rect& rect){
  std::vector<kdLeaf> points;
  Rect rootRect;
  rootRect.center.x = static_cast<unsigned int>(-1)*0.5f;
  rootRect.center.y = static_cast<unsigned int>(-1)*0.5f;
  rootRect.width = static_cast<unsigned int>(-1)*0.5f;
  rootRect.height = static_cast<unsigned int>(-1)*0.5f;
  searchNode<true>(root, rootRect, rect, points);
  return points;
}

template<bool X>
inline Rect CalculateLeftBoundingBox(Rect& rect, uint32_t slice);


template<bool X>
inline Rect CalculateRightBoundingBox(Rect& rect, uint32_t slice);

template<>
inline Rect CalculateLeftBoundingBox<true>(Rect& rect, uint32_t slice){
	Rect childRegion = rect;
	const uint32_t startPos = rect.center.x - rect.width;
	const uint32_t endpos = slice;
	childRegion.width = std::ceil((endpos - startPos)*0.5f);
	childRegion.center.x = startPos + childRegion.width;
	return childRegion;
}

template<>
inline Rect CalculateRightBoundingBox<true>(Rect& rect, uint32_t slice){
	Rect childRegion = rect;
	const uint32_t startPos = slice;
	const uint32_t endpos = rect.center.x + rect.width;
	childRegion.width = std::ceil((endpos - startPos)*0.5f);
	childRegion.center.x = startPos + childRegion.width;
	return childRegion;
}

template<>
inline Rect CalculateLeftBoundingBox<false>(Rect& rect, uint32_t slice){
	Rect childRegion = rect;
	const uint32_t startPos = rect.center.y - rect.height;
	const uint32_t endpos = slice;
	childRegion.height = std::ceil((endpos - startPos)*0.5f);
	childRegion.center.y = startPos + childRegion.height;
	return childRegion;
}


template<>
inline Rect CalculateRightBoundingBox<false>(Rect& rect, uint32_t slice){
	Rect childRegion = rect;
	const uint32_t startPos = slice;
	const uint32_t endpos = rect.center.y + rect.height;
	childRegion.height = std::ceil((endpos - startPos)*0.5f);
	childRegion.center.y = startPos + childRegion.height;
	return childRegion;
}

template<bool splitX>
void kdTree::searchNode(kdNode* node,Rect& parent, Rect& rect, std::vector<kdLeaf>& locatedPoints){
  if(node->type == kdNode::LEAF){
    if(rect.contains(node->leaf.p)){
      locatedPoints.push_back(node->leaf);//add it to the list
    }
  }
  else{
    const uint32_t newOffset = node->branch.location;
    if(node->branch.left != NULL){
      if(node->branch.left->type == kdNode::BRANCH){
		Rect childRegion = CalculateLeftBoundingBox<splitX>(parent, newOffset);
		const int result = childRegion.intersects(rect);
		if(result == 1){
		  collectPoints(node->branch.left, locatedPoints);
		}
		else if(result == 0){
		  searchNode<!splitX>(node->branch.left, childRegion, rect, locatedPoints);
		}
      }
      else{
		if(rect.contains(node->branch.left->leaf.p)){
		   locatedPoints.push_back(node->leaf);//add it to the list
		}
      }
    }
    if(node->branch.right != NULL){
      if(node->branch.right->type == kdNode::BRANCH){
    	Rect childRegion = CalculateRightBoundingBox<splitX>(parent, newOffset);
		const int result = childRegion.intersects(rect);
		if(result == 1){
		  collectPoints(node->branch.right, locatedPoints);
		}
		else if(result == 0){
		  searchNode<!splitX>(node->branch.right, childRegion, rect, locatedPoints);
		}
      }
      else{
    	  if(rect.contains(node->branch.right->leaf.p)){
    	     locatedPoints.push_back(node->leaf);//add it to the list
    	  }
      }
    }
  }
}

void kdTree::collectPoints(kdNode* node, std::vector<kdLeaf>& locatedPoints){
  if(node->type==kdNode::LEAF){
    locatedPoints.push_back(node->leaf);
  }
  else{
    if(node->branch.left != NULL){
      collectPoints(node->branch.left, locatedPoints);
    }
    if(node->branch.right != NULL){
      collectPoints(node->branch.right, locatedPoints);
    }
  }
}

//Sorts the vector according to comparator and returns median index
unsigned int kdTree::estimateMedian(kdLeaf* points, std::size_t length, Compare comparetor){
  const int index = length - 1;
  std::sort(points, &points[index], comparetor);
  return static_cast<unsigned int>(index*0.5f);

}
