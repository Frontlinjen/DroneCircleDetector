#include <vector>
#include <cstdint>
#include <cmath>

struct kdNode;

struct kdPoint{
	unsigned int x;
	unsigned int y;
};

struct kdLeaf{
	kdPoint p;
	void * userdata;
};

struct kdBranch{
	unsigned int location;
	kdNode * left;
	kdNode * right;
};

struct Rect{
	kdPoint center;
	//Half
	uint32_t height;
	//Half
	uint32_t width;

	bool contains(kdPoint& p){
		return std::abs(static_cast<long>(p.x) - center.x) <= width && std::abs(static_cast<long>(p.y) - center.y) <= height;
	}

	int intersects(Rect& other){
		const uint32_t distanceX = std::abs(static_cast<long>(center.x) - other.center.x);
		const uint32_t distanceY = std::abs(static_cast<long>(center.y) - other.center.y);
		if((distanceX > other.width && distanceX > width) || (distanceY > other.height && distanceY > height)){
			return -1; //Don't intercept
		}
		else if((distanceX + width) <= other.width && (distanceY + height) <= other.height){
			return 1; //this is contained within "other"
		}
		else{
			return 0;
		}
	}
};

struct kdNode{
	enum{
		BRANCH,
		LEAF
	} type;
	union{
		kdLeaf leaf;
		kdBranch branch;
	};
};

class kdTree{
	kdNode * root;

public:
	//0 = X-axis, 1 = Y-axis
	typedef std::vector<kdLeaf> PointContainer;
	std::vector<kdLeaf> InRange(Rect& rect);
	void buildTree(PointContainer& points);
	const kdNode* getRoot(){ return root; };
private:
	typedef bool (*Compare)(const kdLeaf& first, const kdLeaf& second);
	kdNode* visitNode(kdLeaf* points, std::size_t length, unsigned int depth);
	unsigned int estimateMedian(kdLeaf* points, std::size_t length, Compare comparetor);
	template<bool splitX>
	void searchNode(kdNode* node, Rect& parent, Rect& rect, std::vector<kdLeaf>& locatedPoints);
	void searchNode(kdNode* node, Rect& parent, Rect& rect, std::vector<kdLeaf>& locatedPoints);

	void collectPoints(kdNode* node, std::vector<kdLeaf>& locatedPoints);
};
