#include <vector>
#include <numeric>

struct ring{
	float centerX;
	float centerY;
	float radius;
}

class ThomasAlgorithm{
public:
	static ring thomasAlgorithm(std::vector<float> x, std::vector<float> y);

}
