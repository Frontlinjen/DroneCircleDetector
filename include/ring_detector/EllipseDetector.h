#include <vector>
#include <opencv2/core/core.hpp>

struct Arc{
};

class EllipseDetector{
	typedef std::vector<cv::Vec4i> LineContainer;

public:
	static std::vector<Arc> detect(const LineContainer  lines);
};
