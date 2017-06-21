#include <chrono>
#include <utility>

#define MEASURE_TIME 1

#ifdef MEASURE_TIME
#define BEGIN_SCOPE_MEASURE(name)  AutoTimeMeasure mTime(name)
#else
#define BEGIN_SCOPE_MEASURE(name) do {} while(0)
#endif

class AutoTimeMeasure{
	const char * name;
	std::chrono::high_resolution_clock::time_point start;
public:
	AutoTimeMeasure(const char * c)
{
		name = c;
		start = std::chrono::high_resolution_clock::now();
}
	~AutoTimeMeasure(){
		std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
		printf("%s finished in %fs\n", name, std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()/1000000000.0);
	}
};
