#include "planning-utils/libs/cxxopts.hpp"
#include "planning-utils/utils.hpp"
#include "planning-utils/display.hpp"

void display()
{
  //TODO well duh, put some code here
  drawLine(Coord(500.0,500.0), Coord(400.0,400.0), 5, HSL(200, 1.0, 0.5));
}

int main(int argc, char* argv[])
{
  // admittedly copying a lot of Bryant's code to get it to work

  int width = 700;
	int height = 700;
	bool isFullscreen = false;
	int monitorNum = 0;
	bool useFmt = false;
	bool usePseudoRandom = false;
	double replanFrequency = -1;

  // clang-format off
	cxxopts::Options options("myRRT", "A cool program for cool things");
	options.add_options()
		("f,fullscreen", "Enable Fullscreen", cxxopts::value(isFullscreen))
		("m,monitor", "Set Monitor Number", cxxopts::value(monitorNum))
		("fmt", "Use FMT*", cxxopts::value(useFmt))
		("p,pr", "Use pseudo-random numbers", cxxopts::value(usePseudoRandom))
		("r,replan", "Replan frequency", cxxopts::value(replanFrequency));
	// clang-format on

	options.parse(argc, argv);

  /*GLFWwindow**/auto window = initWindow(isFullscreen, monitorNum, width, height);

  float ratio;
	ratio = width / (float)height;
	initDisplay(width, height, ratio);

  auto displayCallback = []() { display(); };

  auto remainderCallback = [](){return;};
  /*[&planner, &waldos, &replanInterval, &moveInterval, &lastReplan, &lastMove]() {
		auto currentTime = glfwGetTime();
		if (currentTime - lastMove >= moveInterval) {
			lastMove = currentTime;

			for (const auto& waldo : waldos) {
				waldo->followPath();
			}

			planner->followPath();
		} else if (replanInterval != -1 && currentTime - lastReplan >= replanInterval) {
			lastReplan = currentTime;
			planner->randomReplan();
		} else {
			planner->sample();
		}
	};*/

	displayLoop(window, 30.0, displayCallback, remainderCallback);
}
