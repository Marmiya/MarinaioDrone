#include <iostream>
#include "common_util.h"
#include "Map.h"
#include "Drone.h"

using json = nlohmann::json;

int main(int argc, char* argv[])
{
	std::ifstream in(argv[1]);
	json args;
	in >> args;
	Log::LogSys log(argv[0], argv[1]);
	google::InitGoogleLogging(argv[0]);
	FLAGS_stderrthreshold = 0;

	GTMap map(args["GTModelPath"], args["samplePath"]);
	Drone drone(args["viewDistance"], args["safeDistance"], args["fov"], args["fov"], args["verticalOverlap"], args["horizontalOverlap"]);

	return 0;
}
