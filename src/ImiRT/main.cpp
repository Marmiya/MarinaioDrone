#include <iostream>
#include "common_util.h"

int main(int argc, char* argv[])
{
	Log::LogSys log(argv[0], argv[1]);
	google::InitGoogleLogging(argv[0]);
	FLAGS_stderrthreshold = 0;
	
	return 0;
}
