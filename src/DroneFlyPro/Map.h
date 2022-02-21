#ifndef MMAP
#define MMAP
/*
For displaying debug information, please define the marco MMAP_TEST
*/
#include <vector>
#include <iostream>
#include <string>
#include <utility>
#include <set>
#include <algorithm>


#include "affection.h"
#include "shape.h"
#include "trajectory.h"

#include "model_tools.h"
#include "intersection_tools.h"


using objT = std::tuple <
	tinyobj::attrib_t, std::vector<tinyobj::shape_t>, std::vector<tinyobj::material_t>
>;


namespace map {
	/*
	 * For now, it's not a real-time system.
	 * We just work for designing the alogrithm by ground truth.
	 * The section of detecting buildings is omitted.
	 *
	 */
	class GTEnvironment
	{
	public:
		SurfaceMesh mesh;
		Tree tree;

		GTEnvironment() = delete;
		GTEnvironment(json args);
	};

	class Mapper
	{
	public:
		// Elements
		json args;
		objT obj;
		PointSet3 pts;
		SurfaceMesh mesh;

		// Member function
		Mapper() = delete;
		Mapper(json args);
	};

	//class objMapper {

	//protected:
	//	json m_args;
	//	objT obj;
	//	std::shared_ptr<std::vector<building>> shapes;
	//	size_t clusterNum;
	//	size_t demarcation;
	//	std::shared_ptr<std::vector<targetIndexSet>> targets;
	//	TJT::trajectory trajectory;
	//	
	//public:

	//	objMapper() = default;
	//	objMapper(const json& args, bool);

	//	void visualization(int begin, int end) {
	//		
	//		VER::objDrawFromMem(obj, begin, end);
	//	}
	//	
	//	void fullVis() {
	//		
	//		VER::dynamicDraw(obj, demarcation, 10);
	//	}	

	//	std::shared_ptr<std::vector<building>> getShapes() {
	//		return shapes;
	//	}
	//	std::shared_ptr<std::vector<targetIndexSet>> getTargets() {
	//		return targets;
	//	}

	//	bool buildingClustering();
	//	bool targetsGeneration();
	//	// it's wrong but can be used now!
	//	targetIndexSet nextTarget();

	//	TJT::trajectory trajectoryGeneration(targetIndexSet);
	//	
	//	void completeTraGen();

	//};

	//class parMapper :public objMapper {
	//private:
	//	objT gtobj;
	//public:
	//	parMapper(const json& args);

	//	void grVis();
	//	void GTvisualization(int begin, int end) {
	//		VER::objDrawFromMem(gtobj, begin, end);
	//	}
	//	void run();
	//};

};
#endif // !MMAP