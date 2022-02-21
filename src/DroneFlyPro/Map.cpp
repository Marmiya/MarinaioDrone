#include "Map.h"
namespace map {

	GTEnvironment::GTEnvironment(json args)
	{
		
	}

	Mapper::Mapper(json args):args(args)
	{
		obj = modeltools::load_obj(args["objModelPath"]);
		CGAL::IO::read_point_set(args["plyModelPath"], pts);
		
	}
//
//	plyMapper::plyMapper(const json& args) :m_args(args) {
//		CGAL::Point_set_3<Point_3, Vector_3> pointCloud;
//		string modelPath{ args["plyModelPath"].get<string>() };
//		auto RPS = CGAL::IO::read_point_set(modelPath, pointCloud, pointCloud.parameters().use_binary_mode(true));
//
//		CGAL::Point_set_3<Point_3, Vector_3>::Property_map<int> cluster_map = pointCloud.add_property_map<int>("cluster", -1).first;
//
//		vector < pair<size_t, size_t>> adjacencies;
//
//		size_t nb_clusters = CGAL::cluster_point_set(pointCloud, cluster_map,
//			pointCloud.parameters().neighbor_radius(args["cluster_radius"].get<double>()).adjacencies(back_inserter(adjacencies)));
//
//		buildings.resize(nb_clusters);
//
//#ifdef MMAP_TEST
//		cout << "mMap.cpp: " << "The number of buildings is " << nb_clusters << endl;
//#endif // MMAP_TEST
//
//		CGAL::Point_set_3<Point_3, Vector_3>::Property_map<unsigned char>red = pointCloud.add_property_map<unsigned char>("red", 0).first;
//		CGAL::Point_set_3<Point_3, Vector_3>::Property_map<unsigned char>green = pointCloud.add_property_map<unsigned char>("green", 0).first;
//		CGAL::Point_set_3<Point_3, Vector_3>::Property_map<unsigned char>blue = pointCloud.add_property_map<unsigned char>("blue", 0).first;
//
//		for (CGAL::Point_set_3<Point_3, Vector_3>::Index idx : pointCloud) {
//			int clusterID = cluster_map[idx];
//			CGAL::Random rand(clusterID);
//			red[idx] = rand.get_int(64, 192);
//			green[idx] = rand.get_int(64, 192);
//			blue[idx] = rand.get_int(64, 192);
//
//			auto& currentBuilding = buildings.at(clusterID);
//			currentBuilding.points_world_space.insert(pointCloud.point(idx));
//		}
//
//		for (size_t i = 0; i < buildings.size(); ++i) {
//			buildings.at(i).bounding_box_3d =
//				get_bounding_box_rotated(buildings.at(i).points_world_space);
//			//cout << buildings.at(i).points_world_space.size() << endl;
//			buildings.at(i).boxes.push_back(buildings.at(i).bounding_box_3d);
//			buildings.at(i).viz_num = 99;
//		}
//
//	}
//
//	bool plyMapper::readjust()
//	{
//		for (auto& i : buildings) {
//			auto t = std::make_pair(i, false);
//			reBuildings.push_back(t);
//		}
//		return true;
//	}
//
//	bool plyMapper::nextTarget(std::pair<BDG::Building, bool>& n)
//	{
//		for (auto& i :reBuildings) {
//			if (!i.second) {
//				n = i;
//				return true;
//			}
//		}
//		return false;
//	}
//
//	objMapper::objMapper(const json& args, bool partial) :m_args(args)
//	{
//		string modelPath{ args["objModelPath"].get<string>() };
//		obj = MODELTOOLS::load_obj(modelPath, true, "");
//
//		if (partial) {
//			get<1>(obj).erase(get<1>(obj).begin() + 2);
//		}
//		auto vertices = get<0>(obj).vertices;
//		auto shape = get<1>(obj);
//		demarcation = shape.size();
//
//		std::vector<building> t;
//
//		for (int i = 0; i < shape.size(); i++) {
//
//			auto indice = shape.at(i).mesh.indices;
//			std::set<CGALTOOLS::Point3> pts;
//
//			for (auto j = indice.begin() + 1; j != indice.end() - 1; j++) {
//				double x, y, z;
//				x = vertices.at((*j).vertex_index * 3);
//				y = vertices.at((*j).vertex_index * 3 + 1);
//				z = vertices.at((*j).vertex_index * 3 + 2);
//				Point3 cur(x, y, z);
//				if (pts.find(cur) == pts.end()) {
//					pts.insert(cur);
//				}
//			}
//			
//			CGALTOOLS::cuboid cuboid(pts);
//
//#ifdef MMAP_TEST
//			cout << "--------------" << endl;
//			auto tt = cuboid.getVer();
//			for (const auto& i : tt) {
//				cout << i << endl;
//			}
//			cout << "--------------" << endl;
//#endif // MMAP_TEST
//
//			t.push_back(std::make_pair(SHP::shape(shape.at(i), 0), cuboid ));
//		}
//		shapes = make_shared<std::vector<building>>(t);
//
//#ifdef MMAP_TEST
//
//		std::printf("# of vertices  = %d\n", (int)(get<0>(obj).vertices.size()) / 3);
//		std::printf("# of normals   = %d\n", (int)(get<0>(obj).normals.size()) / 3);
//		std::printf("# of texcoords = %d\n", (int)(get<0>(obj).texcoords.size()) / 2);
//		std::printf("# of materials = %d\n", (int)(get<2>(obj).size()));
//		std::printf("# of shapes    = %d\n", (int)(get<1>(obj).size()));
//	
//		VER::objDrawFromMem(get<0>(obj), get<1>(obj), get<2>(obj), 0 , get<1>(obj).size());
//#endif // MMAP_TEST
//
//	}
//
//
//	bool objMapper::buildingClustering()
//	{
//		int targetID = 1;
//		for (auto i = shapes->begin(); i != shapes->end();i++) {
//			if (i->first.getTargetID() == 0) {
//				i->first.setTargetID(targetID);
//				
//				for (auto j = shapes->begin(); j != shapes->end(); j++) {
//					double dis = CGALTOOLS::squaredDistanceCuboid(i->second, j->second);
//#ifdef MMAP_TEST
//					cout << "The distance is " << dis << endl;
//#endif // MMAP_TEST
//					
//					if (dis < std::powf(2 * m_args["safe_distance"].get<double>(), 2)) {
//						if (j->first.getTargetID() == 0) {
//							j->first.setTargetID(targetID);
//						}
//						else {
//							i->first.setTargetID(j->first.getTargetID());
//						}
//					}
//				}
//			}
//			
//#ifdef MMAP_TEST
//			cout << "Finish cluster  " << targetID << endl;
//#endif // MMAP_TEST
//			targetID += 1;
//			int cl = 0;
//			for (auto k = shapes->begin(); k != shapes->end(); k++) {
//				if (k->first.getTargetID() != 0) {
//					cl += 1;
//				}
//			}
//			if (cl == shapes->size()) {
//				break;
//			}
//		}
//
//		clusterNum = static_cast<size_t> (targetID - 1);
//
//#ifdef MMAP_TEST
//		cout << "Targets' number: " << targets->size() << endl;
//		cout << "The details are below:" << endl;
//		for (const auto& i : *shapes) {
//			cout << i.first.getTargetID() << endl;
//		}
//		std::cout << std::endl;
//
//#endif // MMAP_TEST
//
//		return true;
//	}
//
//	bool objMapper::targetsGeneration()
//	{
//		targets = make_shared<std::vector<targetIndexSet>>();
//
//		for (size_t j = 1; j <= clusterNum; j++) {
//			targetIndexSet t;
//			for (size_t i = 0; i != shapes->size(); i++) {
//				if (shapes->at(i).first.getTargetID() == j) {
//					t.insert(i);
//				}
//			}
//			targets->push_back(t);
//		}
//		return true;
//	}
//
//	targetIndexSet objMapper::nextTarget()
//	{
//
//		return targets->front();
//	}
//
//
//	TJT::trajectory objMapper::trajectoryGeneration(targetIndexSet tis) {
//
//		using namespace TJT;
//
//		auto vertices = get<0>(obj).vertices;
//		CGALTOOLS::pointSet3 trajectory;
//		auto [ifkp, suc] = trajectory.add_property_map<bool>("ifKeyPoint", false);
//		CGALTOOLS::pointSet3 intermediate;
//		auto [ifk, succ] = intermediate.add_property_map<bool>("ifKeyPoint", false);
//		TJT::trajectory tjty;
//
//		 If the target is combined by only one building
//		if (tis.size() == 0) {
//			std::cout<< "empty target set!" << std::endl;
//		}
//		if (tis.size() == 1) {
//			auto bottomSegs = shapes->at(*tis.begin()).second.getBottomSegs();
//			
//			double height = shapes->at(*tis.begin()).second.getVer().at(4).z() -
//				bottomSegs->getSeg().vertex(0).z();
//			
//			double safeDis = m_args["safe_distance"].get<double>();
//
//			auto [expasion, length, width] = 
//				MODELTOOLS::safeExpansion(bottomSegs, safeDis);
//
//			double bottom = expasion->getSeg().vertex(0).z();
//
//			double
//				hFoV = m_args["horizontal_fov"].get<double>(),
//				vFoV = m_args["vertical_fov"].get<double>(),
//				hOverlap = m_args["horizontal_overlap"].get<double>(),
//				vOverlap = m_args["vertical_overlap"].get<double>();
//			 Intervals' distance
//			double
//				hInterval = (1 - hOverlap) * 2 * safeDis * std::tan((hFoV / 2) * (M_PI / 180.0)),
//				proportionOfLength = hInterval / length,
//				proportionOfWidth = hInterval / width,
//				vInterval = (1 - vOverlap) * safeDis * std::tan(vFoV * (M_PI / 180.0)),
//				ProportionOfV = vInterval / height;
//
//			size_t numLI = length / hInterval, numWI = width / hInterval, numVI = height / vInterval;
//
//			for (int i = 0; i < 4; i++) {
//				Vector3 side = expasion->getSeg().to_vector();
//				Point3 base = expasion->getSeg().vertex(0);
//				if (i == 0 || i == 2) {
//					intermediate.insert(base);
//					ifk[*(intermediate.end() - 1)] = true;
//					for (size_t li = 1; li <= numLI; li++) {
//						intermediate.insert(base + li * side * proportionOfLength);
//					}
//				}
//				else {
//					intermediate.insert(base);
//					ifk[*(intermediate.end() - 1)] = true;
//					for (size_t wi = 1; wi <= numWI; wi++) {
//						intermediate.insert(base + wi * side * proportionOfWidth);
//					}
//				}
//
//				expasion = expasion->getNext();
//			}
//
//			for (size_t vi = 1; vi <= numVI; vi++) {
//				double z = bottom + vi * ProportionOfV * height;
//				for (auto j = intermediate.begin(); j != intermediate.end(); j++) {
//					if (ifk[*j] == true) {
//						trajectory.insert(Point3(intermediate.point(*j).x(), intermediate.point(*j).y(), z));
//						ifkp[*(intermediate.end() - 1)] = true;
//					}
//					else {
//						trajectory.insert(Point3(intermediate.point(*j).x(), intermediate.point(*j).y(), z));
//					}
//				}
//			}
//			
//		}
//
//		else {
//			Point_set outerVerticesSequence;
//			std::vector<CGALTOOLS::segmentLN> shps;
//			double maxHeight = .0;
//			double groundHeight = shapes->at(0).second.getVer().at(0).z();
//
//			for (auto i : tis) {
//				double height = shapes->at(i).second.getVer().at(4).z() - shapes->at(i).second.getVer().at(0).z();
//				if (height > maxHeight) {
//					maxHeight = height;
//				}
//				shps.push_back(*(shapes->at(i).second.getBottomSegs()));
//			}
//			
//			auto [profile, sideSize] = CGALTOOLS::makeProfile(shps);
//			auto [profile, it] = CGALTOOLS::makeProfileInterval(shps);
//			auto profile = MODELTOOLS::safeExpansionAny(shps, m_args["safe_distance"].get<double>());
//			
//			double safeDis = m_args["safe_distance"];
//			
//			double
//				hFoV = m_args["horizontal_fov"].get<double>(),
//				vFoV = m_args["vertical_fov"].get<double>(),
//				hOverlap = m_args["horizontal_overlap"].get<double>(),
//				vOverlap = m_args["vertical_overlap"].get<double>();
//			double
//				hInterval = (1 - hOverlap) * 2 * safeDis * std::tan((hFoV / 2) * (M_PI / 180.0)),
//				vInterval = (1 - vOverlap) * safeDis * std::tan(vFoV * (M_PI / 180.0)),
//				ProportionOfV = vInterval / maxHeight, numVI = maxHeight / vInterval;
//
//
//			for (auto p = profile.edges_begin(); p != profile.edges_end(); p++) {
//				double edgeLength = std::sqrt(p->squared_length());
//				double proportionOfLength = hInterval / edgeLength;
//				size_t numLI = edgeLength / hInterval;
//				Point3 base(p->vertex(0).x(), p->vertex(0).y(), groundHeight);
//				for (int i = 0; i <= numLI; i++) {
//					intermediate.insert(
//						base + i * proportionOfLength * Vector3(p->to_vector().x(), p->to_vector().y(), groundHeight));
//				}
//			}
//			for (size_t vi = 1; vi <= numVI; vi++) {
//				double z = groundHeight + vi * ProportionOfV * maxHeight;
//				for (size_t j = 0; j != intermediate.size(); j++) {
//					trajectory.insert(Point3(intermediate.point(j).x(), intermediate.point(j).y(), z));
//				}
//			}
//		}
//		tjty.setpts(trajectory);
//		return tjty;
//	}
//
//	void objMapper::completeTraGen()
//	{
//		for (auto i : *targets) {
//			
//			trajectory.join(trajectoryGeneration(i));
//		}
//		
//	}
//		
//
//	parMapper::parMapper(const json& args) 
//	{
//		m_args = args;
//		obj = MODELTOOLS::load_obj(m_args["objModelPath"], true, "");
//		gtobj = obj;
//
//		double 
//			gsX = m_args["groundStartX"],
//			gsY = m_args["groundStartY"],
//			geX = m_args["groundEndX"],
//			geY = m_args["groundEndY"],
//			gZ = m_args["groundZ"];
//
//		size_t numX = (geX - gsX) / 50.0,
//			numY = (geY - gsY) / 50.0;
//
//		auto& gvertices = get<0>(gtobj).vertices;
//		auto& gshapes = get<1>(gtobj);
//		size_t originalSize = gvertices.size() / 3;
//		for (size_t i = 0; i <= numX; i++) {
//			for (size_t j = 0; j <= numY; j++) {
//				gvertices.push_back(gsX + i * 50);
//				gvertices.push_back(gsY + j * 50);
//				gvertices.push_back(gZ);
//			}
//		}
//		tinyobj::shape_t newShape;
//		for (size_t i = 0; i < numX; i++) {
//			for (size_t j = 0; j < numY; j++) {				
//				tinyobj::index_t f, s, t;
//				f.normal_index = -1;
//				f.texcoord_index = -1;
//				s.normal_index = -1;
//				s.texcoord_index = -1;
//				t.normal_index = -1;
//				t.texcoord_index = -1;
//				f.vertex_index = originalSize + (i + 1) * (numY + 1) + j + 1;
//				s.vertex_index = originalSize + i * (numY + 1) + j + 1;
//				t.vertex_index = originalSize + (i + 1) * (numY + 1) + j;
//				newShape.mesh.indices.push_back(f);
//				newShape.mesh.indices.push_back(s);
//				newShape.mesh.indices.push_back(t);
//				newShape.mesh.num_face_vertices.push_back(3);
//				newShape.mesh.material_ids.push_back(-1);
//				f.vertex_index = originalSize + i * (numY + 1) + j;
//				s.vertex_index = originalSize + i * (numY + 1) + j + 1;
//				t.vertex_index = originalSize + (i + 1) * (numY + 1) + j;
//				newShape.mesh.indices.push_back(f);
//				newShape.mesh.indices.push_back(s);
//				newShape.mesh.indices.push_back(t);
//				newShape.mesh.num_face_vertices.push_back(3);
//				newShape.mesh.material_ids.push_back(-1);
//			}
//		}
//		gshapes.push_back(newShape);
//		demarcation = gshapes.size();
//
//		get<1>(obj).erase(get<1>(obj).begin());
//
//		auto vertices = get<0>(obj).vertices;
//		auto shape = get<1>(obj);
//
//		std::vector<building> t;
//
//		for (int i = 0; i < shape.size(); i++) {
//
//			auto indice = shape.at(i).mesh.indices;
//			std::set<CGALTOOLS::Point3> pts;
//
//			for (auto j = indice.begin() + 1; j != indice.end() - 1; j++) {
//				double x, y, z;
//				x = vertices.at((*j).vertex_index * 3);
//				y = vertices.at((*j).vertex_index * 3 + 1);
//				z = vertices.at((*j).vertex_index * 3 + 2);
//				Point3 cur(x, y, z);
//				if (pts.find(cur) == pts.end()) {
//					pts.insert(cur);
//				}
//			}
//
//			CGALTOOLS::cuboid cuboid(pts);
//
//#ifdef MMAP_TEST
//			cout << "--------------" << endl;
//			auto tt = cuboid.getVer();
//			for (const auto& i : tt) {
//				cout << i << endl;
//			}
//			cout << "--------------" << endl;
//#endif // MMAP_TEST
//
//			t.push_back(std::make_pair(SHP::shape(shape.at(i), 0), cuboid));
//		}
//		shapes = make_shared<std::vector<building>>(t);
//	}
//
//	void parMapper::grVis()
//	{
//		auto tobj = gtobj;
//		for (const auto& i : trajectory) {
//			CGALTOOLS::pointVisualization(tobj, trajectory.point(i), m_args["cameraRadius"]);
//		}
//		VER::dynamicDraw(tobj, demarcation, 10);
//	}
//
//	void parMapper::run()
//	{
//		
//		double 
//			positionX = m_args["startX"],
//			positionY = m_args["startY"],
//			positionZ = m_args["startZ"],
//			cameraX = m_args["cameraX"],
//			cameraY = m_args["cameraY"],
//			cameraZ = m_args["cameraZ"];
//
//		double viewDis = m_args["viewDistance"];
//		double safeDis = m_args["safe_distance"];
//		bool exploating = true;
//		bool reconstructing = false;
//		bool finished = false;
//
//		Point3 UAVPosition(cameraX, cameraY, cameraZ);
//		targetIndexSet target;
//		Point_set estimatedTrajectoty;
//
//		 Get gt sides. Just for experiments
//		std::vector<CGALTOOLS::cuboid> gtcuboids;
//		for (const auto& i : *shapes) {
//			gtcuboids.push_back(i.second);
//		}
//		auto gtSides = CGALTOOLS::getsides(gtcuboids);
//
//		while (!finished) {
//
//			while (exploating) {
//
//				buildingClustering();
//				targetsGeneration();
//				target = nextTarget();
//				estimatedTrajectoty = trajectoryGeneration(target);
//				trajectory.join(MODELTOOLS::CCPP(UAVPosition, estimatedTrajectoty.point(0)));
//				exploating = false;
//				reconstructing = true;
//			}
//
//			while (reconstructing) {
//				std::vector<CGALTOOLS::cuboid> cuboids;
//				for (const auto& i : target) {
//					cuboids.push_back(shapes->at(i).second);
//				}
//				auto sides = CGALTOOLS::getsides(cuboids);
//				for (auto i = estimatedTrajectoty.begin(); i != estimatedTrajectoty.end(); i++) {
//					UAVPosition = estimatedTrajectoty.point(*i);
//					auto nextPosition = estimatedTrajectoty.point(*(i + 1));
//					
//					 Collision detection
//					CGALTOOLS::segment view(UAVPosition, nextPosition);
//					Vector3 vec = viewDis * (view.direction().to_vector() / std::sqrt(view.squared_length()));
//					
//					Eigen::AngleAxisd rightRotation(M_PI * (11.0 / 6.0), Eigen::Vector3d(0, 0, 1));
//					Eigen::AngleAxisd leftRotation(M_PI * (1.0 / 6.0), Eigen::Vector3d(0, 0, 1));
//					Eigen::Vector3d viewEigen(vec.x(), vec.y(), vec.z());
//					Eigen::Vector3d rightViewEigen = rightRotation * viewEigen;
//					Eigen::Vector3d leftViewEigen = leftRotation * viewEigen;
//
//					view = CGALTOOLS::segment(UAVPosition, UAVPosition + vec);
//					CGALTOOLS::segment rightView(UAVPosition, UAVPosition + Vector3(rightViewEigen.x(), rightViewEigen.y(), rightViewEigen.z()));
//					CGALTOOLS::segment leftView(UAVPosition, UAVPosition + Vector3(leftViewEigen.x(), leftViewEigen.y(), leftViewEigen.z()));
//					cout << "View length: " << view.squared_length() << " " << leftView.squared_length() << " " << rightView.squared_length() << endl;
//					
//					bool leftC = false, rightC = false, midC = false;
//
//					for (const auto& i : gtSides) {
//						double height = i.first->getLast()->getSeg().vertex(0).z();
//						if (view.vertex(1).z() < height) {
//							auto leftCollosion = CGAL::intersection(leftView, i.second);
//							if (leftCollosion) {
//								leftC = true;
//							}
//							auto rightCollosion = CGAL::intersection(rightView, i.second);
//							if (rightCollosion) {
//								rightC = true;
//							}
//							auto midCollosion = CGAL::intersection(view, i.second);
//							if (midCollosion) {
//								midC = true;
//							}
//						}
//					}
//					 Collision detection End
//					 
//					 If no collision
//					if (!leftC && !rightC && !midC) {
//						Eigen::AngleAxisd constructingRotation(M_PI_2, Eigen::Vector3d(0, 0, 1));
//						Eigen::Vector3d constructingViewEigen = constructingRotation * viewEigen;
//						CGALTOOLS::segment constructingView(
//							UAVPosition, UAVPosition + Vector3(constructingViewEigen.x(), constructingViewEigen.y(), constructingViewEigen.z()));
//
//						Point3 nearestIstn;
//						double nearestDis = 100.0;
//						for (const auto& i : gtSides) {
//							double height = i.first->getLast()->getSeg().vertex(0).z();
//							if (UAVPosition.z() < height) {
//								auto constructingCollosion = CGAL::intersection(constructingView, i.second);
//								if (constructingCollosion) {
//									Point3 pt = *(boost::get<Point3>(&*constructingCollosion));
//									double dis = std::sqrt(CGAL::squared_distance(pt, UAVPosition));
//									if (dis < nearestDis) {
//										nearestDis = dis;
//										nearestIstn = pt;
//									}
//								}
//							}
//						}
//						if (nearestDis > 2 * safeDis) {
//
//						}
//						else if (nearestDis > safeDis) {
//
//						}
//						else {
//
//						}
//					}
//					else {
//
//					}
//					
//					
//
//
//				}
//			}
//		}
//	}

};