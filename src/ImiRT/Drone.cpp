#include "Drone.h"

Drone::Drone(
	const double& viewDis, const double& safeDis,
	const double& vfov, const double& hfov,
	const double& vOverlap, const double& hOverlap) :
	viewDis(viewDis), safeDis(safeDis), vOverlap(vOverlap), hOverlap(hOverlap)
{
	vFov = (vfov * M_PI) / 180.;
	hFov = (hfov * M_PI) / 180.;
}

// Arg: Point cloud density
bool Drone::SASGen(const double& ptsD)
{
	double gap = ptsD / viewDis;
	int vPtsA = std::floor(vFov / gap) + 2;
	int hPtsA = std::floor(hFov / gap) + 2;

	double hbase = - vFov / 2;
	double vbase = 0;
	
	for (int i = 0; i <= vPtsA; i++)
	{
		for (int j = 0; j <= hPtsA; j++)
		{
			sightAngleSet.push_back(std::make_pair(vbase * i * gap, hbase * j * gap));
		}
		sightAngleSet.push_back(std::make_pair(vbase * i * gap, -hbase));
	}
	for (int j = 0; j <= hPtsA; j++)
	{
		sightAngleSet.push_back(std::make_pair(vFov, hbase * j * gap));
	}
	sightAngleSet.push_back(std::make_pair(vFov, -hbase));
	sightAngleSet.shrink_to_fit();

	return true;
}



void Drone::initialization(const double& initx, const double& inity, const double& initz,
	const std::tuple<double, double, double, double>& range, const double& ptsD)
{
	double x, y, z;
	if (initx <= std::get<0>(range))
	{
		x = std::get<0>(range);
	}
	else if (initx >= std::get<1>(range))
	{
		x = std::get<1>(range);
	}
	else
	{
		x = initx;
	}

	if (inity <= std::get<2>(range))
	{
		y = std::get<2>(range);
	}
	else if (inity >= std::get<3>(range))
	{
		y = std::get<3>(range);
	}
	else
	{
		y = inity;
	}

	if (initz > safeDis)
	{
		z = initz;
	}
	else
	{
		z = safeDis;
	}
	position = Point3(x, y, z);
	direction = Vector3(0, 1, 0);
	SASGen(ptsD);
}

void Drone::whatWeLook(const RTCScene& scene)
{
	int sightA = sightAngleSet.size();
	double dx = direction.x();
	double dy = direction.y();
	double dz = direction.z();

	PointSet3& hpts = perception.pts;
	#pragma omp parallel for
	for (int i = 0; i < sightA; i++)
	{
		double sdx = dx * std::cos(sightAngleSet.at(i).first) - dy * std::sin(sightAngleSet.at(i).first);
		double sdy = dy * std::cos(sightAngleSet.at(i).first) + dx * std::sin(sightAngleSet.at(i).first);
		double sdz = std::sin(sightAngleSet.at(i).second);
		RTCRayHit rayhits;
		rayhits.ray.org_x = static_cast<float>(position.x());
		rayhits.ray.org_y = static_cast<float>(position.y());
		rayhits.ray.org_z = static_cast<float>(position.z());

		rayhits.ray.dir_x = static_cast<float>(sdx);
		rayhits.ray.dir_y = static_cast<float>(sdy);
		rayhits.ray.dir_z = static_cast<float>(sdz);

		rayhits.ray.tnear = 0.;
		rayhits.ray.tfar = std::numeric_limits<float>::infinity();
		rayhits.hit.geomID = RTC_INVALID_GEOMETRY_ID;
		rayhits.hit.primID = RTC_INVALID_GEOMETRY_ID;

		RTCIntersectContext context;
		rtcInitIntersectContext(&context);
		rtcIntersect1(scene, &context, &rayhits);

		if (rayhits.hit.geomID != RTC_INVALID_GEOMETRY_ID) 
		{
			Point3 hit_position = position + rayhits.ray.tfar * Vector3(sdx, sdy, sdz);
			#pragma omp critical
			{
				hpts.insert(hit_position);
			}
		}
	}
	
}