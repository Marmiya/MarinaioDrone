#include "Perception.h"

Perception::Perception(
	const std::tuple<double, double, double, double>& range,
	const std::vector<double>& xgp, const std::vector<double>& ygp)
	:range(range)
{
	xGridIndexA = xgp.size() - 1;
	yGridIndexA = ygp.size() - 1;
	for (int i = 0; i < xgp.size() - 1; i++)
	{
		stateNet.emplace_back(std::vector<stateOfCell>(ygp.size() - 1, unknown));
	}
	stateNet.shrink_to_fit();
	CCL = xgp.at(1) - xgp.at(0);
}

void Perception::initialization(const Point3& initialPos)
{
	int xindex = static_cast<int>(std::floor((initialPos.x() - std::get<0>(range)) / CCL));
	int yindex = static_cast<int>(std::floor((initialPos.y() - std::get<2>(range)) / CCL));
	stateNet.at(xindex).at(yindex) = safe;
}

bool Perception::Pcptcheck() const
{
	for (const auto& i : stateNet)
	{
		for(const auto&j:i)
		{
			if (j == unknown)
				return false;
		}
	}
	return true;
}