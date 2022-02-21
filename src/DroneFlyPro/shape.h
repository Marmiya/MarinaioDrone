#pragma once
#include <tiny_obj_loader.h>
#include <utility>
#include <vector>

namespace SHP {
	class shape
	{
	private:
		tinyobj::shape_t oriShape;
		int targetID;
	public:
		shape() = default;
		shape(tinyobj::shape_t s, int id) :oriShape(s), targetID(id) {};
		
		tinyobj::shape_t& getOriShape() {
			return oriShape;
		}

		int getTargetID()const {
			return targetID;
		}

		void setTargetID(int id) {
			targetID = id;
		}
	};
}