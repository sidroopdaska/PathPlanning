/*
	heuristic.cpp
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#include <cmath>

#include "heuristic.h"

namespace Search {
	float LinfHeuristic::evaluate(const int currentX, const int currentY,
		const int startX, const int startY,
		const int goalX, const int goalY) const {
		float dx = std::abs(currentX - goalX);
		float dy = std::abs(currentY - goalY);

		return std::fmax(dx, dy);
	}
}
