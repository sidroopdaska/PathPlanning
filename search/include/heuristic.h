#pragma once
/*
	heuristic.h
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#ifndef heuristic_h
#define heuristic_h

#include <string>

namespace Search {

	/* Defines an interface class for computing the Heuristic */
	class IHeuristic {
		std::string m_name;
	public:
		IHeuristic(std::string name) : m_name(name) {}

		std::string getName() { return m_name; }
		virtual float evaluate(const int currentX, const int currentY,
			const int startX, const int startY,
			const int goalX, const int goalY) const = 0;

		virtual ~IHeuristic() = default;
	};

	/* Implementation of IHeuristic abstract class.
	Implements the L-infinity heuristic
	*/
	class LinfHeuristic : public IHeuristic {
	public:
		LinfHeuristic(std::string name = "L-inifinity Heuristic")
			: IHeuristic(name) {}

		virtual float evaluate(const int currentX, const int currentY,
			const int startX, const int startY,
			const int goalX, const int goalY) const;

		virtual ~LinfHeuristic() = default;
	};
}

#endif /* heuristic_h */