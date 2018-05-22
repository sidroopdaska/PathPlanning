/*
	astar.cpp
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

#include "astar.h"
#include "constants.h"
#include "node.h"

namespace Search {

	/* The std::priority_queue<T> implements a max-heap by default. Since, we want to pop the node
		with the smallest value of fhat (= PathCost + HeuristicCost) we flip the sign
	*/
	bool operator<(const Node &n1, const Node &n2) {
		/* Used to perform Tie-Breaking, i.e. pick the node that has a lower `estimated` cost to
			the goal, thus leading to faster convergence
		*/
		if (n1.fhat == n2.fhat)
			return n1.hhat > n2.hhat;

		return n1.fhat > n2.fhat;
	}

	/* Used for node comparison while performing the goal test */
	bool operator==(const Node &n1, const Node &n2) {
		return n1.idx == n2.idx;
	}

	AStar::AStar(const std::vector<std::uint8_t> &edata,
		const std::vector<std::uint8_t> &tdata,
		float imgDim, Utilities::ILogger &logger,
		const IHeuristic &heuristic)
		:elevationData(edata), terrainData(tdata), imgDim(imgDim),
		logger(logger), heuristic(heuristic) {

		// @see https://isocpp.org/wiki/faq/exceptions#ctors-can-throw
		if (elevationData.empty() || terrainData.empty()) {
			throw std::runtime_error("AStar::AStar(), please provide a non-empty elevationData &"
				"terrainData vector");
		}

		if ((elevationData.size() != (imgDim * imgDim)) || (terrainData.size() != (imgDim * imgDim))) {
			throw std::runtime_error("AStar::AStar(), please ensure that the elevationData"
				"vector & terrainData vector have same shapes");
		}

		if (imgDim <= 0)
			throw std::runtime_error("AStar::AStar(), the imgDim needs to be > 0");
	}

	bool AStar::isValidState(const int x, const int y) {
		// Bounds checking
		if (x < 0 || x >= imgDim || y < 0 || y >= imgDim)
			return false;

		auto elevation = elevationData[y * imgDim + x];
		auto terrain = terrainData[y * imgDim + x];

		// Terrain analysis
		if ((terrain & (OF_RIVER_MARSH | OF_WATER_BASIN)) || elevation == 0)
			return false;

		return true;
	}

	void AStar::findNeighbours(const int node, std::vector<int> &outNeighbours) {
		int y = node / imgDim;
		int x = node % imgDim;

		// Enumerates all the 8 directions of freedom
		std::vector<std::pair<int, int>> possibleMoves = {
			{ x, y - 1 }, // north
			{ x, y + 1 }, // south
			{ x - 1, y }, // west
			{ x + 1, y }, // east
			{ x - 1, y - 1 }, // north west
			{ x + 1, y - 1 }, // north east
			{ x - 1, y + 1 }, // south west
			{ x + 1, y + 1 } // south east
		};

		int idx = 0;
		for (const auto &move : possibleMoves) {
			auto xnew = move.first;
			auto ynew = move.second;

			if (!isValidState(xnew, ynew)) {
				outNeighbours[idx] = -1;
			}
			else {
				outNeighbours[idx] = ynew * imgDim + xnew;
			}

			idx++;
		}
	}

	float AStar::computeStepCost(const int src, const int dest) {
		auto dx = std::abs((src % imgDim) - (dest % imgDim));
		auto dy = std::abs((src / imgDim) - (dest / imgDim));

		// Results in either 1.0 or sqrt(2)
		float horizontalDist = std::sqrt(dx * dx + dy * dy);

		// Factoring in the elevation angle and scaling it to the range of (-2., 2.)
		// to make the scales match with the horizontalDist
		float delevation = ((elevationData[dest] - elevationData[src]) / 255.0) * 2.0;

		// Computing the overall distance owing to the elevation diff 
		float totalDist = std::sqrt(horizontalDist * horizontalDist + delevation * delevation);

		// Rovers normal speed
		float speed = 1.0;

		// Calculating the cost (in seconds) as a function of distance and elevation angle.
		// The formula below increases the time when going uphill and conversely decreases the time
		// while going downhill
		float cost;
		cost = totalDist / (speed * (1 - delevation));
		return cost;
	}

	//float AStar::computeHeuristicCost(const int node, const int start, const int goal) {
	//	float D = 1.0;
	//	float D2 = std::sqrt(2.0);
	//	auto dx = std::abs((node % imgDim) - (goal % imgDim));
	//	auto dy = std::abs((node / imgDim) - (goal / imgDim));

	//	// L infinity heuristic / Chebyshev distance
	//	float heuristic = std::max(dx, dy);

	//	// Diagonal Distance Heuristic
	//	//float heuristic = D * (dx + dy) + (D2 - 2 * D) * std::fmin(dx, dy);
	//	
	//	/* Alternative Tie Breaking approach: Computes the vector cross-product between
	//		the start to goal vector and the current node to goal vector. When these vectors
	//		don’t line up, the cross product will be larger. Therefore, the
	//		algorithm should some give slight preference to a path that lies along the straight line
	//		path from the start to the goal.
	//		Note: although this approach gave great results in finding a path between Bachelor -> Wedding,
	//		for some odd reason this technique seems to get trapped while searching for a path between
	//		Rover -> Bachelor. Need to investigate and use this as future work for an even speedier
	//		convergence
	//	*/
	//	//auto dx2 = (start % imgDim) - (goal % imgDim);
	//	//auto dy2 = (start / imgDim) - (goal / imgDim);
	//	//float cross = std::abs(dx * dy2 - dx2 * dy);
	//	//auto heuristicWithTieBreak = heuristic + cross * 0.0003;
	//	
	//	return heuristic;
	//}

	AStarSearchResult AStar::search(const int start, const int goal) {

		try {
			// Checks for valid start and goal nodes
			if (!isValidState(start % imgDim, start / imgDim) || !isValidState(goal % imgDim, goal / imgDim)) {
				logger.print("Please provide a valid start and goal state");
				return AStarSearchResult(false, ParentsVec());
			}

			logger.print("Planning route between (" +
				std::to_string(start % imgDim) + ", " +
				std::to_string(start / imgDim) + ") and " +
				"(" + std::to_string(goal % imgDim) + ", " +
				std::to_string(goal / imgDim) + ")...");

			Node startNode(start, 0., 0.);
			Node goalNode(goal, 0., 0.);

			std::priority_queue<Node> nodesToVisit;

			// Path costs, flattened vector with the same shape as that of the flattened grid. Stores the
			// path cost to traverse from the start node up until the current node. `pathCosts[nodeIdx]` represents
			// the `g` in the formula `f = g + h` for a particular node
			const float INF = std::numeric_limits<float>::infinity();
			std::vector<float> pathCosts(imgDim * imgDim, INF);

			std::vector<int> neighbours(8, -1);
			ParentsVec parents(imgDim * imgDim, std::make_pair(-1, false));

			bool solutionFound = false;
			long numNodesExpanded = 0;

			nodesToVisit.push(startNode);
			pathCosts[startNode.idx] = 0.;

			// Starting the clock for time profiling		
			std::clock_t startTime = std::clock();

			while (!nodesToVisit.empty()) {

				// Paths with lower value of f (= PathCost + HeuristicCost) are explored first
				Node current = nodesToVisit.top();
				nodesToVisit.pop();

				if (current == goalNode) {
					solutionFound = true;
					auto duration = (std::clock() - startTime) / (double)CLOCKS_PER_SEC;
					logger.print("Path found! Time taken: " + std::to_string(duration) +
						" seconds. Nodes Expanded: " + std::to_string(numNodesExpanded) +
						" . Path traversal time: " + std::to_string(pathCosts[current.idx])
						+ " island seconds");

					// Flagging all the nodes in the path leading upto the goal node
					auto idx = current.idx;
					while (idx != -1) {
						parents[idx].second = true;
						idx = parents[idx].first;
					}
					parents[start].second = true;
					break;
				}

				numNodesExpanded++;
				findNeighbours(current.idx, neighbours);

				for (int i = 0; i < 8; ++i) {
					float newCost = 0.;
					float heuristicCost = 0.;
					float fhat = 0.;

					// If valid neighbour
					if (neighbours[i] >= 0) {

						// Calculates the path cost so far and the cost of this move
						newCost = pathCosts[current.idx] + computeStepCost(current.idx, neighbours[i]);

						if (newCost < pathCosts[neighbours[i]]) {
							// Estimate the cost from the current node to the goal node based on the
							// above defined heuristic
							heuristicCost =
								heuristic.evaluate(neighbours[i] % imgDim, neighbours[i] / imgDim,
									start % imgDim, start / imgDim,
									goal % imgDim, goal / imgDim);

							fhat = newCost + heuristicCost;
							nodesToVisit.push(Node(neighbours[i], heuristicCost, fhat));

							pathCosts[neighbours[i]] = newCost;
							parents[neighbours[i]].first = current.idx;
						}
					}
				}
			}

			return AStarSearchResult(solutionFound, parents);
		}
		catch (const std::exception &exception) {
			std::cerr << "AStar::search() failed with exception: " << exception.what() << std::endl;
			throw;
		}

		return AStarSearchResult(false, ParentsVec());
	}
}
