/*
	astar.h
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#ifndef astar_h
#define astar_h

#include <vector>

#include "heuristic.h"
#include "logger.h"

namespace Search {

	/* Type alias for the `parents` vector. Each element within the vector is pair of
		(node index in the flattened grid, bool flag to indicate whether the node is part of the final path)
	*/
	using ParentsVec = std::vector<std::pair<int, bool>>;

	/* Represents the search result */
	class AStarSearchResult {
	public:
		bool isSuccessful;
		ParentsVec parents;

		AStarSearchResult(bool result, ParentsVec p) : isSuccessful(result), parents(p) {}
	};

	/* Class providing the state and behaviour for the A* Search algorithm
		Note: the Search Library and consequently the A* algorithm operates on a flattened 1-D grid
		Therefore all the nodes are hereby represented with an integer index value in this flattened grid
	*/
	class AStar {
		/*
			Dimension of the search space or grid. Note: this can be extended
			to cater rect grids if necessary
		*/
		int imgDim;
		const std::vector<std::uint8_t> &elevationData;
		const std::vector<std::uint8_t> &terrainData;
		Utilities::ILogger &logger;
		const IHeuristic &heuristic;

		/*
			Performs bound checking to ensure we have a validate coordinate.
			Performs terrain analysis to ensure that the coordinate can be traversed upon.
			@param x: The x coordinate, index in the collapsed grid
			@param x: The y coordinate, index in the collapsed grid
		*/
		bool isValidState(const int x, const int y);

		/*
			Computes the next possible moves/states that can be explored for the current node
			@param node: The current node being analysed/expanded. This is consumed as an
				index to the collapsed grid
			@param outNeighbours: Reference to a vector for returning the computed neighbours.
				The neighbours are returned as indexes in the collapsed grid
		*/
		void findNeighbours(const int node, std::vector<int> &outNeighbours);

		/*
			Calculates the cost to take a `step` from the current node to its immediate neighbour
			@param src: The current node; index in the flattened grid
			@param dest: The neighbouring node; index in the flattened grid
		*/
		float computeStepCost(const int src, const int dest);

		/*
			DEPRECATED: Calculates estimated cost to traverse from the current node to the goal node.
			Leverages Diagonal Distance Heuristic + Tie Breaking for quick convergence
			@param node: The current node; index in the flattened grid
			@param start: The start node; index in the flattened grid
			@param goal: The goal node; index in the flattened grid
		*/
		//float computeHeuristicCost(const int node, const int start, const int goal);

	public:

		AStar(const std::vector<std::uint8_t> &edata,
			const std::vector<std::uint8_t> &tdata,
			float imgDim, Utilities::ILogger &logger, 
			const IHeuristic &heuristic);

		/*
			Kernel of the class. Implements the core A* search algorithm
			@param start: Start location, index in the flattened grid
			@param goal: Goal location, index in the flattened grid
		*/
		AStarSearchResult search(const int start, const int goal);

	};
}

#endif /* astar_h */
