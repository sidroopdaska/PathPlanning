/*
	node.h
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#ifndef node_h
#define node_h

namespace Search {
	/* Class to represent a single pixel within the search space */
	class Node {
	public:
		/* Index in the flattened grid */
		int idx;

		/* Estimated cost to traverse to the goal node from the current node */
		float hhat;

		/* Stores f = g(Path Cost) + h(Heuristic Cost). This value is used as a `priority value`
			in our min-heap to pop nodes from the open list for expansion
		*/
		float fhat;

		Node(int idx, float h, float f) : idx(idx), hhat(h), fhat(f) {}
	};
}

#endif /* node_h*/