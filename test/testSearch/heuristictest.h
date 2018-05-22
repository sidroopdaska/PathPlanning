/*
	heuristictest.h
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#ifndef heuristictest_h
#define heuristictest_h

#include "gtest/gtest.h"
#include "heuristic.h"

namespace HeuristicTests {

	/* Base fixture for the Search::Heuristic class */
	class HeuristicBase : public testing::Test {
	protected:

		HeuristicBase() = default;
		virtual ~HeuristicBase() = default;

		virtual void SetUp() {}
		virtual void TearDown() {}
	};

	/* The fixture for testing arg handling by Search::AStar::search() */
	struct EvaluateArgs {
		int currX;
		int currY;
		int startX;
		int startY;
		int goalX;
		int goalY;

		int result;
	};

	class EvaluateTest : public HeuristicBase, public ::testing::WithParamInterface<EvaluateArgs> {
	protected:
		EvaluateTest() = default;
		virtual ~EvaluateTest() = default;
	};
}

#endif /* heuristictest_h */