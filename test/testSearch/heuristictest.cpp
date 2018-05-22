/*
	heuristictest.cpp
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#include "heuristictest.h"

namespace HeuristicTests {

	TEST_P(EvaluateTest, LinfHeuristicGetsComputedCorrectly) {
		Search::LinfHeuristic heuristic;

		auto as = GetParam();
		auto result = heuristic.evaluate(as.currX, as.currY, as.startX, as.startY, as.goalX, as.goalY);
		EXPECT_EQ(result, as.result);
	}

	INSTANTIATE_TEST_CASE_P(CheckingEvalutionResult, EvaluateTest,
		testing::Values(
			EvaluateArgs{ 0, 0, 1, 1, 2, 2, 2 },
			EvaluateArgs{ 159, 1520, 89, 1390, 45, 45, 1475 }
	));
}