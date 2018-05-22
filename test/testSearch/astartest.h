/*
	astartest.h
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#ifndef astartest_h
#define astartest_h

#include <vector>
#include <utility>

#include "gtest/gtest.h"
#include "astar.h"
#include "logger.h"

namespace AStarTests {
	using ElDataType = std::vector<uint8_t>;
	using TDataType = std::vector<uint8_t>;

	/* Base fixture for the Search::AStar class */
	class AStarBase : public testing::Test {
	public:
		std::unique_ptr<Search::AStar> astar;

	protected:

		AStarBase() = default;
		virtual ~AStarBase() = default;

		virtual void SetUp() {}
		virtual void TearDown() {}
	};

	/* The fixture for testing arg handling by Search::AStar::search() */
	struct SearchArgs {
		int start;
		int goal;
		bool success;
	};

	class SeachArgsTest : public AStarBase, public ::testing::WithParamInterface<SearchArgs> {
	protected:
		SeachArgsTest() = default;
		virtual ~SeachArgsTest() = default;
	};

	/* The fixture for testing arg handling by Search::AStar::AStar() */
	struct CtorArgs {
		ElDataType elevationData;
		TDataType terrainData;
		int imgDim;
	};

	class CtorArgsTest : public AStarBase, public ::testing::WithParamInterface<CtorArgs> {
	protected:
		CtorArgsTest() = default;
		virtual ~CtorArgsTest() = default;
	};
}

#endif /* astartest_h */