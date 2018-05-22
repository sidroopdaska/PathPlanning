/*
	astartest.cpp
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#include <tuple>
#include <stdexcept>

#include "astartest.h"
#include "constants.h"
#include "heuristic.h"

namespace AStarTests {

	Utilities::ConsoleLogger logger;
	Search::LinfHeuristic heuristic;

	/* Executes the A* search algorithm given the passed args and returns the result */
	Search::AStarSearchResult executeSearch(const std::tuple<ElDataType, TDataType, int> &gridInfo,
		int start, int goal) {
		Search::AStar astar(std::get<0>(gridInfo), std::get<1>(gridInfo),
			std::get<2>(gridInfo), logger, heuristic);

		auto result = astar.search(start, goal);
		return result;
	}

	TEST_P(SeachArgsTest, ValidatesArgs) {
		// Creating a 2x2 Grid
		auto imgDim = 2;
		ElDataType elData(imgDim * imgDim, 0x01);
		TDataType tData = {
			static_cast<uint8_t>(Search::OF_INLAND),
			static_cast<uint8_t>(Search::OF_RIVER_MARSH),
			static_cast<uint8_t>(Search::OF_INLAND),
			static_cast<uint8_t>(Search::OF_INLAND)
		};

		auto as = GetParam();
		auto result = executeSearch(std::make_tuple(elData, tData, imgDim), as.start, as.goal);
		EXPECT_EQ(result.isSuccessful, as.success);
	}

	INSTANTIATE_TEST_CASE_P(CheckingArgs, SeachArgsTest,
		testing::Values(
			SearchArgs{ 0, 0, true }, // all valid states
			SearchArgs{ 0, 1, false }, // goal state is invalid
			SearchArgs{ 1, 0, false } // start state is valid
	));

	TEST_P(CtorArgsTest, ValidatesArgs) {
		auto as = GetParam();

		EXPECT_THROW({
			Search::AStar astar1(as.elevationData, as.terrainData, as.imgDim, logger, heuristic);
			}, std::runtime_error
		);
	}

	INSTANTIATE_TEST_CASE_P(CheckingArgs, CtorArgsTest,
		testing::Values(
			CtorArgs{ ElDataType(), TDataType(), 2 }, // both empty
			CtorArgs{ ElDataType(100), TDataType(100), 2 }, // both different from imgDim
			CtorArgs{ ElDataType(100), TDataType(2), 2 }, // elevation Data different from imgDim
			CtorArgs{ ElDataType(2), TDataType(100), 2 }, // terrain data diff from imgDim
			CtorArgs{ ElDataType(100), TDataType(100), 0 } // imgDim invalid
	));

	/* Creates a uniformly elevated sample grid with two disjoint islands */
	std::tuple<ElDataType, TDataType, int> createSearchGridWithVoid() {
		auto imgDim = 3;

		auto land = static_cast<uint8_t>(Search::OF_INLAND);
		auto water = static_cast<uint8_t>(Search::OF_WATER_BASIN);

		ElDataType eldata(imgDim * imgDim, 0x01);
		TDataType tdata{
			land, land, land,
			land, water, water,
			land, water, land
		};

		return std::make_tuple(eldata, tdata, imgDim);
	}

	/* Creates a sample grid with elevation data */
	std::tuple<ElDataType, TDataType, int> createSearchGridWithElData() {
		auto imgDim = 3;

		auto land = static_cast<uint8_t>(Search::OF_INLAND);
		auto water = static_cast<uint8_t>(Search::OF_WATER_BASIN);

		ElDataType eldata{
			0x08,0x04,0x03,
			0x04,0x02,0x03,
			0x02,0x02,0x02
		};

		TDataType tdata{
			land, land, land,
			land, land, land,
			land, land, land
		};

		return std::make_tuple(eldata, tdata, imgDim);
	}

	TEST(HappyPaths, SeachFindsCorrectPathWithNoElevationData) {
		auto result = executeSearch(createSearchGridWithVoid(), 2, 6);
		EXPECT_EQ(true, result.isSuccessful);
	}

	TEST(HappyPaths, SearchEndsWhenNoPathIsAvailable) {
		auto result = executeSearch(createSearchGridWithVoid(), 0, 8);
		EXPECT_EQ(false, result.isSuccessful);
	}

	TEST(HappyPaths, SearchFindsCorrectPathWithElevationData) {
		auto result = executeSearch(createSearchGridWithElData(), 6, 2);
		EXPECT_EQ(true, result.isSuccessful);

		// Makes certain that the path is correct
		auto path = { 6, 4, 2 };
		for (const auto &step : path) {
			EXPECT_EQ(true, result.parents[step].second);
		}
	}
}