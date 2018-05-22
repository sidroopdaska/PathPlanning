/*
	main.cpp
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "astar.h"
#include "constants.h"
#include "heuristic.h"
#include "logger.h"
#include "visualizer.h"

#ifdef _MSC_VER
static const char* PATH_SEP = "\\";
#else
static const char* PATH_SEP = "/";
#endif

// Some constants
enum {
	IMAGE_DIM = 2048, // Width and height of the elevation and overrides image

	ROVER_X = 159,
	ROVER_Y = 1520,
	BACHELOR_X = 1303,
	BACHELOR_Y = 85,
	WEDDING_X = 1577,
	WEDDING_Y = 1294
};

std::ifstream::pos_type fileSize(const std::string& filename) {
	std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
	if (!in.good())
	{
		throw std::exception();
	}
	return in.tellg();
}

std::vector<uint8_t> loadFile(const std::string& filename, size_t expectedFileSize) {
	size_t fsize = fileSize(filename);
	if (fsize != expectedFileSize)
	{
		throw std::exception();
	}
	std::vector<uint8_t> data(fsize);
	std::ifstream ifile(filename, std::ifstream::binary);
	if (!ifile.good())
	{
		throw std::exception();
	}
	ifile.read((char*)&data[0], fsize);
	return data;
}

bool donut(int x, int y, int x1, int y1) {
	int dx = x - x1;
	int dy = y - y1;
	int r2 = (dx * dx + dy * dy) + 200;
	return r2 >= 150 && r2 <= 400;
}

/*
	Takes in a pair of vertices, computes the optimal path between them and draws it
	on a map.
	@param elevationData: Const reference to the elevation data
	@param terrainData: Const reference to the terrain info
	@param start: Start location; index on the flattened grid
	@param goal: Goal location; index on the flattened grid
	@param fileName: Name of the resulting bitmap file
	@param imgDim: Shape of the search space matrix. Currently supporting square grids with the possibility
		for extension to rectangular grid
	@param logger: Multi-thread friendly logger instance
*/
void planPath(const std::vector<std::uint8_t> &elevationData,
	const std::vector<std::uint8_t> &terrainData,
	const std::pair<int, int> start,
	const std::pair<int, int> goal,
	std::string fileName,
	int imgDim,
	Utilities::ILogger &logger) {

	try {
		Search::AStar astar(elevationData, terrainData, imgDim, logger, Search::LinfHeuristic());
		auto result = astar.search(start.second * imgDim + start.first,
			goal.second * imgDim + goal.first);

		if (!result.isSuccessful) {
			logger.print("Woops! Unable to find a path.");
			return;
		}

		logger.print("Sketching out path...");
		std::ofstream of(fileName + ".bmp", std::ofstream::binary);

		visualizer::writeBMP(
			of,
			&elevationData[0],
			IMAGE_DIM,
			IMAGE_DIM,
			[&](size_t x, size_t y, uint8_t elevation) {

			// Signifies water
			if ((terrainData[y * imgDim + x] & (Search::OF_WATER_BASIN | Search::OF_RIVER_MARSH))
				|| elevation == 0)
			{
				return uint8_t(visualizer::IPV_WATER);
			}

			// Marks target positions on the map
			if (donut(x, y, start.first, start.second) || donut(x, y, goal.first, goal.second))
			{
				return uint8_t(visualizer::IPV_PATH);
			}

			// Draws the path
			if (result.parents[y * IMAGE_DIM + x].first != -1 && result.parents[y * IMAGE_DIM + x].second)
				return uint8_t(visualizer::IPV_PATH);

			// Signifies normal ground color
			if (elevation < visualizer::IPV_ELEVATION_BEGIN)
			{
				elevation = visualizer::IPV_ELEVATION_BEGIN;
			}
			return elevation;
		});

		of.flush();
		logger.print("Done!");
	}
	catch (const std::exception &exception) {
		std::cerr << "planPath() Exception: " << exception.what() << std::endl;
	}
	catch (...) {
		std::cerr << "Sorry, the planPath() failed unexpectedly!" << std::endl;
	}
}

int main(int argc, char** argv) {
	const size_t expectedFileSize = IMAGE_DIM * IMAGE_DIM;
	// Address assets relative to application location
	std::string anchor = std::string(".") + PATH_SEP;
	std::string pname = argv[0];
	auto lastpos = pname.find_last_of("/\\");
	if (lastpos != std::string::npos) {
		anchor = pname.substr(0, lastpos) + PATH_SEP;
	}

	auto elevation = loadFile(anchor + "assets" + PATH_SEP + "elevation.data", expectedFileSize);
	auto overrides = loadFile(anchor + "assets" + PATH_SEP + "overrides.data", expectedFileSize);

	Utilities::ConsoleLogger logger;
	std::tuple<std::pair<int, int>, std::pair<int, int>, std::string> cases[] = {
		{{ROVER_X, ROVER_Y}, {BACHELOR_X, BACHELOR_Y}, "rover-bachelor"},
		{{BACHELOR_X, BACHELOR_Y}, {WEDDING_X, WEDDING_Y}, "bachelor-wedding"}
	};

	int sizeOfCases = sizeof(cases) / sizeof(cases[0]);
	std::vector<std::thread> threads(sizeOfCases);

	for (int i = 0; i < sizeOfCases; i++) {
		threads[i] = std::thread(planPath,
			std::ref(elevation),
			std::ref(overrides),
			std::get<0>(cases[i]),
			std::get<1>(cases[i]),
			std::get<2>(cases[i]),
			IMAGE_DIM,
			std::ref(logger)
		);
	}

	for (int i = 0; i < threads.size(); i++) { threads[i].join(); }

#if __APPLE__
	auto res = system("open pic.bmp");
	(void)res;
#endif
	//system("pause");
	return 0;
}