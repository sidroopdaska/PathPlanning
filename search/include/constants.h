/*
	constants.h
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#ifndef constants_h
#define constants_h

namespace Search {

	/* Bits used in the terrain image bytes */
	enum TerrainFlags
	{
		OF_RIVER_MARSH = 0x10,
		OF_INLAND = 0x20,
		OF_WATER_BASIN = 0x40
	};
}

#endif /* constants_h */