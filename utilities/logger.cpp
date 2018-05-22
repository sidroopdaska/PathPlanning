/*
	logger.cpp
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#include <iostream>

#include "logger.h"

namespace Utilities {
	void ConsoleLogger::print(std::string str) {
		std::lock_guard<std::mutex> locker(mu);
		std::cout << str << std::endl;
	}
}
