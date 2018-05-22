/*
	logger.h
	Bachelor
	@author Siddharth Sharma <@sidroopdaska>
*/

#ifndef logger_h
#define logger_h

#include <mutex>
#include <string>

namespace Utilities {
	/* Interface (abstract) class for exposing common Logger functionality */
	class ILogger {
	public:
		virtual void print(std::string str) = 0;
		virtual ~ILogger() {};
	};

	/* Implementation of the ILogger interface for outputting to the Console stream */
	class ConsoleLogger : public ILogger {
		/* Using a mutex to prevent race condition for shared resources 
			between multiple threads
		*/
		std::mutex mu;

	public:
		virtual void print(std::string str);
	};
}

#endif /* logger_h */