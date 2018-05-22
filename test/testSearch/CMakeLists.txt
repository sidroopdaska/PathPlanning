file(GLOB SRCS *.cpp *.h)

ADD_EXECUTABLE(testSearch ${SRCS})

TARGET_LINK_LIBRARIES(testSearch
	search
	utilities
	gtest_main
)

add_test(NAME testSearch
		 COMMAND testSearch)