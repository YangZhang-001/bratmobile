#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <fstream>
#include "debug.h"

class LoggerTest: public Logger, public ::testing::Test{};

TEST_F(LoggerTest, foldername){
    char name[60];
    std::string custom("/tmp/test");
    std::string fileName=file_dateTime(custom.c_str(), name);
    std::cout<<fileName<<std::endl;
    EXPECT_TRUE(name);
    EXPECT_GT(fileName.size(), 0);
    EXPECT_EQ(fileName.size(), custom.size()+16);
    
}

// TEST(Ofstream, callbacks){
//     std::thread out_thread(LoggerTest::cerr);
//     std::ofstream os("/tmp/ofstream.txt", std::ofstream::out);

// }