#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <fstream>
#include "debug.h"

class LoggerTest: public Logger, public ::testing::Test{
    public:
    void cerr(){
        std::cerr<<"err!"<<std::endl;
    }
};

TEST_F(LoggerTest, foldername){
    char name[60];
    std::string custom("/tmp/test");
    std::string fileName=file_dateTime(custom.c_str(), name);
    EXPECT_TRUE(name);
    EXPECT_GT(fileName.size(), 0);
    EXPECT_EQ(fileName.size(), custom.size()+15);
    
}

TEST(Ofstream, callbacks){
    std::thread out_thread(LoggerTest::cerr);
    std::ofstream os("/tmp/ofstream.txt", std::ofstream::out);

}