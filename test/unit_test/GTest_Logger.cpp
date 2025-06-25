#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <fstream>

class LoggerTest{
    public:
    void cerr(){
        std::cerr<<"err!"<<std::endl;
    }
};

TEST(Ofstream, callbacks){
    std::thread out_thread(LoggerTest::cerr);
    std::ofstream os("/tmp/ofstream.txt", std::ofstream::out);

}