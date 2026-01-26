#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <fstream>
#include "test_classes.h"
const bool DEBUG=false;

class LoggerTest: public Logger, public testing::Test{
    public:

	LoggerTest(){}

	LoggerTest(char * new_folder, char * _dir=NULL, char * customName="/stats"){
		init(new_folder, _dir, customName);
	}

    ~LoggerTest(){}

    void SetUp()override{    }

    void TearDown()override{
        closef();
    }

    void TestBody()override{}

    FILE * file(){
        return f;
    }

    void closef(){
        if (NULL!=f){
            fclose(f);
        }
        f=NULL;
    }

};

TEST_F(LoggerTest, foldername){
    char name[60];
    std::string custom("/tmp/test");
    std::string fileName=file_dateTime(custom.c_str(), name);
    std::cout<<fileName<<std::endl;
    EXPECT_TRUE(name);
    EXPECT_GT(fileName.size(), 0);
    EXPECT_EQ(fileName.size(), custom.size()+16);
}

TEST(logger, Constructor){
    Logger logger("", "/tmp", "constructortest");
    EXPECT_TRUE(logger.get_fileName()!=NULL);
}

TEST_F(LoggerTest, fprintf){
    f=fopen("/tmp/fprintf_test.txt", "w");
    log("%s\n", "why hello!");
}

TEST_F(ConfiguratorTest, logger){
    LoggerTest loggerTest("", "/tmp", "TEST");
    register_logger(&loggerTest);
    EXPECT_FALSE(get_logger()==NULL);
    loggerTest.log("%s\n", "hello!");
    loggerTest.log("%s\n", "hallo!","hullo!");
}
