#include <gtest/gtest.h>
#include "test_classes.h"
const bool DEBUG=false;

class MatcherTest:public StateMatcher, public testing::Test, public testing::WithParamInterface<std::tuple<StateMatcher::MATCH_TYPE, StateMatcher::MATCH_TYPE>>{
    
    public:
   // void TestBody() override{}
    void SetUp() override{}
    void TearDown() override{}

    std::vector<MATCH_TYPE> get_matches(MATCH_TYPE m){
        std::vector<MATCH_TYPE> result;
        switch(m)
        {
        case _FALSE:
            result={_FALSE};
            break;
        case ANY:
            result={StateMatcher::_TRUE,ABSTRACT,POSE,ANY,D_NEW,DN_POSE, D_INIT, DI_POSE,DN_SHAPE,DI_SHAPE};
            break;
        case ABSTRACT:
            result={StateMatcher::_TRUE, ABSTRACT};
            break;
        case POSE:
            result={StateMatcher::_TRUE,POSE};
            break;
        case D_NEW:
            result={StateMatcher::_TRUE, ABSTRACT,D_NEW};
            break;
        case D_INIT:
            result={StateMatcher::_TRUE, ABSTRACT,D_INIT};
            break;
        case DN_POSE:
            result={StateMatcher::_TRUE,StateMatcher::ABSTRACT,D_NEW,DN_POSE};
            break;
        case DI_POSE:
            result={StateMatcher::_TRUE,ABSTRACT,D_INIT,DI_POSE};
            break;
        case DN_SHAPE:
            result={StateMatcher::_TRUE,ABSTRACT,D_NEW,DN_SHAPE};
            break;
        case DI_SHAPE:
            result={StateMatcher::_TRUE,ABSTRACT,D_INIT,DI_SHAPE};
            break;
        case StateMatcher::_TRUE:
            result={StateMatcher::_TRUE};
            break;
        default:
            break;
        }
        return result;
    }

};

TEST_P(MatcherTest, Equal){
    StateMatcher::MATCH_TYPE candidate=std::get<0>(GetParam());
    StateMatcher::MATCH_TYPE desired=std::get<1>(GetParam());
    bool result=match_equal(candidate, desired);
    std::vector<MATCH_TYPE> matches=get_matches(desired);
    std::vector<MATCH_TYPE>::iterator it=check_vector_for(matches, candidate);
    if (it!=matches.end()){
        EXPECT_TRUE(result );
    }
    else{
        EXPECT_FALSE(result);
    }
}

INSTANTIATE_TEST_CASE_P(MatchTypes, MatcherTest, testing::Combine(testing::Values(StateMatcher::_FALSE, 
                                                                    StateMatcher::_TRUE,
                                                                    StateMatcher::ABSTRACT,
                                                                    StateMatcher::POSE ,
                                                                    StateMatcher::ANY,
                                                                    StateMatcher::D_NEW,
                                                                    StateMatcher::DN_POSE, 
                                                                    StateMatcher::D_INIT,
                                                                    StateMatcher::DI_POSE,
                                                                    StateMatcher::DN_SHAPE,
                                                                    StateMatcher::DI_SHAPE
                                                                    ),
                                                                    testing::Values(StateMatcher::_FALSE, 
                                                                    StateMatcher::_TRUE,
                                                                    StateMatcher::ABSTRACT,
                                                                     StateMatcher::POSE ,
                                                                    StateMatcher::ANY,
                                                                    StateMatcher::D_NEW,
                                                                    StateMatcher::DN_POSE, 
                                                                    StateMatcher::D_INIT,
                                                                    StateMatcher::DI_POSE,
                                                                    StateMatcher::DN_SHAPE,
                                                                    StateMatcher::DI_SHAPE
                                                                    ) 
                                                                    ));