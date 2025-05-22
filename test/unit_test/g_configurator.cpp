#include "../test_essentials.h"

TEST_F(Configurator_Test, platToTarget1){
    Task goal(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
    std::vector<vertexDescriptor> plan=get_plan("../target_68cm/");
    EXPECT_FALSE(plan.empty);
    EXPECT_TRUE(plan.size()!=0);
    // EXPECT_LT(configurator.)
}

int main(){

}