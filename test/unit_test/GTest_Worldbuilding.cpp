#include "test_classes.h"
#include <gtest/gtest.h>

TEST(Math, affineTransform){
    Disturbance disturbance(AVOID, b2Vec2(0,0));
    b2Transform transform(b2Vec2(0.5,0), b2Rot(M_PI_2));
    Task task(disturbance, DEFAULT);
    math::MulT(transform, *task.get_disturbance_ptr());
    EXPECT_FALSE(task.get_disturbance().pose()==disturbance.pose());
}

TEST(Robot, Pose){
    b2World world(GRAVITY);
    std::vector<b2Vec2> robotVertices=Robot::get_vertices();
    Robot robot(&world);
    b2AABB aabb =robot.body->GetFixtureList()->GetAABB(0);
    EXPECT_NEAR(aabb.upperBound.y, ROBOT_HALFLENGTH, 0.01);
    EXPECT_NEAR(aabb.lowerBound.y, -ROBOT_HALFLENGTH, 0.01);
    EXPECT_NEAR(aabb.upperBound.x, ROBOT_HALFWIDTH+ROBOT_BOX_OFFSET_X, 0.01);
    EXPECT_NEAR(aabb.lowerBound.y, -ROBOT_HALFWIDTH-ROBOT_BOX_OFFSET_X, 0.01);

}

/**
 * @brief Class to test world building tools (third party)
 * 
 */
class ThirdPartyWB: public ::testing::Test, public testing::WithParamInterface<int>{
    public:

    void SetUp(){}
    void TearDown(){}

    /**
     * @brief Makes custom size cul de sac
     * 
     * @param pts total points
     * @param pts_per_side how many points per side
     * @param sideL left side (y>0)
     * @param sideR right side (y<0)
     * @param front robot-facing side 
     */
    void make_culdesac(std::vector<cv::Point2f>& pts, int pts_per_side, float halfWidth=0.05, std::vector<cv::Point2f>* sideL=NULL, std::vector<cv::Point2f>* sideR=NULL, std::vector<cv::Point2f>* front=NULL){
        float x=0;
        for (int i=0; i<pts_per_side; i++){
            cv::Point2f pt_Lside(x, halfWidth), pt_Rside(x, -y);
            pts.push_back(pt_Lside);
            pts.push_back(pt_Rside);
            if (sideL){
                sideL->push_back(pt_Lside);
            }
            if (sideR){
                sideR->push_back(pt_Rside);
            }
            x+=0.01;
        }
        for (int i=0; i<pts_per_side; i++){
            cv::Point2f pt_front(x, halfWidth);
            if (front){
                front->push_back(pt_front);
            }
            pts.push_back(pt_front);
            halfWidth-=0.01;

        }

    }
};

/**
 * @brief Makes cul de sac and tests if concave
 * 
 */
TEST_F(ThirdPartyWB, testConcave){
    std::vector<cv::Point2f> pts; //total points + sides of cul de sac
    make_culdesac(pts, 10);
    FILE * f=fopen("/tmp/cds_convex.txt", "w");
    for (cv::Point2f p:pts){ 
        fprintf(f, "%.02f\t%.02f\n", p.x, p.y);
    }
    fclose(f);
    EXPECT_FALSE(cv::isContourConvex(pts));
}

TEST_P(ThirdPartyWB, HoughLines){
    std::vector<cv::Point2f> pts, Lside, Rside, front;
    make_culdesac(pts, GetParam(), &Lside, &Rside, &front);
    std::vector<cv::Vec2f> lines;
    try{
        cv::HoughLines(pts, lines, 1, CV_PI/180, 150, 0);

    }
    catch(...){
        std::cout<<"Hough Transform only takes images";
    }
}

TEST_P(ThirdPartyWB, LineSegment){
    std::vector<cv::Point2f> pts, Lside, Rside, front;
    cv::Ptr<cv::LineSegmentDetector> lsd= cv::createLineSegmentDetector(0);
    make_culdesac(pts, GetParam(), &Lside, &Rside, &front);
    std::vector<cv::Vec4f> lines;
    try{
        lsd->detect(pts, lines);
    }
    catch(...){
        std::cout<<"Line Segment Detector only takes images";
    }
}

TEST_P(ThirdPartyWB, CornerHarris){
    std::vector<cv::Point2f> pts, Lside, Rside, front, out;
    make_culdesac(pts, GetParam(), &Lside, &Rside, &front);
    std::vector<cv::Vec4f> lines;
    try{
        cv::cornerHarris(pts, out, 1, 1, 1, 1);
    }
    catch(...){
        std::cout<<"Corner Harris only takes images";
    }

}





INSTANTIATE_TEST_CASE_P(cds_sizes, ThirdPartyWB, ::testing::Values(10, 50, 100));