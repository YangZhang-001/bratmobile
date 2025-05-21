#ifndef SENSOR_H
#define SENSOR_H
#include "CloCK_math.h"

class ConfiguratorInterface;
class Configurator;

class Pointf: public cv::Point2f{
	public: 

	Pointf(){}

	Pointf(float _x, float _y){
		x=_x;
		y=_y;
	}
    
    Pointf operator+(const Pointf p){
			Pointf result;
			result.x = x + p.x;
			result.y = y+ p.y;
			return result;
	}

	Pointf operator-(const Pointf p){
			Pointf result;
			result.x = x - p.x;
			result.y = y- p.y;
			return result;
	}

	bool isin(Pointf, Pointf);

};


template<>
struct cv::traits::Depth<Pointf> {enum {value = Depth<cv::Point2f>::value};};

template<>
struct cv::traits::Type<Pointf> {enum {value = CV_MAKETYPE(cv::traits::Depth<Pointf>::value, 2)};};

float length(cv::Point2f const& p);

float angle(cv::Point2f const&);

bool operator <(Pointf const &, Pointf const&);

bool operator >(const Pointf&,  const Pointf&);

typedef std::set<Pointf> CoordinateContainer;


b2Vec2 getb2Vec2(cv::Point2f );

template <typename T>
Pointf getPointf(T);

// template <typename T>
// cv::Point2f getPoint2f(T);

Pointf Polar2f(float, float);

template <typename T>
std::vector<T> set2vec(std::set<T>);

template <typename T>
std::vector<cv::Point2f> set2vec2f(std::set<T> s){
    std::vector <cv::Point2f> vec;
    for (T t:s){
        vec.push_back(cv::Point2f(t.x, t.y));
    }
    return vec;
}

template <typename T> inline
std::vector<b2Vec2> cast_b2Vec2(const std::vector<T>& v){
	std::vector<b2Vec2> result;
	for (const T & t:v){
		result.push_back(b2Vec2(t.x, t.y));
	}
	return result;
}

template <typename T> inline
std::vector<cv::Point2f> cast_Point2f(const std::vector<T>& v){
	std::vector<cv::Point2f> result;
	for (const T & t:v){
		result.push_back(cv::Point2f(t.x, t.y));
	}
	return result;

}
// template <typename T>
// std::vector<cv::Point2f> set2vec_cv(std::set<T>);

template <typename T>
std::set<T> vec2set(std::vector<T> vec){
	std::set <T> set;
    for (T t:vec){
        set.emplace(t);
    }
    return set;
}

/**
 * @brief Given points, makes rotated bounding box
 * 
 * @param nb points
 * @return std::pair <bool, BodyFeatures> : <are features valid?, features>
 */
std::pair <bool, BodyFeatures> bounding_rotated_box(std::vector <cv::Point2f>nb);

template <typename Pt>
static b2PolygonShape sensor_box(const std::vector <Pt> &all_points_pt, b2Transform robot_pose, const Disturbance * dist){
	b2PolygonShape shape;
	b2Vec2 centroid(2.0, 2.0), center=centroid, center_local=b2Vec2_zero;
	float halfHeight=0, halfWidth=0;
	if (dist->isValid()){
	std::vector <b2Vec2>  d_vertices=dist->vertices(); 
	std::vector <cv::Point2f> all_points=cast_Point2f(all_points_pt);
	for (b2Vec2 p: d_vertices){
		p=b2MulT(robot_pose, p); //get local point
		all_points.push_back(cv::Point2f(p.x, p.y));
	}
	float minx=(std::min_element(all_points.begin(),all_points.end(), CompareX())).base()->x;
	float miny=(std::min_element(all_points.begin(), all_points.end(), CompareY())).base()->y;
	float maxx=(std::max_element(all_points.begin(), all_points.end(), CompareX())).base()->x;
	float maxy=(std::max_element(all_points.begin(), all_points.end(), CompareY())).base()->y;
	halfHeight=(fabs(maxy-miny))/2; //
	halfWidth=(fabs(maxx-minx))/2;
	center.x=maxx-halfWidth;
	center.y=maxy-halfHeight;
	centroid=center-center_local;  
	}
	shape.SetAsBox(halfWidth, halfHeight,centroid, 0);
	return shape;

}


#endif