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
Pointf getPointf(T v){
	return Pointf(v.x, v.y);
}

// template <typename T>
// cv::Point2f getPoint2f(T);

Pointf Polar2f(float, float);

template <typename T>
std::vector<T> set2vec(std::set<T> s){
    std::vector <T> vec;
    for (T t:s){
        vec.emplace_back(t);
    }
    return vec;
}

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

/**
 * @brief Makes an upright bounding box around points
 * 
 * @tparam Pt template for point (Box2D, OpenCV or similar)
 * @param nb points
 * @return std::pair<bool,BodyFeatures> (is the object valid, object)
 */
template <class Pt>
std::pair<bool,BodyFeatures> bounding_box( std::vector <Pt >&nb){//gets bounding box of points
	float  l=(0.0005*2), w=(0.0005*2) ;
	float x_glob=0.0f, y_glob=0.0f;
	std::pair <bool, BodyFeatures> result(0, BodyFeatures());
	if (nb.empty()){
		return result;
	}
	CompareX compareX;
	CompareY compareY;
	typename std::vector<Pt>::iterator maxx=std::max_element(nb.begin(), nb.end(), compareX);
	typename std::vector<Pt>::iterator miny=std::min_element(nb.begin(), nb.end(), compareY);
	typename std::vector<Pt>::iterator minx=std::min_element(nb.begin(), nb.end(), compareX);
	typename std::vector<Pt>::iterator maxy=std::max_element(nb.begin(), nb.end(), compareY);
	if (minx->x!=maxx->x){
		w= fabs((*maxx).x-(*minx).x);
	}
	if (miny->y!=maxy->y){
		l=fabs((*maxy).y-(*miny).y);
	}
	x_glob= ((*maxx).x+(*minx).x)/2;
	y_glob= ((*maxy).y+(*miny).y)/2;
	result.second.halfLength=l/2;
	result.second.halfWidth=w/2;
	result.second.pose.p=b2Vec2(x_glob, y_glob);
	result.first=true;
	return result;
}


#endif