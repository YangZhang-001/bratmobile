#include "worldbuilder.h"

std::pair<Pointf, Pointf> WorldBuilder::bounds(Direction d, b2Transform start, float boxLength, float halfWindowWidth, std::vector <Pointf> *_bounds){
    //float halfWindowWidth=0.15; //wa .1
    std::pair <Pointf, Pointf>result;
    std::vector <Pointf> bds;
    if (d ==LEFT || d==RIGHT){
        boxLength =ROBOT_HALFLENGTH -ROBOT_BOX_OFFSET_X; //og 16 cm
        result.first =Pointf(start.p.x-boxLength, start.p.y-boxLength);
        result.second =Pointf(start.p.x+boxLength, start.p.y+boxLength);
        bds.push_back(result.first);
        bds.push_back(result.second);
        bds.push_back(Pointf(start.p.x+boxLength, start.p.y-boxLength));
        bds.push_back(Pointf(start.p.x-boxLength, start.p.y+boxLength));
    }
    else{
        Pointf positionVector, radiusVector, maxFromStart, top, bottom; 
        radiusVector = Polar2f(boxLength, start.q.GetAngle());
        maxFromStart = Pointf(start.p.x, start.p.y) + radiusVector;
        //FIND THE BOUNDS OF THE BOX
        b2Vec2 unitPerpR(-sin(start.q.GetAngle()), cos(start.q.GetAngle()));
        b2Vec2 unitPerpL(sin(start.q.GetAngle()), -cos(start.q.GetAngle()));
        bds.push_back(Pointf(start.p.x, start.p.y)+Pointf(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth));
        bds.push_back(Pointf(start.p.x, start.p.y)+Pointf(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
        bds.push_back(maxFromStart+Pointf(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth)); 
        bds.push_back(maxFromStart+Pointf(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
        CompareY compareY;
        std::sort(bds.begin(), bds.end(), compareY); //sort bottom to top
        result.first = bds[0]; //bottom 
        result.second = bds[3]; //top  
    }
    if (_bounds!=NULL){ //for debug
        *_bounds=bds;
    }
    return result;
    }

b2PolygonShape WorldBuilder::object_filtering_box(float halfWindowWidth, float boxLength, b2Transform start, Direction d){
    b2PolygonShape box;
    b2Vec2 centroid(0,0);
    if (d ==LEFT || d==RIGHT){
        halfWindowWidth =ROBOT_HALFLENGTH -ROBOT_BOX_OFFSET_X; //og 16 cm
        boxLength=halfWindowWidth*2;
    }
    else{
        // b2Vec2 shift(boxLength/2, 0);
 //       shift=b2Mul(start.q, shift);
   //     center+=shift
        centroid.x=boxLength/2;
    }
    box.SetAsBox(boxLength/2, halfWindowWidth, centroid, 0); //this  allows to filter out objects irrelevant to this task (i.e. collision unlikely assumint they're static)
    return box;
}



std::pair <bool, BodyFeatures> WorldBuilder::Bridger::bounding_rotated_box(std::vector <cv::Point2f>nb){
    for (cv::Point2f & p: nb){
        p.x= round(p.x*100)/100;
        p.y=round(p.y*100)/100;
    }
    std::pair <bool, BodyFeatures> result(0, BodyFeatures());
    if (nb.empty()){
        return result;
    }
    cv::RotatedRect rotated_rect = cv::minAreaRect(nb);
    if (rotated_rect.size.width>rotated_rect.size.height){
        result.second.setHalfLength(rotated_rect.size.width/2); //NVM THIS ///THEY ARE SWAPPED IN OPENCV DO NOT TOUCH
        result.second.setHalfWidth(rotated_rect.size.height/2);
        if (rotated_rect.angle>90){
            rotated_rect.angle-=90;
        }
        else{
            rotated_rect.angle+=90;
        }
    }
    else{
        result.second.setHalfLength(rotated_rect.size.height/2); //NVM THIS ///THEY ARE SWAPPED IN OPENCV DO NOT TOUCH
        result.second.setHalfWidth(rotated_rect.size.width/2);
    }
    result.second.pose.p=b2Vec2(rotated_rect.center.x, rotated_rect.center.y);
    float angle_rad=rotated_rect.angle*DEG_TO_RAD_K;
    result.second.pose.q.Set(angle_rad);
    result.second.pose.q.Set(atan(result.second.pose.q.s/result.second.pose.q.c));
    result.first=true;
    return result;
}


std::vector <std::vector<cv::Point2f>> WorldBuilder::kmeans_clusters( std::vector <cv::Point2f> points,std::vector <cv::Point2f> &centers){
    
    const int MAX_CLUSTERS=3, attempts =3, flags=cv::KMEANS_PP_CENTERS;
    cv::Mat bestLabels;
    cv::TermCriteria termCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0);
    cv::kmeans(points, MAX_CLUSTERS, bestLabels, termCriteria, attempts, flags, centers);
    std::vector <std::vector<cv::Point2f>>result(centers.size());

    for (int i=0; i<bestLabels.rows; i++){ //bestlabel[i] gives the index
            auto index=bestLabels.at<int>(i, 0);
            result[index].push_back(points[i]);
    }
    return result;
}


std::vector <std::vector<cv::Point2f>> WorldBuilder::partition_clusters( std::vector <cv::Point2f> points){
    struct Dist{ //define predicate
        bool operator()(const cv::Point2f& p1, const cv::Point2f& p2){
            float x_2=std::pow(p1.x-p2.x,2);
            float y_2=std::pow(p1.y-p2.y,2);
            return std::sqrt(x_2 +y_2)<.05;
        }
    }dist;
    std::vector <int> labels;
    cv::partition(points, labels, dist);
    int n_clusters= *(std::max_element(labels.begin(), labels.end())) +1;
    std::vector <std::vector<cv::Point2f>>result(n_clusters);
    for (int i=0; i<points.size(); i++){ //bestlabel[i] gives the index
        int label=labels[i];
        result[label].push_back(points[i]);
    }
    return result;
}

std::vector <BodyFeatures> WorldBuilder::cluster_data( const CoordinateContainer & pts, const b2Transform& start, CLUSTERING clustering){
    std::vector <BodyFeatures> result;
    std::vector <cv::Point2f> points, centers;
    for (const Pointf & p:pts){
        points.push_back(cv::Point2f(float(p.x), float(p.y)));
    }
    std::vector<std::vector<cv::Point2f>> clusters;
    if (clustering==KMEANS){
        clusters=kmeans_clusters(points, centers);
    }
    else if (clustering==PARTITION){
        clusters=partition_clusters(points);
    }
    for (int c=0; c<clusters.size(); c++){
        if (std::pair<bool,BodyFeatures>feature=wb_bridger.bounding_rotated_box(clusters[c]); feature.first){
            //feature.second.pose.q.Set(start.q.GetAngle());
            result.push_back(feature.second);
        }
    }
    return result;

}

b2Body* WorldBuilder::makeBody(b2World&w, BodyFeatures features){
	b2Body * body;
	b2BodyDef bodyDef;
	b2FixtureDef fixtureDef;
	bodyDef.type = features.bodyType;	
    bodyDef.position.Set(features.pose.p.x, features.pose.p.y);
    bodyDef.angle=features.pose.q.GetAngle(); 
	body = w.CreateBody(&bodyDef);
    switch(features.shape){
        case b2Shape::e_polygon: {
            b2PolygonShape fixture; 
            fixtureDef.shape = &fixture;             
            fixture.SetAsBox(features.halfWidth, features.halfLength); 
            body->CreateFixture(fixtureDef.shape, features.shift);
            break;
        }
        case b2Shape::e_edge:{ //straight edge
            b2EdgeShape fixture; 
            fixtureDef.shape = &fixture; 
            fixture.m_vertex1 =features.pose.p - b2Vec2(features.halfLength*features.pose.q.c, features.halfWidth*features.pose.q.s);
            fixture.m_vertex2 =features.pose.p + b2Vec2(features.halfLength*features.pose.q.c, features.halfWidth*features.pose.q.s);
	        body->CreateFixture(fixtureDef.shape, features.shift);
            break;
        }
        case b2Shape::e_circle:{
            b2CircleShape fixture;
            fixtureDef.shape = &fixture; 
            fixture.m_radius = features.halfLength;
	        body->CreateFixture(fixtureDef.shape, features.shift);
            break;
        }
        default:
        throw std::invalid_argument("not a valid shape\n");break;
    }
    if (features.attention){
        body->GetUserData().pointer=reinterpret_cast<uintptr_t>(DISTURBANCE_FLAG);
    }

	bodies++;
    return body;
}

std::pair <CoordinateContainer, bool> WorldBuilder::salientPoints(b2Transform start, const CoordinateContainer &current, std::pair<Pointf, Pointf> bt){
    std::pair <CoordinateContainer, bool> result(CoordinateContainer(), 0);
    float qBottomH=0, qTopH=0, qBottomP=0, qTopP=0, mHead=0, mPerp=0, ceilingY=0, floorY=0, frontX=0, backX=0;
    if (sin(start.q.GetAngle())!=0 && cos(start.q.GetAngle())!=0){
        //FIND PARAMETERS OF THE LINES CONNECTING THE a
        mHead = sin(start.q.GetAngle())/cos(start.q.GetAngle()); //slope of heading direction
        mPerp = -1/mHead;
        qBottomH = bt.first.y - mHead*bt.first.x;
        qTopH = bt.second.y - mHead*bt.second.x;
        qBottomP = bt.first.y -mPerp*bt.first.x;
        qTopP = bt.second.y - mPerp*bt.second.x;
        for (Pointf p: current){
            ceilingY = mHead*p.x +qTopH;
			floorY = mHead*p.x+qBottomH;
			float frontY= mPerp*p.x+qBottomP;
			float backY = mPerp*p.x+qTopP;
            if (p.y >=floorY && p.y<=ceilingY && p.y >=frontY && p.y<=backY){
                result.first.insert(p);
            }
        }

    }
    else{
        ceilingY = std::max(bt.second.y, bt.first.y); 
        floorY = std::min(bt.second.y, bt.first.y); 
        frontX = std::min(bt.second.x, bt.first.x);
        backX = std::max(bt.second.x, bt.first.x);
        for (Pointf p: current){
            if (p.y >=floorY && p.y<=ceilingY && p.x >=frontX && p.x<=backX){
                result.first.insert(p);
            }  
        }
    }
    return result;
}


std::vector <BodyFeatures> WorldBuilder::getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering){
    std::vector <BodyFeatures> features;
   // std::pair<Pointf, Pointf> bt = bounds(d, start, boxLength, halfWindowWidth);
    //std::pair <CoordinateContainer, bool> salient = salientPoints(start,current, bt);
    if (current.empty()){
        return features;
    }
    if (clustering==BOX){
        features =processData(current, start);
    }
    else{
        features=cluster_data(current, start,clustering);
    }
    return features;
}



 void WorldBuilder::buildWorld(b2World& world, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task){
    float boxLength=simulationStep-ROBOT_BOX_OFFSET_X;
    std::vector <cv::Point2f> points_to_track, *pointer_to_track;
    if (NULL!=task){
        pointer_to_track=&points_to_track;
    }
    bool has_D=false;
    if (disturbance.getAffIndex()==AVOID && d==DEFAULT){
        std::vector <b2Vec2> d_vertices=disturbance.vertices();
        for (b2Vec2 &v: d_vertices){
            v=v-start.p; //get distance of each vertex of disturbance from start of task
        }
        auto maxx_it= std::max_element(d_vertices.begin(), d_vertices.end(), CompareX());
        float maxx= maxx_it.base()->x; //furthest D vertex from robot
        boxLength=ROBOT_HALFLENGTH*2-ROBOT_BOX_OFFSET_X+maxx;
    }
   // std::vector <BodyFeatures> features =getFeatures(current, start, d, boxLength, halfWindowWidth, clustering);
    b2PolygonShape filter_box=object_filtering_box(halfWindowWidth, boxLength,start, d);
    for (BodyFeatures f: world_objects){
        b2PolygonShape feature_shape;
        feature_shape.SetAsBox(f.halfWidth, f.halfLength);
        if (b2TestOverlap(&filter_box, 0, &feature_shape, 0, start, f.pose)){
            makeBody(world, f);
        }
    }
    int _count=world.GetBodyCount();
	FILE *file;
	if (DEBUG){
		file = fopen(bodyFile, "a+");
		for (b2Body * b = world.GetBodyList(); b!=NULL; b= b->GetNext()){
			fprintf(file, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
		}
		fclose(file);
	}
}

bool WorldBuilder::checkDisturbance(Pointf p, bool& obStillThere, Task * curr, float range){
    bool result=0;
	if (NULL!=curr){ //
        if (!curr->disturbance.isValid()){
            return result;
        }
        cv::Rect2f rect(curr->disturbance.getPosition().x-range, curr->disturbance.getPosition().y+range, range*2, range*2);
		if (p.inside(rect)){
			obStillThere =1;
            result =1;
		}
	}
    return result;
}

b2Vec2 averagePoint(const CoordinateContainer & c, Disturbance & d, float rad = 0.025){
    b2Vec2 result(0,0), centroid(0,0);
    for (const Pointf & p: c){
       centroid.x+=p.x;
       centroid.y +=p.y; 
    }
    centroid.x/=c.size();
    centroid.y/=c.size();
    result = d.getPosition()- centroid;
    d.setPosition(centroid);
    return result;
}

void WorldBuilder::world_cleanup(b2World * world){
    int ct=world->GetBodyCount();
	for (b2Body * b = world->GetBodyList(); b; b = b->GetNext()){
		world->DestroyBody(b);
	}
}

b2Body * WorldBuilder::get_robot(b2World * world){
    for (b2Body * b=world->GetBodyList();b; b=b->GetNext()){
        if (b->GetUserData().pointer==ROBOT_FLAG){
            return b;
        }
    }
    return NULL;

}

b2Fixture * WorldBuilder::get_chassis(b2Body * r){
    if (r->GetUserData().pointer!=ROBOT_FLAG){
        return NULL;
    }
    for (b2Fixture * f=r->GetFixtureList(); f; f=f->GetNext()){
        if (!f->IsSensor()){
            return f;
        }
    }
    return NULL;

}

b2AABB WorldBuilder::makeRobotSensor(b2Body* robotBody, Disturbance * goal){
	b2AABB result;
    if (!goal->isValid()){
        return result;
    }
	b2PolygonShape * poly_robo=(b2PolygonShape*)robotBody->GetFixtureList()->GetShape();
    std::vector <b2Vec2> robot_vertices=arrayToVec(poly_robo->m_vertices, poly_robo->m_count);
    b2PolygonShape shape=sensor_box(robot_vertices,  robotBody->GetTransform(), goal);
    b2Vec2 local_robot=robotBody->GetLocalPoint(robotBody->GetPosition());
	b2FixtureDef fixtureDef;
	fixtureDef.isSensor=true;
	fixtureDef.shape=&shape;
	robotBody->CreateFixture(&fixtureDef);
	shape.ComputeAABB(&result, robotBody->GetTransform(), 0);
	return result;
	
}



cv::Rect2f WorldBuilder::Bridger::real_world_focus(const Task * t){
    std::vector <cv::Point2f> vertices;
    if (t->disturbance.getAffIndex()==NONE){
        return cv::Rect2f(0, 0, 0, 0);
    }
    cv::Point2f bl; //bottom left (documentation CV says top left but not true)
    float max_dimension=std::max(t->disturbance.bf.width(), t->disturbance.bf.length());
    max_dimension+=0.02;    
    bl.x=t->disturbance.pose().p.x-(max_dimension/2);
    bl.y=t->disturbance.pose().p.y-(max_dimension/2);
    cv::Rect2f focus(bl.x, bl.y, max_dimension, max_dimension);
    return focus; //upright bounding rectangle: increases area represented
}


b2Transform WorldBuilder::Bridger::get_transform(const Task & t, const CoordinateContainer & pts, BodyFeatures * observed_disturbance){
   if (t.disturbance.getAffIndex()==NONE || t.disturbance.bf.area()<0.0005){
        tracked_disturbance=Disturbance();
        return t.action.getTransform(LIDAR_SAMPLING_RATE);
    }
    cv::Rect2f focus=real_world_focus(&tracked_disturbance);
    std::vector <cv::Point2f> focus_points;
    cv::Point2f tr=focus.br();
    for (auto p: pts){
        cv::Point2f p_cv=cv::Point2f(p.x, p.y);
        if (focus.contains(p_cv)){
            focus_points.push_back(p_cv);
        }
    }
    std::pair <bool, BodyFeatures> new_d=bounding_rotated_box(focus_points);
    tracked_disturbance=Disturbance(new_d.second);
    if (!new_d.first){
        return t.action.getTransform(LIDAR_SAMPLING_RATE);
    }
    b2Transform mulT=b2MulT(t.disturbance.pose(), new_d.second.pose);
//what to do when different dimensions??
    if (observed_disturbance!=NULL && new_d.first){
        *observed_disturbance=new_d.second;
    }
    if (new_d.second.match(t.disturbance.bf)){
        new_d.second.halfWidth=t.disturbance.bf.halfWidth;
        new_d.second.halfLength=t.disturbance.bf.halfLength;
    }
    return mulT;
    //what's the most likely angle??
}

