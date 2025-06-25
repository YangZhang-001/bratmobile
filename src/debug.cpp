 #include "debug.h"

std::string Logger::file_dateTime(const char* custom, char name[60]){
	time_t now =time(0);
	tm *ltm = localtime(&now);
	int y,m,d, h, min;
	y=ltm->tm_year-100;
	m = ltm->tm_mon +1;
	d=ltm->tm_mday;
	h= ltm->tm_hour;
	min = ltm->tm_min;
	sprintf(name, "%s_%02i%02i%02i_%02i%02i",custom, d,m,y,h,min);
	return std::string(name);
}

bool Logger::fprintf(const char * format, ...){
	va_list args;
	va_start(args, format);
	vfprintf(f, format, args);
	va_end(args);
	fflush(f);
}


b2Vec2 GetWorldPoints(b2Body* b, b2Vec2 v){
	b2Vec2 wp=b->GetWorldPoint(v);
	printf("x=%f, y=%f\t", wp.x, wp.y);
}

char* debug::print_pose(const b2Transform& p, char* msg){
	if (NULL!=msg){
		printf("%s\t", msg);
	}
	char str[256];
	sprintf(str,"x=%f, y=%f, theta=%f", p.p.x, p.p.y, p.q.GetAngle());
	printf("%s\n", str);
}

void debug::print_matrix(const cv::Mat & m){
	std::cout << "M = " << std::endl << " "  << m << std::endl << std::endl;
}

std::vector<b2Vec2> debug::GetBodies(b2World* w){
	std::vector<b2Vec2> result;
	for (b2Body * b=w->GetBodyList(); b; b=b->GetNext()){
		result.push_back(b->GetPosition());
	}
	return result;
}

void debug::print_state_difference(const StateDifference & sd, vertexDescriptor v, vertexDescriptor v1){
	if (v==TransitionSystem::null_vertex()){
		printf("no match no sd");
		return;
	}
	printf("STATE DIFFERENCE between %i and %i\n", v, v1);
	print_pose(sd.Di.pose, "Di pose");
	printf("Di width=%f, Di length=%f", sd.Di.halfWidth, sd.Di.halfLength);
	print_pose(sd.Di.pose, "Dn pose");
	printf("Dn width=%f, Dn length=%f", sd.Dn.halfWidth, sd.Dn.halfLength);

}