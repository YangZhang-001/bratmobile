#ifndef BOX2D_HELPERS_H
#define BOX2D_HELPERS_H
#include "box2d/box2d.h"
#include <ostream>
#include <vector>

/**
 * @brief Searches vector or a certain item
 * 
 * @param vector 
 * @param item 
 * @return auto iterator to element
 */
template <class I>
auto check_vector_for(std::vector <I>& vector, const I& item){
	for (int i=0; i<vector.size(); i++){
		if (vector[i]==item){
			return vector.begin()+i;
		}
	}
	return vector.end();
}

float angle_subtract(float a1, float a2);


typedef b2Transform Transform;
bool operator!=(Transform const &, Transform const &);
bool operator==(Transform const &, Transform const &);
void operator-=(Transform &, Transform const&);
void operator+=(Transform &, Transform const&);
Transform operator+( Transform const &, Transform const &);
Transform operator-( Transform const &, Transform const &);
Transform operator-(Transform const &);




#endif