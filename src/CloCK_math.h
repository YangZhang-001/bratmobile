#include "task.h"

namespace math {

	void applyAffineTrans(const b2Transform& deltaPose, b2Transform& pose);

	void applyAffineTrans(const b2Transform&, State& );

	void applyAffineTrans(const b2Transform& , Task* );

	void applyAffineTrans(const b2Transform&, TransitionSystem&);

	void applyAffineTrans(const b2Transform&, Disturbance&);

	// b2Mat33 b2d_affine_matrix33(const b2Transform &); //returns a box2d object

	cv::Mat cv_affine_matrix33(const b2Transform &); //returns an opencv object

	b2Transform transform_2d(const cv::Mat&); //bets box2d 2dtransform from 3x3 matrix

	b2Transform solveAxB(const b2Transform& x, const b2Transform & B); //solve for A
};

/**
 * @brief Calculates transform between disturbance poses
 * 
 * @param result 
 * @param t_new transform of the new disturbance
 * @param t_tracked transform of tracked disturbance
 */
void calc_transform(b2Transform & result, b2Transform t_new, b2Transform t_prev);
