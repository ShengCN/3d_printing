#include "slicer_experiment.h"
#include "geometry_ds.h"
#include "graphics_lib/common.h"
#include "graphics_lib/Render/mesh.h"
#include <glm/gtx/transform.hpp>

using namespace purdue;
slicer_experiment::slicer_experiment() {
}


slicer_experiment::~slicer_experiment() {
}

/* Test line segments speed */
std::vector<layer_paths> slicer_experiment::exp_speed_test(pd::deg turning_alpha, int seg_num) {
	std::vector<layer_paths> ret;

	vec3 speed_vector(0.0f, 0.0f, -1.0f), axis(0.0f);

	glm::mat4 left_rot = glm::rotate(pd::deg2rad(turning_alpha), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::mat4 right_rot = glm::rotate(-pd::deg2rad(turning_alpha), glm::vec3(0.0f, 1.0f, 0.0f));

	//------- Construct One Column Results --------//
	auto building_block = [](glm::mat4 left_rot, glm::mat4 right_rot, int seg_num, vec3 speed_vector) {
		vec3 cur_pos(0.0f);
		closed_poly cur_layer;
		for (int seg_i = 0; seg_i < seg_num; ++seg_i) {
			bool is_left = (seg_i % 2 == 0);
			speed_vector = is_left ?
				vec3(left_rot * vec4(speed_vector, 0.0)) :
				vec3(right_rot * vec4(speed_vector, 0.0));

			vec3 head = cur_pos;
			cur_pos = cur_pos + speed_vector;
			vec3 tail = cur_pos;
			cur_layer.push_back(std::make_shared<line_segment>(head, tail));
		}
		return cur_layer;
	};

	if (seg_num % 2 == 0)
		seg_num = seg_num - 1;

	closed_poly col_result = building_block(left_rot, right_rot, seg_num, speed_vector);
	vec3 diag = col_result[col_result.size() - 1]->t;
	
	vec3 col_rot_axis, offset;
	if(col_result.size() >= 2) {
		col_rot_axis = col_result[1]->h - col_result[0]->t;
		vec3 a = col_result[0]->t, b = col_result[0]->h, c = col_result[1]->h;
		offset = (2.0f * b - a - c);
	} else {
		WARN("line segment num is too small");
		assert(false);
	}

	 mat4 col_rot_mat = glm::rotate(pd::deg2rad(180.0), col_rot_axis);
	// rotate the first building block 
	closed_poly col_double_result = col_result;
	for(auto s: col_result) {
		col_double_result.push_back(
			std::make_shared<line_segment>(
			vec3(col_rot_mat * vec4(s->t, 0.0)),
			vec3(col_rot_mat * vec4(s->h, 0.0))));
	}

	layer_paths one_layer;
	int col_num =  50;
	for(int col_i = 0; col_i < col_num; ++col_i) {
		closed_poly cur_col;

		for(auto s:col_double_result) {
			cur_col.push_back(std::make_shared<line_segment>(
				s->t - offset * (float)col_i,
				s->h - offset * (float)col_i));
		}

		one_layer.push_back(cur_col);
	}


	ret.push_back({ one_layer });
	return ret;
}
