#include "slicer_experiment.h"
#include "geometry_ds.h"
#include "graphics_lib/common.h"


slicer_experiment::slicer_experiment() {
}


slicer_experiment::~slicer_experiment() {
}

/* Test line segments speed */
std::vector<layer_paths> slicer_experiment::exp_speed_test(pd::deg turning_alpha, int seg_num) {
	std::vector<layer_paths> ret;

	vec3 speed_vector(0.0f, 0.0f, -1.0f), axis(0.0f);

	glm::mat4 left_rot = glm::rotate(deg2rad(turning_alpha), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::mat4 right_rot = glm::rotate(-deg2rad(turning_alpha), glm::vec3(0.0f, 1.0f, 0.0f));

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

	closed_poly col_result = building_block(left_rot, right_rot, seg_num, speed_vector);
	vec3 diag = col_result[col_result.size() - 1]->t;
	vec3 offset = glm::normalize(vec3(glm::rotate(deg2rad(90.0f), vec3(0.0f, 1.0f, 0.0f)) * diag));

	int col_num = 10;
	layer_paths one_layer;
	for(int j =0;j < col_num; ++j) {
		closed_poly cur_col;

		for(auto s:col_result) {
			cur_col.push_back(std::make_shared<line_segment>(
				s->t + offset * (float)j, 
				s->h + offset * (float)j));
		}

		one_layer.push_back(cur_col);
	}

	ret.push_back({ one_layer });
	return ret;
}
