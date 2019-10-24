#include "slicer_experiment.h"
#include "geometry_ds.h"
#include "graphics_lib/common.h"

slicer_experiment::slicer_experiment() {
}


slicer_experiment::~slicer_experiment() {
}

/* Test line segments speed */
std::vector<std::vector<std::shared_ptr<line_segment>>> slicer_experiment::exp_speed_test() {
	float radius = 10.0f;
	int tessellation_min = 4, tessellation_max = 104;
	int tessellation_step = 10;
	
	auto scale_factor = [](float radius, int tessellation_num) {
		float perimeter = 2.0 * pd::pi * radius;
		pd::deg delta_angle = 360 / tessellation_num;
		vec3 c = vec3(radius, 0.0f, 0.0f);
		mat4 rot = glm::rotate(deg2rad(delta_angle), vec3(0.0f, 1.0f, 0.0f));
		vec3 b = vec3(rot * vec4(c,0.0f));
		vec3 a = b - c;

		float delta_length = glm::length(a);
		float delta_arc = deg2rad(delta_angle) * radius;
		return delta_arc / delta_length;
	};

	std::vector<std::vector<std::shared_ptr<line_segment>>> ret;
	for(int tes_num = tessellation_min; tes_num <= tessellation_max; tes_num += tessellation_step) {
		std::vector<std::shared_ptr<line_segment>> boundaries;

		float fract = scale_factor(radius, tes_num);
		INFO("Tes num: " + std::to_string(tes_num));
		INFO("Tes scale: " + std::to_string(fract));
		
		vec3 lst_vec = vec3(radius, 0.0f, 0.0f) * fract;
		vec3 cur_vec = vec3(radius, 0.0f, 0.0f) * fract;
		pd::deg delta_angle = 360.0 / tes_num;
		for(int i = 0; i < tes_num; ++i) {
			mat4 rot = glm::rotate(deg2rad(delta_angle), glm::vec3(0.0f, 1.0f, 0.0f));
			cur_vec = vec3(rot * vec4(cur_vec, 0.0f));
			
			boundaries.push_back(std::make_shared<line_segment>(lst_vec, cur_vec));
			lst_vec = cur_vec;
		}

		ret.push_back(boundaries);
	}

	return ret;
}
