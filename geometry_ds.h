#pragma once
#include <sstream>
#include "graphics_lib/common.h"
#include "graphics_lib/Utilities/Utils.h"
#include "graphics_lib/Utilities/Logger.h"

struct triangle {
	vec3 a, b, c;
	triangle() :a(vec3(0.0f)), b(vec3(0.0f)), c(vec3(0.0f)) {}
	triangle(vec3 a, vec3 b, vec3 c):a(a), b(b),c(c) {}	
};

struct line_segment{
	vec3 t, h;	// tail, head
	line_segment():t(vec3(0.0f)), h(vec3(0.0f)) {}
	line_segment(vec3 t, vec3 h) :t(t), h(h) {}

	std::string to_string() {
		std::ostringstream oss;
		oss << "tail: " << purdue::to_string(t);
		oss << " head: " << purdue::to_string(h);
		return oss.str();
	}
};

struct plane {
	vec3 p, n;	// point p, normal m
	plane(): p(vec3(0.0f)), n(vec3(0.0f,1.0f,0.0f)) {}
	plane(vec3 p, vec3 n) : p(p), n(n) {}
};

inline bool same_line_segment(std::shared_ptr<line_segment> a, std::shared_ptr<line_segment> b) {
	return purdue::same_point(a->t, b->t) && purdue::same_point(a->h, b->h);
}

inline bool left_turn(vec3 &a, vec3 &b, vec3 &c) {
	mat3 m;
	m[0] = a - b;
	m[1] = c - b;
	m[2] = vec3(1.0f);
	return glm::determinant(m) >= 0.0f;
}