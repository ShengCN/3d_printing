#pragma once
#include <vector>

#include "graphics_lib/Render/mesh.h"
#include "geometry_ds.h"

namespace mesh_opt {

	/* Compute meshes' aabb */
	AABB compute_world_aabb(std::vector<std::shared_ptr<mesh>> meshes);

	bool point_upper_plane(vec3 pp, std::shared_ptr<plane> p);
	bool points_same_side(vec3 p1, vec3 p2, std::shared_ptr<plane> p);

	bool line_segment_plane_intersection(std::shared_ptr<line_segment> ls,
										 std::shared_ptr<plane> p,
										 float& t);

	//------- Triangle-Plane Intersection Type --------//
	enum tp_intersection_type {
		intersect,
		upper,		// triangle is upper side of the plane according to normal
		bottom,
		unknown,
	};
	tp_intersection_type triangle_plane_intersect(std::shared_ptr<triangle> t,
												  std::shared_ptr<plane> p,
												  std::shared_ptr<line_segment> intersections,
												  std::vector<vec3>& bary_centric);	// intersections' bary_centric coordinates

	// given a mesh and plane parameter
	// slice the mesh to two meshes
	bool slice_mesh(std::shared_ptr<mesh> m, std::shared_ptr<plane> p,
					std::vector<std::shared_ptr<line_segment>>& segments,
					std::shared_ptr<mesh> upper, std::shared_ptr<mesh> bottom);

	bool slice_mesh(std::shared_ptr<mesh> m, std::shared_ptr<plane> p,
					std::vector<std::shared_ptr<line_segment>>& segments);

	void merge_mesh(std::vector<std::shared_ptr<mesh>> meshes, std::shared_ptr<mesh> out);
};