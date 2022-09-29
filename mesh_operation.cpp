#include <algorithm>
#include <queue>
#include <set>

#include <igl/triangle/triangulate.h>

#include "mesh_operation.h"
#include "graphics_lib/Utilities/Utils.h"
#include "graphics_lib/Utilities/Logger.h"

using glm::dot;
using namespace mesh_opt;
using namespace purdue;

AABB mesh_opt::compute_world_aabb(std::vector<std::shared_ptr<mesh>> meshes) {
	if (meshes.empty())
		return AABB(vec3(0.0f));

	AABB ret = meshes[0]->compute_world_aabb();
	for(size_t mi = 1; mi < meshes.size(); ++mi) {
		ret.add_aabb(meshes[mi]->compute_world_aabb());
	}
	return ret;
}

bool mesh_opt::point_upper_plane(vec3 pp, std::shared_ptr<plane> p) {
	return dot(pp - p->p, p->n) >= 0.0f;
}

bool mesh_opt::points_same_side(vec3 p1, vec3 p2, std::shared_ptr<plane> p) {
	return dot(p1 - p->p, p->n) * dot(p2 - p->p, p->n) >= 0.0f;
}

// t is parameter of interpolation from tail to head
// t = 0 -> tail
// t = 1 -> head
bool mesh_opt::line_segment_plane_intersection(std::shared_ptr<line_segment> ls, std::shared_ptr<plane> p, float& t) {
	float a = (dot(p->p, p->n) - dot(ls->t, p->n));
	float b = (dot(ls->h, p->n) - dot(ls->t, p->n));
	t =  a/b;
	return t <= 1.0f && t >= 0.0f;
}

tp_intersection_type mesh_opt::triangle_plane_intersect(std::shared_ptr<triangle> t, std::shared_ptr<plane> p,
											  std::shared_ptr<line_segment> intersections, std::vector<vec3>& bary_centric) {
	if(!t || !p || !intersections) {
		WARN("shared_ptr input");
		return tp_intersection_type::unknown;
	}
	
	bary_centric.clear();
	bool is_ab_same_side = points_same_side(t->a, t->b, p);
	bool is_ac_same_side = points_same_side(t->a, t->c, p);
	bool is_bc_same_side = points_same_side(t->b, t->c, p);
	if (is_ab_same_side && is_ac_same_side) {
		return dot(t->a, p->n) >= 0.0f ? tp_intersection_type::upper : tp_intersection_type::bottom;
	}

	vec3 same_side_p1, same_side_p2, other_p;
	float p1_t, p2_t;
	vec3 t_bary, h_bary;
	if(is_ab_same_side) {
		same_side_p1 = t->a;
		same_side_p2 = t->b;
		other_p = t->c;

		line_segment_plane_intersection(std::make_shared<line_segment>(same_side_p1, other_p), p, p1_t);
		line_segment_plane_intersection(std::make_shared<line_segment>(same_side_p2, other_p), p, p2_t);
		t_bary = vec3(1.0 - p1_t, 0.0f, p1_t);
		h_bary = vec3(0.0f, 1.0 - p2_t, p2_t);
	} 
	else if(is_ac_same_side) {
		same_side_p1 = t->c;
		same_side_p2 = t->a;
		other_p = t->b;
		line_segment_plane_intersection(std::make_shared<line_segment>(same_side_p1, other_p), p, p1_t);
		line_segment_plane_intersection(std::make_shared<line_segment>(same_side_p2, other_p), p, p2_t);

		t_bary = vec3(0.0f, p1_t, 1.0 - p1_t);
		h_bary = vec3(1.0 - p2_t, p2_t, 0.0f);
	} else if(is_bc_same_side) {
		same_side_p1 = t->b;
		same_side_p2 = t->c;
		other_p = t->a;
		line_segment_plane_intersection(std::make_shared<line_segment>(same_side_p1, other_p), p, p1_t);
		line_segment_plane_intersection(std::make_shared<line_segment>(same_side_p2, other_p), p, p2_t);

		t_bary = vec3(p1_t, 1.0 - p1_t, 0.0f);
		h_bary = vec3(p2_t, 0.0f, 1.0 - p2_t);
	}

	// std::cerr << p1_t << " " << p2_t << std::endl;
	intersections->t = pd::lerp(same_side_p1, other_p, p1_t);
	intersections->h = pd::lerp(same_side_p2, other_p, p2_t);
	
	// clamp to deal with end points
	t_bary = vec3(t_bary.x, t_bary.y, t_bary.z);
	h_bary = vec3(h_bary.x, h_bary.y, h_bary.z);

	bary_centric.push_back(t_bary);
	bary_centric.push_back(h_bary);

	return tp_intersection_type::intersect;
}

bool loop_line_segments(const std::vector<std::shared_ptr<line_segment>> &unsorted_ls, std::vector<std::shared_ptr<line_segment>> &sorted_ls) {
	if (unsorted_ls.empty())
		return false;
	
	// sort line segments into a loop
	std::vector<std::shared_ptr<line_segment>> sorted_queue;
	std::set<std::shared_ptr<line_segment>> un_sorted_set;

	for (auto &s : unsorted_ls) {
		un_sorted_set.insert(s);
	}

	/* Important Assumption: One whole */
	while(!un_sorted_set.empty()) {
		std::shared_ptr<line_segment> first_seg = *un_sorted_set.begin();
		std::shared_ptr<line_segment> cur_seg = first_seg;
		un_sorted_set.erase(cur_seg);
		sorted_queue.push_back(cur_seg);

		while (!pd::same_point(cur_seg->h, first_seg->t)) {
			bool is_found = false;
			for (auto &ls : un_sorted_set) {
				if (pd::same_point(cur_seg->h, ls->t)) {
					cur_seg = ls;
					un_sorted_set.erase(ls);
					sorted_queue.push_back(cur_seg);

					is_found = true;
					break;
				}

				if (pd::same_point(cur_seg->h, ls->h)) {
					cur_seg = std::make_shared<line_segment>(ls->h, ls->t);
					un_sorted_set.erase(ls);
					sorted_queue.push_back(cur_seg);

					is_found = true;
					break;
				}
			}

			if (!is_found) {
				INFO(cur_seg->to_string() + " Finding Loop");
				INFO("Remaining: ");
				for (auto &s : un_sorted_set) {
					INFO(s->to_string());
				}
                return false;
				// assert(false);
				break;
			}
		}
	}
	sorted_ls = sorted_queue;
	return true;
}

bool mesh_opt::slice_mesh(std::shared_ptr<mesh> m,
				std::shared_ptr<plane> p, 
				std::vector<std::shared_ptr<line_segment>>& segments, 
				std::shared_ptr<mesh> upper, 
				std::shared_ptr<mesh> bottom) {
    bool is_intersect = false;
	segments.clear();

	if(!m || !p || !upper || !bottom) {
		WARN("input of slice mesh");
		return false;
	}

	upper->clear_vertices();
	bottom->clear_vertices();
	auto world_verts = m->compute_world_space_coords();

	for(int ti = 0; ti < world_verts.size()/3; ++ti) {
		size_t a_ind = 3 * ti + 0;
		size_t b_ind = 3 * ti + 1;
		size_t c_ind = 3 * ti + 2;

		std::shared_ptr<triangle> cur_triangle = std::make_shared<triangle>(world_verts[a_ind],
																			world_verts[b_ind],
																			world_verts[c_ind]);
		std::shared_ptr<line_segment> intersection = std::make_shared<line_segment>();
		std::vector<vec3> bary_centric;
		switch (triangle_plane_intersect(cur_triangle, p, intersection, bary_centric)) {
		case tp_intersection_type::intersect:
			{
				is_intersect = true;
				assert(intersection != nullptr);
				segments.push_back(intersection);

				// generate new triangles
				vec3 d = bary_centric_interpolate(cur_triangle->a, cur_triangle->b, cur_triangle->c, bary_centric[0]);
				vec3 e = bary_centric_interpolate(cur_triangle->a, cur_triangle->b, cur_triangle->c, bary_centric[1]);
				vec3 dn = bary_centric_interpolate(m->m_norms[a_ind], m->m_norms[b_ind], m->m_norms[c_ind], bary_centric[0]);
				vec3 en = bary_centric_interpolate(m->m_norms[a_ind], m->m_norms[b_ind], m->m_norms[c_ind], bary_centric[1]);
				vec3 dc = bary_centric_interpolate(m->m_colors[a_ind], m->m_colors[b_ind], m->m_colors[c_ind], bary_centric[0]);
				vec3 ec = bary_centric_interpolate(m->m_colors[a_ind], m->m_colors[b_ind], m->m_colors[c_ind], bary_centric[1]);

				vec3 bary_prod = bary_centric[0] * bary_centric[1];
				// std::cerr << bary_prod << std::endl;

				vec3 same_side_pp0, same_side_pn0, same_side_pc0;
				vec3 same_side_pp1, same_side_pn1, same_side_pc1;
				vec3 other_side_pp, other_side_pn, other_side_pc;

				// a, bc
				if(!float_equal(bary_prod.x, 0.0f)) {
					other_side_pp = cur_triangle->a, other_side_pn = m->m_norms[a_ind], other_side_pc = m->m_colors[a_ind];
					same_side_pp0 = cur_triangle->b, same_side_pn0 = m->m_norms[b_ind], same_side_pc0 = m->m_colors[b_ind];
					same_side_pp1 = cur_triangle->c, same_side_pn1 = m->m_norms[c_ind], same_side_pc1 = m->m_colors[c_ind];
				}

				// b, ca
				else if (!float_equal(bary_prod.y, 0.0f)) {
					other_side_pp = cur_triangle->b; other_side_pn = m->m_norms[b_ind]; other_side_pc = m->m_colors[b_ind];
					same_side_pp0 = cur_triangle->c; same_side_pn0 = m->m_norms[c_ind]; same_side_pc0 = m->m_colors[c_ind];
					same_side_pp1 = cur_triangle->a; same_side_pn1 = m->m_norms[a_ind]; same_side_pc1 = m->m_colors[a_ind];
				}

				// c, ab
				else if (!float_equal(bary_prod.z, 0.0f)) {
					other_side_pp = cur_triangle->c; other_side_pn = m->m_norms[c_ind]; other_side_pc = m->m_colors[c_ind];
					same_side_pp0 = cur_triangle->a; same_side_pn0 = m->m_norms[a_ind]; same_side_pc0 = m->m_colors[a_ind];
					same_side_pp1 = cur_triangle->b; same_side_pn1 = m->m_norms[b_ind]; same_side_pc1 = m->m_colors[b_ind];
				}

				if (point_upper_plane(other_side_pp, p)) {
					upper->add_face(d, e, other_side_pp);

					bottom->add_face(d, same_side_pp0, same_side_pp1);
					bottom->add_face(same_side_pp1, e, d);
				}
				else {
					bottom->add_face(d, e, other_side_pp);

					upper->add_face(d, same_side_pp0, same_side_pp1);
					upper->add_face(same_side_pp1, e, d);
				}

				break;
			}

		case tp_intersection_type::upper:
			{
				assert(m->m_norms.size() > 0);
				assert(m->m_colors.size() > 0);

				upper->add_face(cur_triangle->a, cur_triangle->b, cur_triangle->c);
				break;
			}
		case tp_intersection_type::bottom:
			{
				assert(m->m_norms.size() > 0);
				assert(m->m_colors.size() > 0);

				bottom->add_face(cur_triangle->a, cur_triangle->b, cur_triangle->c);
				break;
			}
		default:
			break;
		}
	}

	std::vector<std::shared_ptr<line_segment>> sorted_queue;
	loop_line_segments(segments, sorted_queue);
	segments = sorted_queue;
	/* upper and bottom needs to be closed mesh, so both should have the triangle that lie on intersection plane */
	if(segments.size() > 2) {
		/* Find correct loop orientation */
		vec3 a = sorted_queue[0]->h - sorted_queue[0]->t;
		vec3 b = sorted_queue[1]->h - sorted_queue[1]->t;

		//------- use libigl to triangulate --------//
		/* line segments to libigl ds */
		// Input polygon
		Eigen::MatrixXd V;
		Eigen::MatrixXi E;
		Eigen::MatrixXd H;

		// Triangulated interior
		Eigen::MatrixXd V2;
		Eigen::MatrixXi F2;

		V.resize(segments.size(), 2);
		E.resize(segments.size(), 2);
		for(int i = 0; i < segments.size(); ++i) {
			/* Since we are 3D printing, assumption is the plane is horizontal */
			V(i, 0) = segments[i]->t.x;
			V(i, 1) = segments[i]->t.z;
			E(i, 0) = i;
			E(i, 1) = (i + 1)%((int)segments.size());
		}

        igl::triangle::triangulate(V, E, H, "q", V2, F2);
        // igl::triangle::triangulate(V,E,H,"a0.005q",V2,F2);

		int row = F2.rows();
		for(int ri = 0; ri < row; ++ri) {
			vec3 a = vec3(V2(F2(ri, 0), 0), segments[0]->t.y, V2(F2(ri, 0), 1));
			vec3 b = vec3(V2(F2(ri, 1), 0), segments[0]->t.y, V2(F2(ri, 1), 1));
			vec3 c = vec3(V2(F2(ri, 2), 0), segments[0]->t.y, V2(F2(ri, 2), 1));

			if(glm::dot(glm::cross(b-a, c-b), vec3(0.0f,1.0f,0.0f)) >= 0.0f) {
				bottom->add_face(a, b, c);
				upper->add_face(c, b, a);
			} else {
				bottom->add_face(c, b, a);
				upper->add_face(a, b, c);
			}
		}
	}

	return is_intersect;
}

bool mesh_opt::slice_mesh(std::shared_ptr<mesh> m,
                          std::shared_ptr<plane> p,
                          std::vector<std::shared_ptr<line_segment>>& segments) {
	bool is_intersect = false;
	segments.clear();

	if (!m || !p) {
		WARN("input of slice mesh");
		return false;
	}

	auto world_verts = m->compute_world_space_coords();

	for (int ti = 0; ti < world_verts.size() / 3; ++ti) {
		size_t a_ind = 3 * ti + 0;
		size_t b_ind = 3 * ti + 1;
		size_t c_ind = 3 * ti + 2;

		std::shared_ptr<triangle> cur_triangle = std::make_shared<triangle>(world_verts[a_ind],
																			world_verts[b_ind],
																			world_verts[c_ind]);
		std::shared_ptr<line_segment> intersection = std::make_shared<line_segment>();
		std::vector<vec3> bary_centric;
		if(triangle_plane_intersect(cur_triangle, p, intersection, bary_centric) == tp_intersection_type::intersect) {
			is_intersect = true;
			segments.push_back(intersection);
		}
	}

	loop_line_segments(segments, segments);
	return is_intersect;
}

void mesh_opt::eigen_to_mesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F, std::shared_ptr<mesh> m) {
    if (m == nullptr) {
        ERROR("Input null");
        return;
    }
    size_t fn = F.rows();

    m->clear_vertices();
    for (int ti = 0; ti < fn; ++ti) {
        vec3 p0(V(F(ti, 0), 0), V(F(ti, 0), 1), V(F(ti, 0), 2));
        vec3 p1(V(F(ti, 1), 0), V(F(ti, 1), 1), V(F(ti, 1), 2));
        vec3 p2(V(F(ti, 2), 0), V(F(ti, 2), 1), V(F(ti, 2), 2));

        m->add_face(p0, p1, p2);
    }
}


/*
 * Note, we assume the vertices are in the world space
 */
void mesh_opt::mesh_to_eigen(std::shared_ptr<mesh> m, Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
    if (m == nullptr) {
        ERROR("Input null");
        return;
    }

    std::vector<glm::vec3> world_verts = m->compute_world_space_coords();
    size_t vert_n = world_verts.size();
    size_t face_n = vert_n / 3;

    V = Eigen::MatrixXd(vert_n, 3);
    F = Eigen::MatrixXi(face_n, 3);

    for (int ti = 0; ti < face_n; ++ti) {
        V.row(3 * ti + 0) << world_verts[3 * ti + 0].x, world_verts[3 * ti + 0].y, world_verts[3 * ti + 0].z;
        V.row(3 * ti + 1) << world_verts[3 * ti + 1].x, world_verts[3 * ti + 1].y, world_verts[3 * ti + 1].z;
        V.row(3 * ti + 2) << world_verts[3 * ti + 2].x, world_verts[3 * ti + 2].y, world_verts[3 * ti + 2].z;

        F.row(ti) << 3 * ti + 0, 3 * ti + 1, 3 * ti + 2;
    }
}

