#pragma once
#include <vector>
#include <string>
#include "geometry_ds.h"

typedef std::vector<std::shared_ptr<line_segment>> closed_poly;
typedef std::vector<closed_poly> layer_paths;

/************************************************************************/
/* Base Class for Infill Path                                           */
/************************************************************************/
class infill_path {
public:
	infill_path() = default;

	//------- Interface --------//
	virtual void compute_path(std::vector<line_segment> boundary_loop,
					  std::vector<line_segment> &out_path) = 0;
};

// what slicer should do:
// 1. input a triangle mesh and a plane(vec3 p, vec3 n), compute intersection
// 2. generate slicing layer (should just be a line loop?)
// 3. given the boundary representation, infill the inner space(zigzag first)
// 4. generate g code?
// 5. generate infill patterns
// 6. generate support
class slicer {
public:
	slicer() = default;

	static bool write_layers(std::vector<layer_paths> all_layers, const std::string output_file);
};