#pragma once

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

	//------- Interface --------//
};