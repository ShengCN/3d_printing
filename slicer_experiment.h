#pragma once
#include "slicer.h"
#include "geometry_ds.h"

class slicer_experiment
{
public:
	slicer_experiment();
	~slicer_experiment();

	static std::vector<layer_paths> exp_speed_test(pd::deg turning_alpha, int seg_num);
};

