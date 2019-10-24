#pragma once
#include "geometry_ds.h"

class slicer_experiment
{
public:
	slicer_experiment();
	~slicer_experiment();

	static std::vector<std::vector<std::shared_ptr<line_segment>>> exp_speed_test();
};

