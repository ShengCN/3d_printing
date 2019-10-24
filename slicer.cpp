#include <sstream>
#include <fstream>
#include "slicer.h"

bool slicer::write_layers(std::vector<layer_paths> all_layers, const std::string output_file) {
	std::ostringstream oss;
	
	int layer_counter = 0;
	for(auto &l:all_layers) {
		/* ignore some layer that only has two points(not become a surface) */
		if(l.size() <=2 ) {
			continue;
		}
		oss << "layer " << layer_counter++ << std::endl;
		/* Some closed polygons */
		for(auto &poly:l) {
			/* For each closed polygon */
			for (auto &line_seg : poly) {
				oss << line_seg->t.x << " " << line_seg->t.z << " ";
			}
			oss << poly[0]->t.x << " " << poly[0]->t.z << " b ";
		}
		oss << std::endl;
	}

	bool success = false;
	std::ofstream ofile(output_file);
	if (ofile.is_open()) {
		ofile << oss.str();
		success = true;
		INFO("File " + output_file + " saved");
	} else {
		LOG_FAIL("File " + output_file + " save");
	}
	ofile.close();

	return success;
}

