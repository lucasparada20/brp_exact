#ifndef LOAD_BRP
#define LOAD_BRP

#include "json.hpp"
#include "ProblemDefinition.h"
#include "RouteFeasibility.h"

class LoadBRP
{
	public:
		
		void Load_slr_instance(Prob & pr, char * filename);
		double CalculateHarvesineDistance(Node * n1, Node * n2);
		void Load_coord(Prob & pr, const char * filename);
		void Load_targets(Prob & pr, const char * filename);
		void Load_json_station_status(Prob & pr, const char * filename);
		
		void Load_brp_instance(Prob & pr, const char * filename); // To load DHIN instances

		std::map<std::string,int> bss_id_map;		
		
};

#endif