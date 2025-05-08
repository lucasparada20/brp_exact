#ifndef ROUTE_FEASIBILITY
#define ROUTE_FEASIBILITY

#include "ProblemDefinition.h"
#include "NodeBRP.h"
#include "DriverBRP.h"
#include <vector>
#include <algorithm>

class RouteFeasibility
{
	public:
	
		static double Calculate(Prob * prob);
		static double Calculate(Prob* prob, std::vector<Node*>& stations);
		static double CalculateWithMinDriverCount(Prob* prob);
		static double CalculateWithMinDriverCount(Prob* prob, std::vector<Node*>& stations);		
		
		static bool IsFeasible(Prob * prob, std::vector<Node*> & path, bool show = false);
		static double RecourseCost(Prob * prob, std::vector<Node*>& path);
		static int GetDriverCount(Prob * prob, std::vector<Node*>& path);
		static int GetDriverCount(Prob* prob);
};

#endif