#ifndef COST_FUNCTION_BRP
#define COST_FUNCTION_BRP

class Sol;  // Forward declaration of Sol

#include "NodeBRP.h"
#include "DriverBRP.h"

class CostFunctionBRP
{
	public:
		CostFunctionBRP(){}
		~CostFunctionBRP(){}

		double GetCost(Sol & s);
		double GetCost(Sol & s, Driver * d);
		void Update(Sol & s);
		void Update(Sol & s, Driver * d);
		void Show(Sol * s, Driver * d);
	private:
		std::vector<Node*> path;
};


#endif
