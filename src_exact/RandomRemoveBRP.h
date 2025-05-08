#ifndef REMOVE_RANDOM
#define REMOVE_RANDOM

#include "Solution.h"
#include <vector>
#include <algorithm>
#include "OperatorBase.h"


class RemoveRandomBRP : public RemoveOperator
{
	public:
	
		void Remove(Sol & s, int count) override;
		
	private:
	
		std::vector<Node*> vect;
		
};


#endif