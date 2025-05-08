#ifndef SEQUENTIAL_INSERTION
#define SEQUENTIAL_INSERTION

#include "Solution.h"
#include "Move.h"
#include "InsRmvMethodBRP.h"
#include "OperatorBase.h"

class SequentialInsertionBRP : public InsertOperator
{
	public:
		SequentialInsertionBRP(InsRmvMethodBRP & insrmv): _insrmv(insrmv){}

	void Insert(Sol & s) override;

	private:
		InsRmvMethodBRP & _insrmv;

};


#endif