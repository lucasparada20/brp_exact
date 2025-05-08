
#ifndef EXACT_BRP_SEP_CVRPSEP_H
#define EXACT_BRP_SEP_CVRPSEP_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <ilcplex/ilocplex.h>
#include <algorithm>
#include <vector>
#include <map>

#include "NodeBRP.h"
#include "DriverBRP.h"
#include "ProblemDefinition.h"
#include "ExactBrpGraph.h"
#include "RouteFeasibility.h"
#include "Parameters.h"
#include "ExactBrpSepBase.h"

// From CVRPSEP
#include "basegrph.h"
#include "cnstrmgr.h"
#include "capsep.h"


class ExactBrpSepCvrpsep : public ExactBrpSepBase 
{
	public:
		
		ExactBrpSepCvrpsep(IloEnv env, ExactBrpGraphO * graph, IloNumVarArray x, IloNumVar theta);
		~ExactBrpSepCvrpsep();

		void SeparateInt(IloRangeArray array) override;
		void SeparateFrac(IloRangeArray array) override;
		const char* VersionTag() const override { return "Separator: CVRPSEP"; }
		
		int  SeparateMaxRouteDistCut(IloRangeArray array);
		void SeparateOptCut(IloRangeArray array);
		
		// Calls CVRPSEP
		void SeparateSubTourInequalities(IloRangeArray constraints);

		//These return the number of added constraints
		int TestAndAddInfeasiblePath(std::vector<Node*> & path, IloRangeArray array);

		double best_sol;
		double best_sol_recourse;
		double best_sol_distance;
		std::vector< std::vector<Node*> > best_solution;
		
		//Counters for cuts
		int nb_opt_cuts;
		int nb_inf_sets;
		int nb_inf_paths;
		int nb_sub_tours;
		int nb_sub_tour_frac;
		int nb_max_route_cuts;
		int nb_dk_cuts;
    
		std::vector<int> _component;

	private:		
		
		IloEnv _env;
		ExactBrpGraphO * _graph;
		IloNumVarArray _x;
		IloNumVar _theta;

		Prob * _prob;
		
		//CVRPSEP objects
		char IntAndFea;
		int NoOfCustomers,CAP,NoOfEdges,MaxNoOfCuts;
		double EpsForIntegrality;
		double MaxViolationRCI;
		double MaxViolationMS;
		double MaxViolationHypotour;
		double * Demands;
		int *EdgeTail, *EdgeHead;
		double *EdgeX;
		CnstrMgrPointer MyCutsCMP,MyOldCutsCMP;		

};

#endif