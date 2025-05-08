
#ifndef EXACT_BRP_H
#define EXACT_BRP_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <ilcplex/ilocplex.h>
#include <algorithm>
#include <vector>
#include "NodeBRP.h"
#include "DriverBRP.h"
#include "ProblemDefinition.h"
#include "ExactBrpGraph.h"
#include "ExactBrpCallBacks.h"
#include "Solution.h"
#include "ExactBrpSepBase.h"

#pragma once
#ifdef USE_CVRPSEP
#include "ExactBrpSepCvrpsep.h"
using ExactBrpSep = ExactBrpSepCvrpsep;
#else
#include "ExactBrpSepUser.h"
using ExactBrpSep = ExactBrpSepUser;
#endif




class ExactBrpO
{
	public:
		ExactBrpO()
		{
			max_time = 300;
			lazy_call = NULL;
		}
		
		void Solve(Prob * prob);
		void Init(IloEnv env);
		void SolveProblem(IloEnv env);
		void AddInfSetInq(IloEnv env);
		void Clear();
		void SetSolution(Sol & _sol){ s = _sol; }
		void SetMipStart();

		int max_time;
		double time_taken;
		double start_time;
		int status;
		double lb;
		double ub;
		double ub_recourse;
		double ub_distance;
		int nb_inf_sets;
		int nb_inf_paths;
		int nb_sub_tours;
		int nb_sub_tour_frac;
		int nb_l_cuts;
		int nb_p_cuts;
		int nb_frac_l_cuts;
		int nb_sorted_l_cuts;
		int nb_benders_cuts;
		int drvs;

		double cplex_distances;
		double cplex_recourse;
		double cplex_relative_gap;
		bool solvedAtRoot;

	private:

		Prob * prob;
		ExactBrpGraphO * graph;
		Sol s;
		
		IloModel model;
		IloObjective obj_func;
		IloCplex cplex;
		IloNumVarArray x;

		IloNumVar z; //Nb of vehicles
		IloNumVar theta; //Recourse
		
	
		ExactBrpSep * _sep;
		ExactBrpLazyCallBackO * lazy_call;
		ExactBrpUserCutCallBackO * user_call;
		
};

#endif
