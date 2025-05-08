#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <csignal>
#include <string.h> //strcm

#include "NodeBRP.h"
#include "DriverBRP.h"
#include "LoadBRP.h"
#include "ProblemDefinition.h"
#include "ExactBrp.h"
#include "DriverCountBppLb.h"
#include "../src_heur/InsRmvMethodBRP.h"
#include "../src_heur/CostFunctionBRP.h"
#include "../src_heur/SequentialInsertionBRP.h"
#include "../src_heur/RelatednessRemoveBRP.h"
#include "../src_heur/AlnsBRP.h"

#include <ctime>
#include <fstream>
#include <iostream>

int main(int arg, char ** argv)
{
	Parameters param;
	param.Read(arg,argv);
	
	Prob pr;
	LoadBRP Load;
	
	Parameters::SetDelta(1);

	if(Parameters::GetInstanceType() == 1 ) // dhin
		Load.Load_brp_instance(pr,Parameters::GetInstanceFileName());
	if(Parameters::GetInstanceType() == 2 ) // slr
		Load.Load_slr_instance(pr,Parameters::GetInstanceFileName());

	CostFunctionSBRP cost_func;
	
	Sol sol(&pr,&cost_func);
	sol.PutAllNodesToUnassigned();
	
	InsRmvMethodBRP method(pr);
	SequentialInsertionBRP seq(method);
	
	seq.Insert(sol); //Sequential insertion of nodes to build an initial solution and store in sol
		
	int min_drv_cnt1 = RouteFeasibility::GetDriverCount(&pr);
	DriverCountBppLb driver_lb;
	int min_drv_cnt2 = driver_lb.GetLB(&sol); //The set covering model uses the initial solution
	pr.SetDriverCountLB( std::max( min_drv_cnt1, min_drv_cnt2) );
	
	double lb_recourse = RouteFeasibility::CalculateWithMinDriverCount(&pr);
	printf("Recourse LB with min drivers:%.2lf MinDriverCount1:%d MinDriverCount2:%d\n", lb_recourse, min_drv_cnt1,min_drv_cnt2);		

	//------- ALNS ----------//
	if(Parameters::GetInstanceType() == 1) // no recourse in dhin instances ...
	{
		RemoveRandomBRP random_remove;
		RelatednessRemoveBRP related_remove(pr.GetDistances());
		
		ALNS alns;
		alns.AddInsertOperator(&seq);
		//---------------------------------
		alns.AddRemoveOperator(&random_remove);
		alns.AddRemoveOperator(&related_remove);
		
		//Optimize
		alns.SetTemperatureIterInit(0);
		alns.SetTemperature(0.99);
		alns.SetIterationCount(750000);//Remember to set a lot of iterations
		
		clock_t start_time = clock();
		alns.Optimize(sol);
		clock_t end_time = clock();
		double elapsed_seconds = (double)( end_time - start_time ) / CLOCKS_PER_SEC;
		
		
		sol.Update();
		sol.Show();
		
		double heur_ub = sol.GetTotalDistances();
		double heur_nb_drivers = sol.GetUsedDriverCount();
		pr.SetUpperBound( sol.GetTotalDistances() );
		
		std::string re_file_name_heur_str = std::string("results/re_dhin_heur_") + Parameters::GetCityName() + "_" + std::to_string(pr.GetDriver(0)->capacity) + ".txt";
		
		std::ofstream re_file_name_heur(re_file_name_heur_str);
		
		if(!re_file_name_heur.is_open())
		{
			printf("Could not open re file: %s. Phil Collins (1989)\n",re_file_name_heur_str.c_str());
			exit(1);
		}
		//re file
		re_file_name_heur << std::string(Parameters::GetCityName()) << "," << pr.GetCustomerCount() << "," << pr.GetDriver(0)->capacity << "," << heur_ub << "," << heur_nb_drivers << "," << std::fixed << std::setprecision(2) << elapsed_seconds << "\n";
		printf("Re file written to:%s\n",re_file_name_heur_str.c_str());
		re_file_name_heur.close();		
	}
	
	ExactBrpO ex;
	if(Parameters::GetInstanceType() == 1)
		ex.SetSolution(sol);
	ex.max_time = 3600;
	ex.Solve(&pr);

	std::string re_file_name_exact_str = std::string("results/re_exact_") + Parameters::GetCityName() + "_" + std::to_string(pr.GetDriver(0)->capacity) + "_" + std::to_string(pr.GetCustomerCount()) + ".txt";
	
	std::ofstream re_file_name_exact(re_file_name_exact_str);
	if(!re_file_name_exact.is_open())
	{
		printf("Could not open re file: %s. Phil Collins (1989)\n",re_file_name_exact_str.c_str());
		exit(1);
	}
	//re file
	re_file_name_exact 
		<< std::string(Parameters::GetCityName()) << "," 
		<< pr.GetCustomerCount() << "," 
		<< pr.GetDriver(0)->capacity << "," 
		<<  ex.status << "," 
		<< ex.ub << ","
		<< ex.lb << "," 		
		<< ex.ub_distance << "," 
		<< ex.ub_recourse << "," 
		<< ex.drvs << "," 
		<< ex.nb_inf_sets << "," 
		<< ex.nb_inf_paths << "," 
		<< ex.nb_sub_tours << "," 
		<< ex.nb_sub_tour_frac << "," 
		<< std::fixed << std::setprecision(2) << ex.time_taken << "\n";
		
	printf("Re file written to:%s\n",re_file_name_exact_str.c_str());
	re_file_name_exact.close();	

	return 0;
}


