#include "ExactBrp.h"
#include "Constants.h"
#include "Parameters.h"

void ExactBrpO::Solve(Prob* pprob)
{
	start_time = clock();
	prob = pprob;
	graph = new ExactBrpGraphO(prob);

	IloEnv env;
	Init(env);
	SolveProblem(env);
	
	Clear(); //Releases resource from cplex callback objects and user defined objects graph and sep
	
	env.end();
}

void ExactBrpO::SolveProblem(IloEnv env)
{
	try
	{
		bool re = cplex.solve();
		clock_t end_time = clock();
		time_taken = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		
		int cplex_status = (int)cplex.getCplexStatus();
		double sol = re?cplex.getObjValue():9999999.0;
		
		printf("re:%d sol:%.3lf status:%d nbnodes:%d time:%.3lf status:%d\n", (int)re, sol, cplex_status, (int)cplex.getNnodes(), time_taken,cplex_status);

		if( re && cplex_status == 11 )
		{	
			status = 0;
			ub = sol;
			ub_distance = _sep->best_sol_distance;
			ub_recourse = ub > 9999 ? 999999.0 : cplex.getValue(theta);
			lb = cplex.getBestObjValue();
			drvs = ub < 9999999.0 ? (int)cplex.getValue(z) : 9999999.0;
			//cannot call cplex.GetValue() if no solution was found ...
			printf("Timed out ObjLb:%.1lf Ub:%.1lf DistUbFromSep:%.1lf Rec:%.1lf Drvs:%d\n"
					,lb,ub,ub_distance,ub_recourse,drvs);
		}
		else if( re && cplex_status == 1)
		{
			//build the solution
			for (int i = 0; i < graph->GetArcCount(); i++)
			{
				graph->GetArc(i)->value = cplex.getValue(x[i]);
				//printf("Arc:%d value:%.2lf\n",i,cplex.getValue(x[i]));
			}
				
			graph->AssignPositiveValues();
			graph->MakePaths();
			//graph->ShowPosValueArcs();
			printf("Solution IsInteger:%d Paths:%d\n",graph->IsInteger(),graph->GetPathCount()); graph->ShowPaths();

			double sum_d = 0; double sum_rec = 0;
			for (int i = 0; i < graph->GetPathCount(); i++)
			{
				std::vector<Node*>& path = graph->GetPath(i);
				
				double rec = RouteFeasibility::RecourseCost(prob,path);
				bool is_feas = RouteFeasibility::IsFeasible(prob,path);
				printf("Path:%d IsFeas:%d Rec:%.1lf\n",i,is_feas,rec);
				
				sum_rec += RouteFeasibility::RecourseCost(prob,path);
				for (int j = 1; j < path.size(); j++)
				{
					ExBrpArcO* arc = graph->GetArc(path[j - 1]->no, path[j]->no);
					sum_d += arc->cost;
				}
			}
			std::cout << "Distances: " << sum_d << std::endl;
			std::cout << "Theta: " << cplex.getValue(theta) << " sumRec: " << sum_rec << std::endl;
			
			status = 1;
			ub = sum_d + cplex.getValue(theta);
			ub_distance = sum_d;
			ub_recourse = cplex.getValue(theta);
			lb = ub;
			drvs = (int)cplex.getValue(z);
			
			//graph->PrintGraph((char*)"sol.dot");
		}

		nb_inf_sets = _sep->nb_inf_sets;
		nb_inf_paths = _sep->nb_inf_paths;
		nb_sub_tours = _sep->nb_sub_tours;
		nb_sub_tour_frac = _sep->nb_sub_tour_frac;
		
		printf("nb_sub_tours:%d\n", _sep->nb_sub_tours);
		printf("nb_sub_tours_frac:%d\n", _sep->nb_sub_tour_frac);
		printf("nb_inf_paths:%d\n",_sep->nb_inf_paths);
		printf("nb_inf_sets:%d\n",_sep->nb_inf_sets);
		printf("nb_max_route_cuts:%d\n",_sep->nb_max_route_cuts);
		printf("nb_opt_cuts:%d\n",_sep->nb_opt_cuts);

	} catch (IloException &ex) {
	   std::cerr << ex << std::endl;
	}

}

void ExactBrpO::Init(IloEnv env)
{
	int Q = prob->GetDriver(0)->capacity;
	model = IloModel(env);

	std::vector<Node*> stations;
	stations.resize(graph->GetNodeCount(), NULL);
	for (int i = 0; i < graph->GetNodeCount(); i++)
		stations[i] = graph->GetNode(i);

	x = IloNumVarArray(env, graph->GetArcCount(), 0, 1, ILOINT);
	z = IloNumVar(env, prob->GetDriverCountLB(), prob->GetDriverCount(), ILOINT);
	theta = IloNumVar(env,0,IloInfinity,ILOFLOAT);
	
	theta.setName("t");
	z.setName("z");

	for (int i = 0; i < graph->GetArcCount(); i++)
	{
		ExBrpArcO* ar = graph->GetArc(i);
		//printf("arc:%d index:%d cost:%lf from:%d to:%d\n",i,ar->index,ar->cost,ar->from->no,ar->to->no);
		char name[40];
		sprintf(name, "x%d_%d", ar->from->no, ar->to->no);
		x[i].setName(name);
		if (ar->from->no != 0)
			x[i].setBounds(0, 1);
	}

	IloExpr obj1(env);
	for (int i = 0; i < graph->GetArcCount(); i++)
		obj1 += graph->GetArc(i)->cost * x[i];
	obj1 += theta;
	obj_func = IloMinimize(env, obj1);
	model.add(obj_func);
	obj1.end();


	//\sum_{ i \in C} x_{0i} = 2K
	{
		IloExpr expr(env);
		for (int i = 0; i < graph->GetArcsInOfCount(0); i++)
			expr += x[graph->GetArcInOf(0, i)->index];
		model.add(expr == z);
		expr.end();
	}

	{
		IloExpr expr(env);
		for (int i = 0; i < graph->GetArcsOutOfCount(0); i++)
			expr += x[graph->GetArcOutOf(0, i)->index];
		model.add(expr == z);
		expr.end();
	}

	//station's in and out degree constrains
	{
		for (int i = 1; i < graph->GetNodeCount(); i++)
		{
			IloExpr expr(env);
			for (int j = 0; j < graph->GetArcsInOfCount(i); j++)
				expr += x[graph->GetArcInOf(i, j)->index];
			model.add(expr == 1);
			expr.end();
		}

		for (int i = 1; i < graph->GetNodeCount(); i++)
		{
			IloExpr expr(env);
			for (int j = 0; j < graph->GetArcsOutOfCount(i); j++)
				expr += x[graph->GetArcOutOf(i, j)->index];
			model.add(expr == 1);
			expr.end();
		}
	}

	//x_{ij} + x_{ji} \leq 1
	//Necessary for preventing 2-cycles
	{
		for (int i = 1; i < graph->GetNodeCount(); i++)
			for (int j = i + 1; j < graph->GetNodeCount(); j++)
				if (i != j)
				{
					ExBrpArcO* ar = graph->GetArc(i, j);
					ExBrpArcO* arr = graph->GetArc(j, i);
					if(ar == NULL || arr == NULL) continue;

					IloExpr expr(env);
					expr += x[ar->index];
					expr += x[arr->index];
					model.add(expr <= 1);
					expr.end();
				}
	}

	//Infeasible Arc Inequalities (to outgoing station 'h')
	std::vector<Node*> path(3, NULL);
	for (int i = 0; i < graph->GetArcCount(); i++)
	{
		ExBrpArcO* ar = graph->GetArc(i);
		path[0] = ar->from;
		path[1] = ar->to;
		if(ar->to->no == 0) continue;

		IloExpr expr(env);
		expr += x[ar->index];

		int nb=0;
		for (int j = 0; j < graph->GetArcsOutOfCount(ar->to->no); j++)
		{
			ExBrpArcO* arr = graph->GetArcOutOf(ar->to->no, j);
			path[2] = arr->to;

			if(!RouteFeasibility::IsFeasible(prob, path))
			{
				expr += x[arr->index];
				nb++;
			}
		}

		if(nb >= 1)
		{
			//std::cout << expr << "<= 1" << std::endl;
			model.add(expr <= 1);
		}
		expr.end();
	}

	//Infeasible Arc Inequalities (from incoming station 'h')
	for (int i = 0; i < graph->GetArcCount(); i++)
	{
		ExBrpArcO* ar = graph->GetArc(i);
		path[1] = ar->from;
		path[2] = ar->to;
		if(ar->from->no == 0) continue;

		IloExpr expr(env);
		expr += x[ar->index];

		int nb=0;
		for (int j = 0; j < graph->GetArcsInOfCount(ar->from->no); j++)
		{
			ExBrpArcO* arr = graph->GetArcInOf(ar->from->no, j);
			path[0] = arr->from;

			if(!RouteFeasibility::IsFeasible(prob, path))
			{
				expr += x[arr->index];
				nb++;
			}
		}

		if(nb >= 1)
		{
			//std::cout << expr << "<= 1" << std::endl;
			model.add(expr <= 1);
		}
		expr.end();
	}


	if(Parameters::GetInstanceType() == 2) // slr
	{
		//theta >= sum over all arcs (i,j) of sigma_ij x_ij
		//where sigma_ij is the recourse cost of traversing arc (i,j)
		{
			IloExpr expr(env);
			expr += theta;
			int nb1=0;
			for (int i = 0; i < graph->GetArcCount(); i++)
			{
				ExBrpArcO* ar = graph->GetArc(i);
				if(ar->from->no == 0 || ar->to->no == 0) continue;

				double cost = 0;
				int nb = 0;
				if(std::abs(ar->from->demand + ar->to->demand) > Q)
				{
					cost += std::abs(ar->from->demand + ar->to->demand) - Q;
					nb++;
				}
				if(nb >= 1)
				{
					expr -= cost * x[ar->index];
					nb1++;
				}
			}

			//std::cout << expr << " >= 0" << std::endl;
			if(nb1 >= 1)
				model.add(expr >= 0);
			expr.end();
		}
		//Bounding theta >= sum_{i \in arcs} L * x_i 
		{
			IloExpr expr(env);
			expr += theta;
		
			double l = RouteFeasibility::Calculate(prob);

			for (int i = 0; i < graph->GetArcCount(); i++)
			{
				ExBrpArcO* ar = graph->GetArc(i);
				if(ar->from->no == 0 || ar->to->no == 0) continue;
				expr -= l * x[ar->index];
			}
			model.add(expr >= (l* (3 - graph->GetNodeCount())) );
			//std::cout << expr << " >= " << (l* (3 - graph->GetNodeCount())) << std::endl;
			expr.end();
		}

		//L_1 (\underscore(K)_1) inequality
		{
			IloExpr expr(env);
				
			expr += theta;
			
			double lb_recourse = RouteFeasibility::CalculateWithMinDriverCount(prob);

			double rhs = lb_recourse + Q * prob->GetDriverCountLB();
			expr += z * Q;

			//std::cout << expr << " >= " << rhs << std::endl;
			model.add(expr >= rhs);
			expr.end();
		}		
	}	
	
	
	AddInfSetInq(env); //Separete inf sets of size 3

	// To separate the cuts 
	_sep = new ExactBrpSep(env, graph, x, theta);
	printf("%s\n",_sep->VersionTag());
	
	lazy_call = new (env) ExactBrpLazyCallBackO(env, graph, x, theta, _sep);
	user_call = new (env) ExactBrpUserCutCallBackO(env, graph, x, theta, _sep);
	
	_sep->best_sol = 9999999999; _sep->best_sol_distance = 9999999999;	

	cplex = IloCplex(model);
	if(Parameters::GetInstanceType() == 1) //dhin
		SetMipStart();
	//cplex.exportModel("brp_first_stage.lp");
	cplex.setParam(IloCplex::Param::TimeLimit,max_time);
	cplex.setParam(IloCplex::Param::Threads, 1);
	cplex.setWarning(env.getNullStream());
	cplex.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, prob->GetUpperBound() + EPSILON + 0.01);
	cplex.use(lazy_call);
	cplex.use(user_call);
	
	lazy_call->add_constraints = user_call->add_constraints = true;
	cplex.setParam(IloCplex::Param::MIP::Limits::Nodes,0);
	cplex.solve();
		
	double obj = cplex.getBestObjValue();
	printf("Best Obj:%.3lf LazyInqs:%d UserInqs:%d\n"
			, obj, (int)lazy_call->added_constraints.getSize(), (int)user_call->added_constraints.getSize() );
	model.add(lazy_call->added_constraints);
	model.add(user_call->added_constraints);
	
	
	lazy_call->add_constraints = user_call->add_constraints = false;
	cplex.setParam(IloCplex::Param::MIP::Limits::Nodes,9999999999);
	cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0);
	
}

void ExactBrpO::AddInfSetInq(IloEnv env)
{
	std::vector<Node*> path(3, NULL);
	for(int i=1;i<graph->GetNodeCount();i++)
		for(int j=i+1;j<graph->GetNodeCount();j++)
			for(int k=j+1;k<graph->GetNodeCount();k++)
			{
				path[0]=graph->GetNode(i);
				path[1]=graph->GetNode(j);
				path[2]=graph->GetNode(k);
				
				int nb_veh = RouteFeasibility::GetDriverCount(prob,path);
				if(nb_veh==1) 
				{
					continue;
				}
				
				int cntr=0;
				IloExpr expr(env);
					for(size_t u = 0; u < path.size(); u++)
						for(size_t v = 0; v < path.size(); v++)
							if (u != v)
							{
								ExBrpArcO* arc = graph->GetArc(path[u]->no, path[v]->no);
								if (arc == NULL) continue;
								expr += x[arc->index];
								cntr++;
							}
				if(cntr > 3)
				{
					model.add(expr <= 3 - nb_veh);
					//std::cout << "root inf set:" << expr << " <= " << 3-nb_veh << std::endl;				
				}
				expr.end();						
			}
			
}

void ExactBrpO::SetMipStart()
{
	std::vector<double> start_arcs(graph->GetArcCount(),0.0);
	for(int i=0;i<prob->GetDriverCount();i++)
	{
		Driver * d = s.GetDriver(i);
		Node * cur = s.GetNode( d->StartNodeID );
		if(s.RoutesLength[i] == 0) continue; //Unassigned customers
		while(cur != NULL)
		{
			int start_no = cur->no;
			cur = s.Next[ cur->id ];
			int next_no = cur->no;
			ExBrpArcO* arc = graph->GetArc(start_no,next_no);

			start_arcs[arc->index] = 1.0;
			//printf("start arc---> index:%d from:%d to:%d value:%.1lf\n",arc->index,arc->from->no,arc->to->no,start_arcs[arc->index]);

			if(cur->type == NODE_TYPE_END_DEPOT) break;
		}
	}

	//x_ij
	IloNumVarArray startVar(cplex.getEnv());
	IloNumArray startVal(cplex.getEnv());
	for(int i = 0; i < graph->GetArcCount(); i++)
	{
		ExBrpArcO* arc = graph->GetArc(i);
		startVar.add(x[arc->index]);
		startVal.add(start_arcs[i]);
		//startVal.add(start_arcs[i]+0.01);
	}
	cplex.addMIPStart(startVar, startVal, IloCplex::MIPStartRepair);
	//cplex.setParam(IloCplex::Param::MIP::Limits::RepairTries, 10);
	startVal.end();
	startVar.end();

}

void ExactBrpO::Clear()
{
	delete graph;
	delete _sep;
	if (lazy_call != NULL)
	{
		cplex.remove(lazy_call);
		delete lazy_call;

	}
	if (user_call != NULL)
	{
		cplex.remove(user_call);
		delete user_call;
	}
}

