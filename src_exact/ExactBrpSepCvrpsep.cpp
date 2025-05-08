#include "ExactBrpSepCvrpsep.h"

ExactBrpSepCvrpsep::ExactBrpSepCvrpsep(IloEnv env, ExactBrpGraphO* graph, IloNumVarArray x, IloNumVar theta) : _env(env), _graph(graph), _x(x), _theta(theta), _prob(graph->GetProblem())
{
	nb_inf_sets = 0;
	nb_inf_paths = 0;
	nb_sub_tours = 0;
	nb_sub_tour_frac = 0;
	nb_opt_cuts = 0;
	nb_max_route_cuts = 0;
	nb_dk_cuts = 0;
	best_sol = best_sol_recourse = best_sol_distance = 9999999999;

	// Lysgaard objects
	CMGR_CreateCMgr(&MyCutsCMP,_graph->GetNodeCount());
	CMGR_CreateCMgr(&MyOldCutsCMP,_graph->GetNodeCount());
	Demands = (double*)calloc(_graph->GetNodeCount()+2,sizeof(double));
	EdgeTail = (int*)calloc(_graph->GetArcCount()+1, sizeof(int));
	EdgeHead = (int*)calloc(_graph->GetArcCount()+1, sizeof(int));
	EdgeX = (double*)calloc(_graph->GetArcCount()+1, sizeof(double));
	EpsForIntegrality = 0.0001;
	MaxViolationRCI = 0.2;
	MaxViolationMS = 0.1;
	MaxViolationHypotour = 0.1;
	MaxNoOfCuts = _graph->GetNodeCount();
	NoOfCustomers = _graph->GetNodeCount();

	for(int i = 0 ; i < _graph->GetNodeCount();i++)
 		Demands[i] = _graph->GetNode(i)->demand;
}

ExactBrpSepCvrpsep::~ExactBrpSepCvrpsep()
{
	free(Demands);free(EdgeTail);free(EdgeHead);free(EdgeX);
	CMGR_FreeMemCMgr(&MyCutsCMP);
	CMGR_FreeMemCMgr(&MyOldCutsCMP);
}

void ExactBrpSepCvrpsep::SeparateInt(IloRangeArray array)
{
	SeparateSubTourInequalities(array);
	nb_sub_tours += array.getSize();
	if(array.getSize() == 0)
	{
		_graph->MakePaths();
		for(int i=0;i<_graph->GetPathCount();i++)
		{
			std::vector<Node*> & path = _graph->GetPath(i);
			TestAndAddInfeasiblePath(path, array);
		}
	}
	
	if(Parameters::GetInstanceType() == 1 && array.getSize() == 0) //dhin
	{
		double sol_distance = _graph->GetCost();
		if(sol_distance < best_sol)
		{
			best_sol_distance = sol_distance;
			best_sol_recourse = 0;
			best_sol = sol_distance;
			printf("New solution cost:%.1lf dist:%.1lf rec:%.1lf paths:%d\n", best_sol, best_sol_distance, best_sol_recourse,_graph->GetPathCount());
			best_solution.clear(); best_solution.reserve( _graph->GetPathCount() );
			for(int i = 0; i < _graph->GetPathCount(); i++)
				best_solution.push_back( _graph->GetPath(i) );
		}	
	}
}

void ExactBrpSepCvrpsep::SeparateFrac(IloRangeArray array)
{
    SeparateSubTourInequalities(array);
	nb_sub_tour_frac += array.getSize();
    //if (array.getSize() > 0)
    //    return;
	
	//if not subtours, find paths and check infeasible sets, paths ...
    const int node_count = _graph->GetNodeCount();
    std::vector<bool> walk_on(node_count, false);
    std::vector<Node*> path;
    path.reserve(node_count + 2); // +2 for depot at start and end

    for (int i = 1; i < node_count; ++i)
    {
        if (walk_on[i]) continue;
		double sumv=0.0;
        path.clear();
        path.push_back(_graph->GetNode(0)); // depot at start

        ExBrpArcO* ar = nullptr;
        const int out_count = _graph->GetArcsOutPosCount(i);

        // Find first strong arc
        for (int j = 0; j < out_count; ++j) {
            ExBrpArcO* ar2 = _graph->GetArcsOutPos(i, j);
            if (ar2->value >= 0.9) 
			{
                sumv += ar2->value;
				ar = ar2;
                break;
            }
        }

        if (!ar || ar->to->no == 0) continue;

        walk_on[ar->from->no] = true;
        path.push_back(ar->from);

        Node* current = ar->to;
        bool is_sub_tour = false;

        while (true)
        {
            walk_on[current->no] = true;
            path.push_back(current);

            ar = nullptr;
            const int out_current = _graph->GetArcsOutPosCount(current->no);
            for (int j = 0; j < out_current; ++j) {
                ExBrpArcO* ar2 = _graph->GetArcsOutPos(current->no, j);
                if (ar2->value >= 0.9) 
				{
					sumv += ar2->value;
                    ar = ar2;
                    break;
                }
            }

            if (!ar) break;
            current = ar->to;

            if (current->no == i) { is_sub_tour = true; break; }
            if (walk_on[current->no] || current->no == 0) break;
        }

        path.push_back(_graph->GetNode(0)); // depot at end

        if (!is_sub_tour)
		{
			if(path.size()<=5) return; // All 2,3-size inf sets are added at the root as inf arcs.
			//Additionally, all single routes are feasible.
			//So an inf set needs to be at least: 0-i-j-k-l-0, size>=6

			int nb_veh = RouteFeasibility::GetDriverCount(_prob, path);
			if( sumv > path.size() - 2 - nb_veh + 0.2 )
				TestAndAddInfeasiblePath(path, array);
		}
            
    }
}

void ExactBrpSepCvrpsep::SeparateSubTourInequalities(IloRangeArray constraints)
{
	std::vector<Node*> nodes;
	NoOfEdges = 0;
	for(int i = 0; i < _graph->GetPosArcCount(); i++)
	{
		ExBrpArcO* arc = _graph->GetPosArc(i);
		ExBrpArcO* arc2 = _graph->GetArcsPos(arc->to->no, arc->from->no);
		double sumv = arc->value;
		if (arc2 != NULL) sumv += arc2->value; //Do to 2-cycles the other one is zero ... but cvrpsep needs unoriented graphs
		int f0 = arc->from->no != 0 ? arc->from->no : NoOfCustomers;
		int t0 = arc->to->no != 0 ? arc->to->no : NoOfCustomers;
		int from = std::min(f0,t0);
		int to = std::max(f0,t0);

		if (arc2 != NULL && arc->from->no > arc->to->no) continue;

		NoOfEdges++;
		EdgeTail[NoOfEdges] = from != 0 ? from : NoOfCustomers;
		EdgeHead[NoOfEdges] = to != 0 ? to : NoOfCustomers;
		EdgeX[NoOfEdges] = sumv;
		//printf("i:%d from:%d to:%d v:%.2lf\n",NoOfEdges,EdgeTail[NoOfEdges],EdgeHead[NoOfEdges],sumv);
	}
	
	int cap = _prob->GetDriver(0)->capacity;
	double mv;
	CAPSEP_SeparateCapCuts(	NoOfCustomers-1,Demands,cap,NoOfEdges, EdgeTail,EdgeHead,EdgeX,
					MyOldCutsCMP, MaxNoOfCuts, EpsForIntegrality, 0.2, &IntAndFea, &mv, MyCutsCMP);

	for(int i = 0; i < MyCutsCMP->Size; i++)
	{	
		nodes.clear(); nodes.reserve( MyCutsCMP->CPL[i]->IntListSize );
		for (int j=1;j<=MyCutsCMP->CPL[i]->IntListSize; j++)
			if(MyCutsCMP->CPL[i]->IntList[j] != NoOfCustomers)
				nodes.push_back( _prob->GetCustomer( MyCutsCMP->CPL[i]->IntList[j] - 1) );
		
		int nb = nodes.size();
		
		// All 2,3-size inf sets are added at the root as inf arcs.
		//Additionally, all single routes are feasible.
		//So an inf set needs to be at least: 0-i-j-k-l-0, size>=6
		int nbv = nb > 3 ? RouteFeasibility::GetDriverCount(_prob, nodes) : 1;		
		double sumt = 0, sumv = 0;

		{//subtours
			IloExpr expr(constraints.getEnv());
			for(size_t j=0;j<nodes.size();j++)
				for(size_t k=0;k<nodes.size();k++)
					if(j != k)
					{
						ExBrpArcO * arc = _graph->GetArc(nodes[j]->no,nodes[k]->no);
						if(arc != NULL)
						{
							expr += _x[arc->index];
							sumv += arc->value;
						}
					}
			if(sumv > nb - nbv + 0.2)
				constraints.add(expr <= nb - nbv);
			//if(nb == 2 || nb == 3) 
			//	printf("nb:%d using_nbv:%d real_nbv\n",nb,nbv,RouteFeasibility::GetDriverCount(_prob, nodes));			
				
			//std::cout << "Subtours: " << expr << " <= " << MyCutsCMP->CPL[i]->RHS<< std::endl;
			expr.end();
		}
	}

	/* Move the new cuts to the list of old cuts: */
	for(int i = 0; i < MyCutsCMP->Size; i++)
		CMGR_MoveCnstr(MyCutsCMP, MyOldCutsCMP, i, 0);
	MyCutsCMP->Size = 0;
}


// Test if a the path is infeasible
// In such case, it searches for the smallest infeasible set
// The smallest infeasible path
// Note that the first and last nodes are the depot
int ExactBrpSepCvrpsep::TestAndAddInfeasiblePath(std::vector<Node*> & path, IloRangeArray array)
{
	int nb_inq = 0;
	if(RouteFeasibility::IsFeasible(_prob, path)) return 0;
	if(path.size() <= 3) return 0;

	std::vector<Node*> subpath;

	//First, check for infeasible sets
	bool found_inf = false;
	for(int k=1;k+2<path.size();k++)
	{
		for(int j=1;j+2<path.size() && j+k+1<path.size();j++)
		{
			subpath.clear(); subpath.reserve( (int)path.size() );
			subpath.push_back(path[0]);

			for(int l=j;l<=j+k && l+1 < path.size();l++)
				subpath.push_back(path[l]);
			subpath.push_back(path.back());

			int nb_drivers = RouteFeasibility::GetDriverCount(_prob, subpath);
			if(nb_drivers <= 1) continue;

			IloExpr expr(array.getEnv());
			for(int l=1;l+1<subpath.size();l++)
				for(int k=1;k+1<subpath.size();k++)
					if(l != k)
					{
						ExBrpArcO* arc = _graph->GetArc( subpath[l]->no, subpath[k]->no);
						if(arc != NULL)
							expr += _x[arc->index];
					}
			array.add(expr <= ((int)subpath.size()) - 2 - nb_drivers);
			expr.end();
			nb_inf_sets++;
			nb_inq++;
			found_inf = true;
			break;
		}//end for j
		if(found_inf) break;

	}//end for k
	if(found_inf) return nb_inq;

	//Second, check for infeasible path (since a set bounds more solutions, from here onwards the code is not executed)
	{
		IloExpr expr(array.getEnv());
		int nb = 0;
		for(int j = 1; j < path.size(); j++)
		{
			if (path[j - 1]->type != NODE_TYPE_CUSTOMER || path[j]->type != NODE_TYPE_CUSTOMER) continue;

			ExBrpArcO* arc = _graph->GetArc(path[j - 1]->no, path[j]->no);
			expr += _x[arc->index];
			nb++;
		}

		printf("infeasible path\n");
		std::cout << expr << " <= " << nb-1 << std::endl;
		array.add(expr <= nb - 1);
		nb_inf_paths++;
		nb_inq++;
		expr.end();
	}

	if(path.size() <= 4) return nb_inq;


	for(int k=1;k+2<path.size();k++)
	{
		bool found_inf = false;
		for(int j=1;j+2<path.size() && j+k+1<path.size();j++)
		{
			subpath.clear(); subpath.reserve( (int)path.size() );
			subpath.push_back(path[0]);

			for(int l=j;l<=j+k && l+1 < path.size();l++)
				subpath.push_back(path[l]);
			subpath.push_back(path.back());

			if(RouteFeasibility::IsFeasible(_prob, subpath)) continue;

			IloExpr expr(array.getEnv());
			int nb = 0; double sum_v=0.0;
			for(int l = 1; l < subpath.size(); l++)
			{
				if(subpath[l - 1]->type != NODE_TYPE_CUSTOMER || subpath[l]->type != NODE_TYPE_CUSTOMER) continue;

				ExBrpArcO* arc = _graph->GetArc(subpath[l - 1]->no, subpath[l]->no);
				if(arc != NULL)
				{
					expr += _x[arc->index];
					sum_v += arc->value;
					nb++;
				}
			}
			if(sum_v > nb - 1 + 0.2 )
			{
				array.add(expr <= nb-1);
				nb_inf_paths++;
				nb_inq++;
				expr.end();
				found_inf = true;				
			}

		}//end for j

		if(found_inf) break;
	}//end for k

	return nb_inq;
}

int ExactBrpSepCvrpsep::SeparateMaxRouteDistCut(IloRangeArray array)
{
	int inqs = 0;
	for(int i=0;i<_graph->GetPathCount();i++)
	{
		double dist = 0.0; 
		int nb = 0;
		
		IloExpr expr(array.getEnv());
		
		std::vector<Node*> path = _graph->GetPath(i);
		for(size_t k=1;k<path.size();k++)
		{
			ExBrpArcO * a = _graph->GetArc(path[k-1]->no,path[k]->no);
			dist += a->cost; 
			
			if(a->from->type != NODE_TYPE_CUSTOMER || a->to->type != NODE_TYPE_CUSTOMER) continue;
			
			expr += _x[ a->index ];
			nb++;
		}
		
		if( dist > (double)Parameters::MaxRouteDistance())
		{
			array.add( expr <= nb - 1 ); 
			inqs++;
			//std::cout << "MaxRouteCut dist:" << dist << " nb:" << nb << " nodes:" << (int)path.size()-2 << " <= " << nb-1 << std::endl;
		}

	}
	
	nb_max_route_cuts += inqs;
	
	return inqs;
}

void ExactBrpSepCvrpsep::SeparateOptCut(IloRangeArray array)
{
	double sum_path_recourse = 0.0;
	int nb = 0;	
	
	IloExpr expr(array.getEnv());
	for(int i=0;i<_graph->GetPathCount();i++)
	{	
		std::vector<Node*> path = _graph->GetPath(i);
		double rec = RouteFeasibility::RecourseCost(_prob, path);
		sum_path_recourse += rec;
		
		for(size_t k=1;k<path.size();k++)
		{	
			ExBrpArcO * a = _graph->GetArc(path[k-1]->no,path[k]->no);
			if(a->from->type != NODE_TYPE_CUSTOMER || a->to->type != NODE_TYPE_CUSTOMER) continue;
			
			expr -= rec * _x[ a->index ];
			nb++;
		}

	}	
	
	if(sum_path_recourse > 0.0001)
	{
		array.add(_theta + expr >= sum_path_recourse * (1 - nb));
		std::cout << "OptCut: " << nb_opt_cuts << " Cut: " << _theta << " <= " << sum_path_recourse << std::endl;
		nb_opt_cuts++;
	}
	expr.end();
	
	double sol_distance = _graph->GetCost();
	if(sol_distance + sum_path_recourse < best_sol)
	{
		best_sol_distance = sol_distance;
		best_sol_recourse = sum_path_recourse;
		best_sol = sol_distance + sum_path_recourse;
		printf("New solution cost:%.1lf dist:%.1lf rec:%.1lf paths:%d\n", best_sol, best_sol_distance, best_sol_recourse,_graph->GetPathCount());
		best_solution.clear();
		for(int i = 0; i < _graph->GetPathCount(); i++)
			best_solution.push_back( _graph->GetPath(i) );
	}	
}