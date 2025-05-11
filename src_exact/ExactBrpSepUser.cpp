#include "ExactBrpSepUser.h"

ExactBrpSepUser::ExactBrpSepUser(IloEnv env, ExactBrpGraphO* graph, IloNumVarArray x, IloNumVar theta) : _env(env), _graph(graph), _x(x), _theta(theta), _prob(graph->GetProblem())
{
	nb_inf_sets = 0;
	nb_inf_paths = 0;
	nb_sub_tours = 0;
	nb_sub_tour_frac = 0;
	nb_opt_cuts = 0;
	nb_max_route_cuts = 0;
	best_sol = best_sol_recourse = best_sol_distance = 9999999999;

	_component.resize(_graph->GetNodeCount(), -1);
}

//Separates inequalities from fractional solutions
void ExactBrpSepUser::SeparateFrac(IloRangeArray array)
{	
	
	for(int i = 0; i < _graph->GetNodeCount(); i++)//stations + depot
		_component[i] = -1;

	int comp = -1;
	for(int i=1;i<_graph->GetNodeCount();i++)
		if(_component[_graph->GetNode(i)->no] == -1)
			ResearchConnectedComponent(_graph->GetNode(i), ++comp);
	comp++;

	std::vector< std::vector<Node*> > components(comp);
	for(int i=0;i<comp;i++)
		components[i].reserve( _graph->GetNodeCount() );
	
	for(int i=1;i<_graph->GetNodeCount();i++)
	{
		Node * n = _graph->GetNode(i);
		components[ _component[n->no] ].push_back(n);
	}
	
	for(int i=0;i<comp;i++)
	{
		if(components[i].size()<=2) continue;//Added to the root
		
		double sumv = 0;
		int nb=components[i].size();

		int nb_veh = RouteFeasibility::GetDriverCount(_prob, components[i]);
		IloExpr expr(array.getEnv());
		for(int m = 0; m < nb; m++)
			for(int n = 0; n < nb; n++)
				if (m != n)
				{
					ExBrpArcO* arc = _graph->GetArc(components[i][m]->no, components[i][n]->no);
					if (arc == NULL) continue;
					sumv += arc->value;
					expr += _x[arc->index];
				}
			
		if(sumv > nb - nb_veh + 0.2)
		{
			array.add(expr <= nb - nb_veh);
			nb_sub_tour_frac++;

		}
		expr.end();
	}		
	
	//Searched for connected components to the depot
	/*for(int i = 0; i < _graph->GetNodeCount(); i++)//stations + depot
		_component[i] = -1;
	ResearchDepotComponent(_graph->GetNode(0), 0);
	
	//printf("Components:");
	//for(size_t i=0;i<_component.size();i++)
	//	printf(" co%zu:%d",i,_component[i]);
	//printf("\n");
	
	int cntr = 0;
	double sumv = 0.0;

	while (cntr < _component.size())
	{
		std::vector<Node*> tour;
		tour.reserve(_graph->GetNodeCount());
		tour.push_back(_graph->GetNode(0));
		sumv = 0.0;

  		// Flag to detect whether we added anything
		bool added = false;

		// Build a tour while the component marker is 0
		while (cntr < _component.size() && _component[cntr] == 0)
		{
			if (cntr + 1 < _graph->GetNodeCount())
			{
				ExBrpArcO* a = _graph->GetArc(tour.back()->no, _graph->GetNode(cntr + 1)->no);
				if (a != NULL)
					sumv += a->value;
				tour.push_back(_graph->GetNode(cntr + 1));
				added = true;
			}
			cntr++; // always increment to avoid infinite loop
		}

		// Skip the -1 that marks the end of the component (if present)
		if (cntr < _component.size() && _component[cntr] == -1)
			cntr++;

		// If nothing was added, skip
		if (!added)
			continue;

		// Complete the tour by returning to depot
		ExBrpArcO* a = _graph->GetArc(tour.back()->no, _graph->GetNode(0)->no);
		if (a != NULL)
			sumv += a->value;

		tour.push_back(_graph->GetNode(0));
		if (tour.size() <= 5)
			continue;
		
		if(sumv < (int)tour.size() - 2 - 1) 
			continue;
		
		int nb_veh = RouteFeasibility::GetDriverCount(_prob, tour);
		
		//inq : sumv <= tour.size() - 2 - nb_veh
		if (sumv > tour.size() - 2 - nb_veh + 0.2)
			TestAndAddInfeasiblePath(tour, array);
	}*/	
}

//Separates inequalities from integer solutions
void ExactBrpSepUser::SeparateInt(IloRangeArray array)
{
	if(!SeparateUserCapRecursive(array))
	{
		_graph->MakePaths();
		for(int i=0;i<_graph->GetPathCount();i++)
		{
			std::vector<Node*> path = _graph->GetPath(i);
			if(path.size()<=5) continue; // All 2,3-size inf sets are added at the root as inf arcs.
			//Additionally, all single routes are feasible.
			//So an inf set needs to be at least: 0-i-j-k-l-0, size>=6
			TestAndAddInfeasiblePath(path,array);
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

bool ExactBrpSepUser::SeparateUserCapRecursive(IloRangeArray array)
{
	bool added_inq=false;
	
	for(int i = 0; i < _graph->GetNodeCount(); i++)
		_component[i] = -1;

	int comp = -1;
	for(int i = 0; i < _graph->GetNodeCount(); i++)
		if (_component[_graph->GetNode(i)->no] == -1)
			ResearchConnectedComponent(_graph->GetNode(i), ++comp);
	
	//Could be done in the same fashion as the fractional components
	for(int co = 1; co <= comp; co++)
	{
		//get the ordered cycle
		std::vector<Node*> tour; tour.reserve( _graph->GetNodeCount() );
		for(int i=1;i<_graph->GetNodeCount();i++)
			if(_component[_graph->GetNode(i)->no] == co)
			{
				ExBrpArcO* ar = _graph->GetArcsOutPos(_graph->GetNode(i)->no, 0);
				tour.push_back(ar->from);

				Node* from = ar->from;
				Node* to = ar->to;
				while (1)
				{
					tour.push_back(to);
					from = to;
					ar = _graph->GetArcsOutPos(to->no, 0);
					to = ar->to;
					if(to->no == _graph->GetNode(i)->no) break;
				}
				break;
			}
		if (tour.size() == 0) continue;

		int r_S = RouteFeasibility::GetDriverCount(_prob, tour);
		
		double sumv=0.0;
		IloExpr expr(array.getEnv());
		for(int m = 0; m < tour.size(); m++)
			for(int n = 0; n < tour.size(); n++)
				if (m != n)
				{
					ExBrpArcO* arc = _graph->GetArc(tour[m]->no, tour[n]->no);
					if (arc != NULL) expr += _x[arc->index];
					if (arc != NULL) sumv += arc->value;
				}
		double rhs = (tour.size()) - r_S;

		array.add(expr <= rhs);
		nb_sub_tours++;
		expr.clear();
		added_inq=true;

		expr.end();

	}// End connected component
	
	if(!added_inq) return added_inq; // Easier to find inf sets later
	
	int cur_path = 0;
	for(int i = 0; i < _graph->GetArcsOutPosCount(0); i++)
	{
		std::vector<Node*> tour; tour.reserve( _graph->GetNodeCount() );
		int nb_arcs=0;
		ExBrpArcO* ar = _graph->GetArcsOutPos(0, i);
		tour.push_back(ar->from);
		nb_arcs++;

		Node* from = ar->from;
		Node* to = ar->to;
		while (1)
		{
			tour.push_back(to);
			from = to;
			if (from->no == 0) break;
			ar = _graph->GetArcsOutPos(to->no, 0);
			to = ar->to;
			nb_arcs++;
		}

		if(tour.size()<=5) return added_inq; // All 2,3-size inf sets are added at the root as inf arcs.
		//Additionally, all single routes are feasible.
		//So an inf set needs to be at least: 0-i-j-k-l-0, size>=6

		int inf_set_inq_found = TestAndAddInfeasiblePath(tour, array);
	}
	return added_inq;

}

void ExactBrpSepUser::ResearchConnectedComponent(Node* n, int comp)
{
	_component[n->no] = comp;
	for(int i = 0; i < _graph->GetArcsOutPosCount(n->no); i++)
	{
		ExBrpArcO* a = _graph->GetArcsOutPos(n->no, i);
		if (a->to->no!=0 && _component[a->to->no] == -1 && a->value >= 0.45)//more subtours than inf set
		{
			_component[a->to->no] = comp;
			ResearchConnectedComponent(_graph->GetNode(a->to->no),comp);
		}
	}
}

void ExactBrpSepUser::ResearchDepotComponent(Node* n, int comp)
{
	_component[n->no] = comp;
	for(int i = 0; i < _graph->GetArcsOutPosCount(n->no); i++)
	{
		ExBrpArcO* a = _graph->GetArcsOutPos(n->no, i);
		if (_component[a->to->no] == -1 && a->value >= 0.60)
		{
			_component[a->to->no] = comp;
			ResearchDepotComponent(_graph->GetNode(a->to->no),comp);
		}
	}
}

// Test if a the path is infeasible
// In such case, it searches for the smallest infeasible set
// The smallest infeasible path
// Note that the first and last nodes are the depot
int ExactBrpSepUser::TestAndAddInfeasiblePath(std::vector<Node*> & path, IloRangeArray array)
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

int ExactBrpSepUser::SeparateMaxRouteDistCut(IloRangeArray array)
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

void ExactBrpSepUser::SeparateOptCut(IloRangeArray array)
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
