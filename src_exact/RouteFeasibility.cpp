#include "RouteFeasibility.h"

double RouteFeasibility::Calculate(Prob * prob)
{	
	std::vector<Node*> stations;
	stations.push_back(prob->GetNode(prob->GetNodeCount() - 2));
	for (int i = 0; i < prob->GetCustomerCount(); i++)
		stations.push_back(prob->GetCustomer(i));
	stations.push_back(prob->GetNode(prob->GetNodeCount() - 1));

	double lb = Calculate(prob, stations);

	//printf("L1:%.3lf Cost:%.2lf time:%.3lf\n",l1,lb, time_taken_l1);

	return lb;
}

double RouteFeasibility::Calculate(Prob* prob, std::vector<Node*>& stations)
{
	int nb_iter = 1;
	int Q = prob->GetDriver(0)->capacity;
	int dmds = 0;
	int lb_left = 0, ub_left = 0, lb_right = 0, ub_right = 0;
	for (size_t l = 0; l < stations.size(); l++)
	{
		if(stations[l]->type != NODE_TYPE_CUSTOMER) continue;

		Node * n = stations[l];
		int dmd = n->demand;
		dmds += dmd;

		if(dmd > 0) { lb_left += dmd; ub_left += dmd; }
		ub_left += n->w_minus;

		if(dmd < 0) { lb_right -= dmd; ub_right -= dmd; }
		ub_right += n->w_plus;
	}

	while(ub_left+Q < lb_right || lb_left > ub_right+Q)
	{
		Q += prob->GetDriver(0)->capacity;
		nb_iter++;
	}

	//printf("ub_left:%d < lb_right:%d || lb_left:%d > ub_right:%d\n", ub_left, lb_right,lb_left , ub_right);
	if(ub_left+Q  < lb_right || lb_left > ub_right+Q)
		return 9999999999;
	else
		return std::max(0, std::abs(dmds) - Q);
}

bool RouteFeasibility::IsFeasible(Prob * prob, std::vector<Node*> & path, bool show) 
{
	int Q = prob->GetDriver(0)->capacity;
	int min_lambda = 0; int sum_lambda = 0; 
	int max_mu = 0; int sum_mu = 0;
	if(show)
		printf("Path in IsFeasible?\n");
	for (size_t i=0;i<path.size();i++)
	{
		Node* n = path[i];
		if(n->type != NODE_TYPE_CUSTOMER) continue;//ignore depots

		int lambda_i = std::max(-Q, n->demand - n->w_plus);
		int mu_i = std::min(Q, n->demand + n->w_minus);
		sum_lambda += lambda_i;
		min_lambda = std::min(sum_lambda, min_lambda);
		sum_mu += mu_i;
		max_mu = std::max(sum_mu, max_mu);

		int lb = sum_lambda - min_lambda;
		int ub = sum_mu + Q - max_mu;
		
		if(show) printf("EndLoad Lb>Ub?%d no:%d lb:%d ub:%d dmd:%d wp:%d wm:%d lambda:%d mu:%d sum_lambda:%d sum_mu:%d min_lambda:%d max_mu:%d\n",lb>ub,n->no,lb,ub,n->demand,n->w_plus,n->w_minus,lambda_i,mu_i,sum_lambda,sum_mu, min_lambda, max_mu);
		if(lb > ub)
			return false;
	}
	return true;
}

int RouteFeasibility::GetDriverCount(Prob* prob)
{	
	std::vector<Node*> stations;
	for (int i = 0; i < prob->GetCustomerCount(); i++)
		stations.push_back(prob->GetCustomer(i));
	return GetDriverCount(prob, stations);
}

//skips the depots
int RouteFeasibility::GetDriverCount(Prob * prob, std::vector<Node*>& path)
{
	int Q = prob->GetDriver(0)->capacity;
	//Quantities to compute ...
	int nb_drivers = 1;
	int sum_dmd = 0; 
	int lb_left = 0; int ub_left = 0;
	int lb_right = 0; int ub_right = 0;
	for (int i = 0; i < path.size(); i++)
	{
		if(path[i]->type != NODE_TYPE_CUSTOMER) continue;

		Node * n = path[i];
		sum_dmd += n->demand;
		
		if(n->demand > 0) { lb_left += n->demand; ub_left += n->demand; }
		ub_left += n->w_minus;

		if(n->demand < 0) { lb_right -= n->demand; ub_right -= n->demand; }
		ub_right += n->w_plus;
	}
	//get the minimum number of vehicles
	while(ub_left + Q*nb_drivers < lb_right || lb_left > ub_right + Q*nb_drivers)
		nb_drivers++;
		
	return nb_drivers;
}

double RouteFeasibility::RecourseCost(Prob* prob, std::vector<Node*>& path)
{
    const int Q = prob->GetDriver(0)->capacity;
    const int BigM = 9999;

    std::vector<int> buffer1(Q + 1, 0);
    std::vector<int> buffer2(Q + 1, 0);
    int* prev = buffer1.data();
    int* next = buffer2.data();

    for (int i = path.size() - 2; i >= 1; --i) {
        Node* n = path[i];
        int demand = n->demand;

        for (int q = 0; q <= Q; ++q) {
            int best = BigM;

            // w_plus
            int u_min_plus = std::max(0, q + demand - Q);
            int u_max_plus = std::min(n->w_plus, q + demand);
            for (int u = u_min_plus; u <= u_max_plus; ++u) {
                int qprime = q + demand - u;
                best = std::min(best, u + prev[qprime]);
                if (best == 0) break;
            }

            // w_minus
            if (best > 0) {
                int u_min_minus = std::max(0, -q - demand);
                int u_max_minus = std::min(n->w_minus, Q - q - demand);
                for (int u = u_min_minus; u <= u_max_minus; ++u) {
                    int qprime = q + demand + u;
                    best = std::min(best, u + prev[qprime]);
                    if (best == 0) break;
                }
            }

            next[q] = best;
        }

        std::swap(prev, next);
    }

    int cost = *std::min_element(prev, prev + Q + 1);
    return Parameters::GetDelta() * cost;
}


/*double RouteFeasibility::RecourseCost(Prob * prob, std::vector<Node*>& path)
{
	double cost = 0;
	int Q = prob->GetDriver(0)->capacity;
	int BigM = 9999;
	int * prev = new int[Q+1];
	int * next = new int[Q+1];
	
	for(int q=0;q<=Q;q++)
		next[q] = prev[q] = 0;

	for(int i=path.size()-2;i>=1;i--)
	{
		Node * n = path[i];
		for(int q=0;q<=Q;q++)
		{
			int best = BigM;
			
			// w_plus
			for (int u = 0; u <= n->w_plus; ++u) {
				int qprime = q + n->demand - u;
				if (qprime < 0 || qprime > Q) continue;
				best = std::min(best, u + prev[qprime]);
				if (best == 0) break;  // early exit
			}

			// If not pruned yet, try w_minus
			if (best > 0) {
				for (int u = 0; u <= n->w_minus; ++u) {
					int qprime = q + n->demand + u;
					if (qprime < 0 || qprime > Q) continue;
					best = std::min(best, u + prev[qprime]);
					if (best == 0) break;  // early exit
				}
			}
			//for(int u=0;u<=n->w_plus;u++)
			//	if(q+n->demand-u >= 0 && q+n->demand-u<=Q)
			//		best = std::min(best, u+prev[q+n->demand-u]);
					
			//for(int u=0;u<=n->w_minus;u++)
			//	if(q+n->demand+u >= 0 && q+n->demand+u<=Q)
			//		best = std::min(best, u+prev[q+n->demand+u]);
					
			next[q] = best;
			
			//printf("ContinueRec Node:%d q:%d Best Cost:%d\n", n->no, best, q);
			//getchar();
		}
		
		int * temp = next;
		next = prev;
		prev = temp;		
	}

	int cost2 = BigM;
	for(int q=0;q<=Q;q++)
		cost2 = std::min(cost2 , prev[q]);
	cost += cost2;
	
	//printf("ContinueRec cost:%d\n", cost2);
	//getchar();	
	
	delete [] prev;
	delete [] next;
	return Parameters::GetDelta() * cost;
}*/

double RouteFeasibility::CalculateWithMinDriverCount(Prob* prob)
{
	std::vector<Node*> stations;
	for (int i = 0; i < prob->GetCustomerCount(); i++)
		stations.push_back(prob->GetCustomer(i));
	return CalculateWithMinDriverCount(prob, stations);
}
double RouteFeasibility::CalculateWithMinDriverCount(Prob* prob, std::vector<Node*>& stations)
{
	//first sum the demands
	int dmds = 0;
	int lb_left(0);
	int ub_left(0);
	int lb_right(0);
	int ub_right(0);
	
	for (size_t l = 0; l < stations.size(); l++)
	{
		if(stations[l]->type != NODE_TYPE_CUSTOMER) continue;

		Node * n = stations[l];
		int dmd = n->demand;
		dmds += dmd;

		if(dmd > 0) { lb_left += dmd; ub_left += dmd; }
		ub_left += n->w_minus;

		if(dmd < 0) { lb_right -= dmd; ub_right -= dmd; }
		ub_right += n->w_plus;
	}

	//get the minimum number of vehicles
	int Q = prob->GetDriver(0)->capacity;
	int nb_drivers = 1;
	
	while(ub_left + Q*nb_drivers < lb_right || lb_left > ub_right + Q*nb_drivers)
		nb_drivers++;

	double lb = 0;
	lb += std::max(0, std::abs(dmds) - Q*nb_drivers);

	return lb;
}