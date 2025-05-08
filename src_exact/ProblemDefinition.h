#ifndef PROBLEM_DEF_H
#define PROBLEM_DEF_H

#include <stddef.h>
#include <vector>
#include <cstring>
#include "Constants.h"
#include "NodeBRP.h"
#include "DriverBRP.h"


class Prob
{
	public:
		Prob(): _nodes(0),_customers(0),_drivers(0),_distances(NULL),_dimension(0),_driver_count_lb(1), _upper_bound(9999999999.9),_delete_matrices(true)
		{
			_nodes.reserve(4000);		
			_customers.reserve(4000);
			_drivers.reserve(300);
		}

		~Prob()
		{
			if(_delete_matrices)
			{
				if(	_distances != NULL)
				{
					for(int i = 0 ; i < _dimension ; i++)
						delete [] _distances[i];
					delete [] _distances;
				}
				_distances = NULL;
			}
		}

		void AddCustomer(Node * n){ _customers.push_back(n->id); }
		void AddNode(Node & n){ _nodes.push_back(n); }

		int GetCustomerCount(){ return (int)_customers.size();}
		int GetNodeCount(){ return (int)_nodes.size();}

		Node* GetCustomer(int i){ return &_nodes[ _customers[i] ]; }
		Node* GetNode(int i){ return &_nodes[i]; }

		void AddDriver(Driver & d){ _drivers.push_back(d);}
		int GetDriverCount(){ return (int)_drivers.size();}
		Driver* GetDriver(int i){ return &_drivers[i];}

		void SetMatrices(double ** d, int dim){ _distances = d; _dimension = dim;}
		double ** GetDistances(){ return _distances;}
		double GetDistance(Node * i, Node * j){ return _distances[i->distID][j->distID];}
		double GetDist(Node * i, Node * j){ return _distances[i->distID][j->distID];}

		void ShowNodes()
		{
			for(size_t i=0;i<_nodes.size();i++)
				_nodes[i].Show();
		}

		double GetUpperBound(){return _upper_bound;}
		void SetUpperBound(double ub){_upper_bound = ub;}
		
		double GetRecLowerBound(){ return _lower_bound; }
		void SetLowerBound(double lb){ _lower_bound = lb; }

		int GetDriverCountLB(){return _driver_count_lb;}
		void SetDriverCountLB(int d){_driver_count_lb = d;}
		void SetVehicleCapacity( int Q )
		{
			for( Driver & d : _drivers)
				d.capacity = Q;
		}
		void SetQtot(int q){Qtot=q;}
		int GetQtot(){return Qtot;}
		void SetInitialCapacity(int i, int cap){ _nodes[ _customers[i] ].SetInitialCapacity(cap); }
		void SetCapacity(int i, int cap){ _nodes[ _customers[i] ].SetCapacity(cap); }
		void SetTarget(int i, int t){ _nodes[ _customers[i] ].SetTarget(t) ; }
		void DuplicateAndStore(int i) 
		{
			Node* originalNode = &_nodes[_customers[i]]; // Retrieve the original node
			Node* newNode = new Node(*originalNode);

			int excess = std::abs(originalNode->target - originalNode->initialcapacity) - Parameters::GetHardQ();
			double excessPercentage = excess / (double)std::abs(originalNode->target - originalNode->initialcapacity);

			int new_stationcapacity = std::ceil(originalNode->stationcapacity * excessPercentage);
			int new_initialcapacity = std::ceil(originalNode->initialcapacity * excessPercentage);
			int new_target = std::ceil(originalNode->target * excessPercentage);

			originalNode->stationcapacity -= new_stationcapacity; newNode->stationcapacity = new_stationcapacity;
			originalNode->initialcapacity -= new_initialcapacity; newNode->initialcapacity = new_initialcapacity;
			originalNode->target -= new_target; newNode->target = new_target;

			originalNode->UpdateDemand(); originalNode->UpdateW();
			newNode->UpdateDemand(); newNode->UpdateW();

			newNode->id = _nodes.size(); 
			newNode->no = _nodes.size() + 1;

			// Add the new node to _nodes and _customers
			AddNode(*newNode);
			AddCustomer(newNode);
			
			/*printf("Modified node:\n");
			_nodes[_customers[i]].Show();
			printf("New node:\n");
			newNode->Show();
			printf("Excess:%d percentage:%.2lfx100 New cap:%d initCap:%d tgt:%d\n",excess,excessPercentage,new_stationcapacity,new_initialcapacity,new_target);*/
			//getchar();
		}		
	
	private:
		std::vector<Node> _nodes;			//list of nodes
		std::vector<int> _customers;		//list of nodes that are customers
		std::vector<Driver> _drivers;		//list of drivers

		double ** _distances;
		int _dimension;

		int _driver_count_lb;
		double _upper_bound;
		double _lower_bound;
		bool _delete_matrices;		//if the problem definition comes from a copy it is false,
											//if it is original it is true
											
		int Qtot;
  
};

#endif
