#ifndef NODE_BRP
#define NODE_BRP

#include "Constants.h"
#include "Parameters.h"
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstdint>

class Node
{
	public:
		Node() :id(-1), origin_id(-1), demand(0), stationcapacity(0), distID(-1), no(0), type(0), serv_time(0), x(0), y(0), lat(0.0), lon(0.0), initialcapacity(-1),target(0), w_plus(0), w_minus(0), is_depot_duplicated(0), depot_distance(0.0), sumLambda(0), sumMu(0), minLambda(0), maxMu(0), sumGamma(0), sumZeta(0), maxZeta(0), minGamma(0), IsForwardFeasible(0), IsBackwardFeasible(0), Q(0)  {}


		int id;			//from 0 to n-1
		int origin_id;  //ID at the begining, it is never modified, only customers have a valid unique ID
		int stationcapacity;   // Station capacity
		int target;   //Initial occupational level of the station
		int initialcapacity;
		int distID;		//indicate which line and column that contains the distance/time in the matrices
		int no;			//personnal identifier, put what you want
		char type;		//type of the node
		int serv_time;
		double x;
		double y;
		int demand;
		double lat;
		double lon;
		int w_plus;
		int w_minus;
		int Q; // Only stored in the Update() function to print the feasibility quantities.
		
		std::vector<int> costs;
		
		/*---------Feasibility Quantities---------*/
		int16_t sumLambda; int16_t minLambda; int16_t sumGamma; int16_t minGamma;
		int16_t sumMu; int16_t maxMu; int16_t sumZeta; int16_t maxZeta;
		uint8_t IsForwardFeasible; uint8_t IsBackwardFeasible;
		void ResetFeasibilityQuantities(){ sumLambda = 0; minLambda = 0; minGamma = 0; sumMu = 0; maxMu = 0; maxZeta = 0; sumGamma = 0; sumZeta = 0; IsForwardFeasible = 0; IsBackwardFeasible = 0;}
		void ShowFeasibilityQuantities(){
			int lb = sumLambda - minLambda;
			int ub = sumMu + Q - maxMu;
			
			int lb1 = sumGamma - minGamma;
			int ub1 = sumZeta + Q - maxZeta;
			printf("Node:%d sumL:%d minL:%d sumM:%d maxM:%d sumG:%d minG:%d sumZ:%d maxZ:%d ForwardFeas?%d lb:%d ub:%d BackwardFeas?%d lb1:%d ub1:%d\n",no,sumLambda,minLambda,sumMu,maxMu,sumGamma,minGamma,sumZeta,maxZeta,IsForwardFeasible,lb,ub,IsBackwardFeasible,lb1,ub1);
		}
		/*----------------------------------------*/
		
		double depot_distance;
		void SetDepotDistance(double d){depot_distance = d;}
		
		int16_t is_depot_duplicated;
		int IsDuplicatedDepot(){return is_depot_duplicated;}
		
		bool IsNotInCycle;
		
		void SetInitialCapacity(int cap){initialcapacity = cap;}
		void SetCapacity(int cap){stationcapacity = cap;}
		void SetTarget(int t){ target = t; }
		void UpdateDemand(){ demand = target - initialcapacity; }
		void UpdateW(){ w_plus = initialcapacity; w_minus = stationcapacity - initialcapacity; }
		
		void Show()
		{
			if(type == NODE_TYPE_CUSTOMER)
			{
				printf("Node:%d type:Cust cap:%d initCap:%d tgt:%d dmd:%d wp:%d wm:%d lat:%.4lf lon:%.4lf\n", no, stationcapacity, initialcapacity, target, demand, w_plus, w_minus, lat, lon);

			}
			else if(type == NODE_TYPE_START_DEPOT)
			{
				printf("Node:%d type:stadepot lat:%.4lf lon:%.4lf\n", id, lat, lon);
			}
			else if(type == NODE_TYPE_END_DEPOT)
			{
				printf("Node:%d type:enddepot lat:%.4lf lon:%.4lf\n", id, lat, lon);
			}
		}

		bool IsCustomer()
		{
			return (type & NODE_TYPE_CUSTOMER) == NODE_TYPE_CUSTOMER;
		}
};

#endif
