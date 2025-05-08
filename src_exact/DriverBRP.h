#ifndef DRIVER_BRP
#define DRIVER_BRP

class Driver
{
	public:
		Driver() : id(-1), StartNodeID(-1), EndNodeID(-1), capacity(-1), sum_demand(-1), curDistance(-1), curRecourse(-1), sumPosDmd(0), sumNegDmd(0), sumWp(0), sumWm(0), is_feasible(true), nb_customers(0) {}
		int id;				//from 0 to nb drivers - 1
		int StartNodeID;	//
		int EndNodeID;		//
		int capacity;
		int sum_demand;
		double curDistance;
		double curRecourse;	
		bool is_feasible;
		int16_t nb_customers;
		void SetCapacity(int q){ capacity = q; } 
		
		
		/*-----QuantitiesForFeasibilityCheck------*/
		int sumPosDmd; int sumNegDmd; int sumWp; int sumWm; 
		void ResetFeasibilityQuantities(){ sumPosDmd = 0; sumNegDmd = 0; sumWp = 0; sumWm = 0; }
		/*----------------------------------------*/
		
		void Show() { printf("Driver:%d Start:%d End:%d Q:%d nbCust:%d sumDmd:%d Dist:%.2lf Rec:%.2lf isFeas:%d\n",id,StartNodeID,EndNodeID,capacity,nb_customers,sum_demand,curDistance,curRecourse,is_feasible); }

		int GetStartNodeId(){return StartNodeID;}
		int GetEndNodeId(){return EndNodeID;}
};

#endif
