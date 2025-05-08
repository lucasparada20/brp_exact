#include "RandomRemoveBRP.h"

void RemoveRandomBRP::Remove(Sol & s, int count)
{
	//printf("RandomRemoveBRP\n");
	vect.clear(); vect.reserve( s.GetCustomerCount() );
	for(int i = 0 ; i < s.GetCustomerCount() ; i++)
		if( s.GetAssignedTo( s.GetCustomer(i) ) != NULL)
			vect.push_back( s.GetCustomer(i) );
	
	int cpt = std::min(count, (int)vect.size());
	for(int i = 0 ; i < cpt ; i++)
	{
		int index = vect.size() == 0 ? 0 : (rand() % (vect.size() - i));
		s.RemoveAndUnassign( vect[index] );
		vect[index] = vect[ vect.size() - i - 1];
	}
	
}