#include "LoadBRP.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <climits>

constexpr double EarthRadius = 6371.0; // in kilometers

void LoadBRP::Load_slr_instance(Prob & pr, char * filename)
{
	Load_coord(pr, Parameters::GetInstanceFileName());
	
	std::string targets_file_name_str = "instances_slr/targets/targets_" + std::string(Parameters::GetCityName()) + ".txt";
	Load_targets(pr, targets_file_name_str.c_str());
	
	std::string status_file_name_str = "instances_slr/status/" + std::string(Parameters::GetCityName()) + "_station_status.json";
	Load_json_station_status(pr, status_file_name_str.c_str());
	
	// Feasibility checks ...
	int max_abs_dmd = 0; int max_station_cap = 0;
	for(int i=0;i<pr.GetCustomerCount();i++)
	{
		//if(pr.GetNode(i)->IsDuplicatedDepot()) continue;
		
		pr.GetNode(i)->UpdateDemand(); 
		pr.GetNode(i)->UpdateW();
		max_abs_dmd = std::max( std::abs(pr.GetCustomer(i)->demand), max_abs_dmd );
		max_station_cap =  std::max( pr.GetCustomer(i)->demand, max_station_cap );
	}
	pr.SetVehicleCapacity( std::max( max_abs_dmd, max_station_cap ) );
	printf("In main: Delta:%d Vehicle Capacity Q:%d max_|q|:%d max_station_cap:%d drvCount:%d\n",Parameters::GetDelta(),pr.GetDriver(0)->capacity,max_abs_dmd,max_station_cap,pr.GetDriverCount()); //getchar();
	//pr.ShowNodes();
	
	//Setting a hard max vehicle cap Q = 40
	Parameters::SetHardQ(40); 
	//if(Parameters::GetHardQ() < pr.GetDriver(0)->capacity)
	{	
		std::vector<Node*> node_vec; int nbBigStationCap = 0;
		for(int i=0;i<pr.GetCustomerCount();i++)
		{
			Node * n = pr.GetCustomer(i);
			if(n->demand > Parameters::GetHardQ() || n->demand < -1*Parameters::GetHardQ())
			{
				n->Show();
				node_vec.push_back( n );
			}
			if( n->stationcapacity > Parameters::GetHardQ() ) nbBigStationCap++;
		}
		printf("Changing Q:%d to HardQ:%d NodesAboveHardQ:%d nbBigStationCap:%d\n",pr.GetDriver(0)->capacity,Parameters::GetHardQ(),(int)node_vec.size(),nbBigStationCap);
		for(int i=0;i<node_vec.size();i++)
			pr.DuplicateAndStore( node_vec[i]->id );
		for(int i=0;i<pr.GetDriverCount();i++)
			pr.GetDriver(i)->SetCapacity( Parameters::GetHardQ() );
	}

	
	//Feasibility test
	printf("Checking feasibility of all stations ...\n");
	for(int i=0;i<pr.GetCustomerCount();i++)
	{
		std::vector<Node*> path(1,pr.GetCustomer(i));
		bool IsFeas = RouteFeasibility::IsFeasible(&pr,path,false);
		//printf("Bool:%d ",IsFeas);
		if(IsFeas == false)
		{
			printf("Infeasible node:");
			pr.GetCustomer(i)->Show();
			exit(1);
		}
	}	
	printf("All customers feasible! ...\n");	
}

double LoadBRP::CalculateHarvesineDistance(Node * n1, Node * n2) {
    double lat1 = n1->lat;
    double lon1 = n1->lon;
    double lat2 = n2->lat;
    double lon2 = n2->lon;

    // Convert latitude and longitude from degrees to radians
    lat1 *= M_PI / 180.0;
    lon1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    lon2 *= M_PI / 180.0;

    // Haversine formula
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double distance = EarthRadius * c;

    return distance;
}

void LoadBRP::Load_coord(Prob & pr, const char * filename)
{
    std::cout << "In the Load station coordinates:\n";
    std::ifstream infile(filename);
    if (!infile)
    {
        std::cerr << "Error in the input filename: " << filename << std::endl;
        exit(1);
    }
    std::cout << "Loading coordinates from: " << filename << std::endl;
	
	//Assumption for maximum route duration
	//1.-Amazon drivers drive, on average 100+ miles per day. We thus set the hard limit to <= 100 miles = 160 km
	Parameters::SetMaxRouteDistance(160);
	
    int stations, Qtot, Q;
	std::string line;
    // Read the number of stations
    std::getline(infile, line);
    stations = std::stoi(line);
    // Read Qtot
    std::getline(infile, line);
    Qtot = std::stoi(line);
    // Set Qtot for pr
    pr.SetQtot(Qtot);
	
    Q = 40; // Real Bixi cap from May, 2024
    std::cout << "stations:" << stations << " Qtot:" << Qtot << " Q:" << Q << std::endl;

    std::vector<double> lat(stations, 0.0);
    std::vector<double> lon(stations, 0.0);
    std::vector<int> cap(stations, 0);
    std::vector<std::string> bss_id_vec(stations, "");
	
    std::getline(infile, line);
    std::istringstream iss(line);
	int depot_cap = -1; double depot_lat = -1.0; double depot_lon = -1.0;
	if(!(iss >> depot_cap >> depot_lat >> depot_lon ))
	{
		std::cerr << "Error reading depot line of file " << filename << std::endl;
		std::cout << "line:" << line << std::endl;
		exit(1);		
	} else {
		cap[0] = depot_cap; lat[0] = depot_lat; lon[0] = depot_lon;
	}

    for (int i = 1; i < stations; i++)
    {
        std::getline(infile, line);
		iss.clear(); // Clear any error flags
		iss.str(line); // Set the new string to parse
		std::istringstream iss(line);
		std::string station_id_str;
		// Now you can parse the rest of the line using iss
		if (!(iss >> station_id_str >> cap[i] >> lat[i] >> lon[i])) {
			std::cerr << "Error reading line " << i+1 << " of file " << filename << std::endl;
			std::cout << "line:" << line << std::endl;
			exit(1);
		} else {
			//std::cout << "i:" << i << " station_id:" << station_id_str << " cap:" << cap[i] << " lat:" << lat[i] << " lon:" << lon[i] << std::endl;
		}
        // Assuming station_id_str is the station ID, you can do something with it.
        // For example:
        bss_id_vec[i] = station_id_str;

        //std::cout << "i:" << i << " line:" << line << std::endl;
    }
	
	//'stations' is the integer nb of stations, from the instance .txt
	for(int i = 0; i< stations-1; i++)
	{
		Node n;
		n.id = i;
		n.no = i+1;
		n.distID = i+1;
		n.type = NODE_TYPE_CUSTOMER;
		n.stationcapacity = cap[i+1];
		n.lat = lat[i+1];
		n.lon = lon[i+1];
		
		bss_id_map[ bss_id_vec[i+1] ] = i; //The maps is: station_id from the bss = the node no
		
		pr.AddNode(n);
	}
	printf("Sample node:\n");
	pr.GetNode(0)->Show();
	/*int max_station_cap = *std::max_element(cap.begin(), cap.end());
	if(Q < *std::max_element(cap.begin(), cap.end()))
	{
		printf("Max Station Cap:%d > Q:%d\n",max_station_cap,Q);
		max_station_cap = (int)(std::ceil(max_station_cap / 10.0) * 10.0); // Round up to nearest tenth
		if(Q < max_station_cap)
		{
			printf("New Q:%d\n", max_station_cap);
		}
		Q = max_station_cap;
		//getchar();
	}*/
	
	/*if(Parameters::AddDepotStations())
	{
		printf("Additional depot nodes:\n");
		int nb_additional_stations = 0.1 * stations; //Rouding down the integer
		printf("Nodes no from .txt:%d New nodes no:%d\n",stations,stations+nb_additional_stations);
		printf("Last regular station:%d First additional station:%d\n",stations-2, stations-2);
		
		for(int i=stations-1; i<stations+nb_additional_stations-1; i++)
		{
			Node n;
			n.id = i;
			n.no = i+1;
			n.distID = i+1;
			n.type = NODE_TYPE_CUSTOMER;
			n.stationcapacity = Q;
			n.lat = lat[0];
			n.lon = lon[0];
			n.initialcapacity = Q;
			n.demand = 0;
			n.target = 0; // demand is also set as : target - initial_capacitie = 0 - 0 = 0
			n.is_depot_duplicated = 1;
			n.w_plus = Q;
			n.w_minus = Q;
			
			n.Show();
			pr.AddNode(n);			
		}
		
		stations += nb_additional_stations;
	}*/	
	//OG
	for(int i = 0; i< stations-1; i++)	
		pr.AddCustomer( pr.GetNode(i) );

	//for(int i = 0 ; i < std::min( stations, 30 ) ; i++)
	for(int i = 0 ; i < stations ; i++)
	{
		Node dep1;
		dep1.id = stations - 1 + i*2;
		dep1.no = 0;
		dep1.distID = 0;
		dep1.type = NODE_TYPE_START_DEPOT;
		dep1.stationcapacity = 0;
		dep1.lat = lat[0];
		dep1.lon = lon[0];

		Node dep2(dep1);
		dep2.id = stations + i*2;
		dep2.type = NODE_TYPE_END_DEPOT;
		dep2.stationcapacity = 0;
		dep2.lat = lat[0];
		dep2.lon = lon[0];

		
		Driver d;
		d.capacity = Q;
		d.StartNodeID = dep1.id;
		d.EndNodeID = dep2.id;
		d.id = i;

		pr.AddNode(dep1);
		pr.AddNode(dep2);
		pr.AddDriver(d);
	}
	printf("Sample driver:\n");
	pr.GetDriver(0)->Show();

	int dim = stations;
	double ** d = new double*[dim];
	
	// Vector to store all distances
	std::vector<double> all_distances;
	//printf("Distance Matrix:\n");
	for(int i=0;i<dim;i++)
   {
      Node * n1 = pr.GetNode(i);
      d[n1->distID] = new double[dim];
      for(int j=0;j<dim;j++)
      {
         Node * n2 = pr.GetNode(j);
		 //printf("distID:%d distID:%d\n",n1->distID,n2->distID);
		 //n1->Show();
		 //n2->Show();
		 
		 d[n1->distID][n2->distID] = i==j ? 0 : CalculateHarvesineDistance(n1,n2);		
		//printf("distID:%d distID:%d = %d\n",n1->distID,n2->distID,(int)d[n1->distID][n2->distID]);
		all_distances.push_back(d[n1->distID][n2->distID]); // Store the distance in the vector
		if(d[n1->distID][n2->distID] > 100.0) // An intercity distance of >100km should be wrong!
		{
			printf("distance:%.3lf Nodes:\n",d[n1->distID][n2->distID]); n1->Show(); n2->Show(); getchar();
		}
		/*if(d[n1->distID][n2->distID] < 0.001 && n1->distID != n2->distID) // An intercity distance of >100km should be wrong!
		{
			printf("distance:%.3lf Nodes:\n",d[n1->distID][n2->distID]); n1->Show(); n2->Show(); getchar();
		}*/

      }
	  //printf("\n");
	  //getchar();
   }
	//exit(1);
	pr.SetMatrices(d,dim);
	
	Node * depot = pr.GetNode( stations );
	if(depot->type == NODE_TYPE_CUSTOMER)
	{
		std::cerr << "Wrong depot node. Exiting ..." << std::endl;
		exit(1);
	}
	for(int i=0; i<pr.GetCustomerCount();i++)
	{
		double depot_distance = -1.0;
		depot_distance =  pr.GetDist(depot, pr.GetCustomer(i));
		pr.GetCustomer(i)->SetDepotDistance(depot_distance);
	}
	
	// Sort the vector of distances
	std::sort(all_distances.begin(), all_distances.end());

	// Calculate quantiles (for example, 25th, 50th, and 75th percentiles)
	double quantile_25 = all_distances[static_cast<int>(0.25 * all_distances.size())];
	double quantile_50 = all_distances[static_cast<int>(0.50 * all_distances.size())];
	double quantile_75 = all_distances[static_cast<int>(0.75 * all_distances.size())];

	// Print quantiles
	printf("25th percentile: %.2f[km]\n", quantile_25);
	printf("50th percentile (median): %.2f[km]\n", quantile_50);
	printf("75th percentile: %.2f[km]\n", quantile_75);
	printf("100th percentile: %.2f[km]\n", all_distances[ all_distances.size()-1 ]);
	
	printf("Stations loaded:%d\n",pr.GetCustomerCount());
}

void LoadBRP::Load_json_station_status(Prob & pr, const char * filename)
{
    printf("In the Load Json Station Status:\n");

    std::ifstream file_json(filename);
    if (!file_json)
    {
        printf("Error in the input filename: %s\n", filename);
        exit(1);
    }

    // Parse the JSON data
    nlohmann::json jsonData;
    file_json >> jsonData;
    nlohmann::json json_stations = jsonData["data"]["stations"];
    printf("Loading initial capacities from: %s NbStations: %d\n", filename, (int)json_stations.size());

    int found = 0;
    int counter = 0;
    for (const auto& json_station : json_stations)
    {
        std::string station_id = json_station["station_id"];
        counter++;
        if (counter % 50 == 0)
        {
            printf("Looped through %d stations from the json file ...\n", counter);
        }

        int bikes = 0;
        auto it = bss_id_map.find(station_id);
        if (it != bss_id_map.end())
        {
            if (json_station.contains("num_bikes_available"))
            {
                bikes = json_station["num_bikes_available"];
                pr.SetInitialCapacity(it->second, bikes);
                found++;
            }
        }
    }
    printf("Found %d/%d stations from the json station_status file! pausing execution ...\n", found, pr.GetCustomerCount()); //getchar();

    for (int i = 0; i < pr.GetCustomerCount(); i++)
    {
        if (pr.GetCustomer(i)->initialcapacity == -1)
        {
            pr.SetInitialCapacity(i, 0);
        }
        if (pr.GetCustomer(i)->initialcapacity > pr.GetCustomer(i)->stationcapacity)
        {
            pr.SetCapacity(i, pr.GetCustomer(i)->initialcapacity);
        }
        if (pr.GetCustomer(i)->initialcapacity < 0 || pr.GetCustomer(i)->target > pr.GetCustomer(i)->stationcapacity || pr.GetCustomer(i)->initialcapacity > pr.GetCustomer(i)->stationcapacity)
        {
            printf("Something happened on the way to heaven\n");
            pr.GetCustomer(i)->Show();
            exit(1);
        }
    }
	
	/*int i=0;
	for (const auto& json_station : json_stations)
	{
		if(i>pr.GetCustomerCount()) break;
		
		//std::cout << json_station << std::endl;
		
		int ebikes = 0;
		int bikes = 0;

		if (json_station.contains("num_bikes_available"))
		{
			bikes = json_station["num_bikes_available"];
		}
		if (json_station.contains("num_ebikes_available"))
		{
			ebikes =  json_station["num_ebikes_available"];
		}
		int totalBikes = ebikes + bikes;
		int stationCapacity = pr.GetNode(i)->stationcapacity;

		if (totalBikes <= stationCapacity) {
			pr.SetInitialCapacity(i, totalBikes);
		} else if (bikes <= stationCapacity) {
			pr.SetInitialCapacity(i, bikes);
		} else {
			pr.SetInitialCapacity(i, 0);
		}
		pr.GetNode(i)->Show();
		i++;
	}
	while(i<=pr.GetCustomerCount())
	{
		pr.SetInitialCapacity(i, 0);
		i++;
	}*/
}

void LoadBRP::Load_targets(Prob & pr, const char * filename)
{
	printf("In the Load Targets:\n");
	FILE * ff = fopen(filename,"r");
 	if(!ff)
	{
		printf("Error in the input filename:%s\n", filename);
		exit(1);
	}
	printf("Loading targets from:%s\n",filename);
	char line[100];
	int i=0;
	while (fgets(line, 100, ff) && i<=pr.GetCustomerCount()) 
	{
		int t;
		if(sscanf(line,"%d",&t)==1)
		{
			printf("tgt%d:%d ",i,t);
			pr.SetTarget(i,t);
			i++;
			//targets.push_back(t);
		} else{
			printf("Error parsing line:%s Load Targets\n", line);
		}		
	}
	printf("\n");
	fclose(ff); 
}

// To load DHIN instances
void LoadBRP::Load_brp_instance(Prob & pr, const char * filename)
{
	std::cout << "In the Load brp instances:\n";
    std::ifstream infile(filename);
    if (!infile)
    {
        std::cerr << "Error in the input filename: " << filename << std::endl;
        exit(1);
    }
		
    int stations, Q;
	std::string line;
    // Read the number of stations
    std::getline(infile, line);
    stations = std::stoi(line);
    // Read demands
    std::getline(infile, line);
    std::vector<int> demands; demands.reserve( stations );
	std::istringstream istr(line);
	
	//std::cout << "Dmds:" << line << std::endl;
	for(int i=0;i<stations;i++)
	{
		int no; istr >> no;
		demands.push_back( no );
	}
		
    // Read Q
    std::getline(infile, line);
	Q = std::stoi(line);
	
    std::cout << "stations:" << stations << " Q:" << Q << std::endl;

	std::vector< std::vector<int> > distance_matrix(stations, std::vector<int>(stations, 9999));
	for (int i = 0; i < stations; i++)
	{
		std::getline(infile, line);
		std::istringstream istr1(line);
		for (int j = 0; j < stations; j++)
		{
			double no; istr1 >> no;
			//printf("%.1lf ",no);
			distance_matrix[i][j] = no > 1e+008 ? 99999999 : no;
		}
		//printf("\n");
	}
	
	//for(size_t i=0;i<distance_matrix.size();i++)
	//{
	//	for(size_t j=0;j<distance_matrix[i].size();j++)
	//		printf("%d ",distance_matrix[i][j]);
	//	printf("\n");
	//}

	//'stations' is the integer nb of stations, from the instance .txt
	for(int i = 0; i< stations-1; i++)
	{
		Node n;
		n.id = i;
		n.no = i+1;
		n.distID = i+1;
		n.demand = demands[i+1];
		n.type = NODE_TYPE_CUSTOMER;
		
		pr.AddNode(n);
		n.Show();
	}
	//printf("Sample node:\n");
	//pr.GetNode(0)->Show();
	
	//OG
	for(int i = 0; i< stations-1; i++)	
		pr.AddCustomer( pr.GetNode(i) );
	
	for(int i = 0 ; i < stations ; i++)
	{
		Node dep1;
		dep1.id = stations - 1 + i*2;
		dep1.no = 0;
		dep1.distID = 0;
		dep1.type = NODE_TYPE_START_DEPOT;

		Node dep2(dep1);
		dep2.id = stations + i*2;
		dep2.type = NODE_TYPE_END_DEPOT;
		
		Driver d;
		d.capacity = Q;
		d.StartNodeID = dep1.id;
		d.EndNodeID = dep2.id;
		d.id = i;

		pr.AddNode(dep1);
		pr.AddNode(dep2);
		pr.AddDriver(d);
	}
	printf("Sample driver:\n");
	pr.GetDriver(0)->Show();

	int dim = stations;
	double ** d = new double*[dim];
	
	printf("Distance Matrix:\n");
	double min_val = 1e9;
	double max_val = -1e9;
	double sum = 0.0;
	int count = 0;

	for(int i = 0; i < dim; i++)
	{
		Node* n1 = pr.GetNode(i);
		d[n1->distID] = new double[dim];
		
		for(int j = 0; j < dim; j++)
		{
			Node* n2 = pr.GetNode(j);
			double val = (i == j) ? 0.0 : distance_matrix[n1->distID][n2->distID]; // original logic
			d[n1->distID][n2->distID] = val;

			if (i != j) 
			{
				if (val < min_val) min_val = val;
				if (val > max_val) max_val = val;
				sum += val;
				count++;
			}

			printf("%.1lf ", val);
		}
		printf("\n");
	}

	double avg_val = (count > 0) ? sum / count : 0.0;

	printf("Min: %.2lf, Avg: %.2lf, Max: %.2lf\n", min_val, avg_val, max_val); //getchar();
	pr.SetMatrices(d,dim);
	
	Node * depot = pr.GetNode( stations );
	if(depot->type == NODE_TYPE_CUSTOMER)
	{
		std::cerr << "Wrong depot node. Exiting ..." << std::endl;
		exit(1);
	}
}