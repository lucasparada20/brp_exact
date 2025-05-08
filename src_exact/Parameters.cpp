#include "Parameters.h"
#include "Constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <string>
#include <string.h>
#include <vector>
#include <limits>
#include<algorithm>
#include <iostream>

int Parameters::nb_stations=0;
int Parameters::delta=1;
int Parameters::hard_Q=1;
int Parameters::max_route_dist=1;
int Parameters::instance_type_int=0;

char instance_file_temp[100];
char instance_type_temp[100];

char * Parameters::instance_file = NULL;
char * Parameters::instance_type = NULL;
std::string Parameters::re_file_name;
std::string Parameters::output_file_name;
std::string Parameters::city_name;
std::string Parameters::targets_file_name;
std::string Parameters::initial_capacities_file_name;


bool Parameters::SetCityName(const char* filePath) {
    if (!filePath) {
        std::cerr << "Error: filePath is NULL!" << std::endl;
        return false;
    }

    std::cout << "Received filePath: " << filePath << std::endl;

    std::string full_path(filePath);
    // Extract directory path
    size_t last_slash = full_path.find_last_of('/');
    std::string base_path = (last_slash != std::string::npos) ? full_path.substr(0, last_slash) : ".";

    // Replace "instances" with "results"
    size_t pos = base_path.find("instances_dhin");
    if (pos != std::string::npos) {
        base_path.replace(pos, 9, "results");
    }

    // Extract city name and number
    size_t pos_start = last_slash + 1;
    size_t num_start = pos_start;
    
    while (num_start < full_path.size() && !isdigit(full_path[num_start])) {
		num_start++;
    }

    city_name = full_path.substr(pos_start, num_start - pos_start);
    std::string number_str = full_path.substr(num_start);

    std::cout << "Extracted base path: " << base_path << std::endl;
    std::cout << "Extracted city name: " << city_name << std::endl;
    std::cout << "Extracted number: " << number_str << std::endl;

    // Ensure nb_stations is valid
    nb_stations = (!number_str.empty() && isdigit(number_str[0])) ? std::stoi(number_str) : 0;

    // Construct file paths in "results/"
    re_file_name = base_path + "/re_" + city_name + number_str;
    output_file_name = base_path + "/out_" + city_name + number_str;

    // Debug output
    std::cout << "In parameters parsing ..." << std::endl;
    std::cout << "City name: " << city_name << std::endl;
    std::cout << "NbStations: " << nb_stations << std::endl;
    std::cout << "Re file name: " << re_file_name << std::endl;
    std::cout << "Out file name: " << output_file_name << std::endl;
	
	return true;
}

void Parameters::Read(int arg, char ** argv)
{
	printf("Reading parameters\n");
	for(int i=0;i<arg;i++)
	{
		char * first = strtok (argv[i]," ;=");
		char * second = strtok (NULL, " ;=");
		printf ("Parameter:%s value:%s\n",first,second);
		if(second == NULL) continue;

		if(strcmp(first, "instance_file") == 0)
		{
			strcpy(instance_file_temp, second);
			instance_file = instance_file_temp;
			SetCityName(instance_file);
		}		
		if(strcmp(first, "instance_type") == 0)
		{
			stpcpy(instance_type_temp,second);
			instance_type = instance_type_temp;
			if(strcmp(instance_type,"dhin")==0)
				instance_type_int = 1;
			if(strcmp(instance_type,"slr")==0)
				instance_type_int = 2;
		}
		else if(strcmp(first,"targets_file_name")==0)
		{
			targets_file_name = std::string(second);
		}
		else if(strcmp(first,"initial_capacities_file_name")==0)
		{
			initial_capacities_file_name = std::string(second);
		}		
	}
}
