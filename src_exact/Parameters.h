#ifndef _PARAMETERS_H
#define _PARAMETERS_H

#include <time.h>
#include <math.h>
#include <string>


class Parameters
{
	public:
		void Read(int arg, char ** argv);
		bool SetCityName(const char* filePath);
		static char* GetTargetsFileName(){ return (char *)targets_file_name.c_str(); }
		static char* GetInitialCapacitiesFileName(){ return (char *)initial_capacities_file_name.c_str(); }		

		static char* GetInstanceFileName(){return instance_file;}
		static int GetInstanceType(){return instance_type_int;}
		static char* GetReFileName(){ return (char *)re_file_name.c_str(); }
		static char* GetOutputFileName(){ return (char *)output_file_name.c_str(); }
		static char* GetCityName() {return (char*)city_name.c_str();}		
		static int GetNbStations(){ return nb_stations; }
		static int GetDelta(){ return delta; }
		static void SetDelta(int i){ delta=i; }
		static void SetHardQ(int i){ hard_Q=i; }
		static int GetHardQ(){ return hard_Q; }
		static void SetMaxRouteDistance(int i){ max_route_dist=i; }
		static int MaxRouteDistance(){ return max_route_dist; }

	private:

		static char * instance_file;
		static char * instance_type;
		static std::string re_file_name;
		static std::string output_file_name;
		static std::string city_name;
		static std::string targets_file_name;
		static std::string initial_capacities_file_name;			
		
		static int nb_stations;
		static int delta;
		static int hard_Q;
		static int max_route_dist;
		static int instance_type_int;
};


#endif
