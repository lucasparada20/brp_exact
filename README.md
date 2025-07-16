# brp_exact
An exact branch-and-cut algorithm for the bicycle repositioning problem. The algorithm solves a Cvrp-like problem using Cplex with UserCut and Lazy callbacks. The callbacks dynamically separate rounded capacity inequalities, infeasible paths, sets, and optimality cuts. Additionally, two implementations are presented as separation algorithms, using the so-called CVRPSEP package and user-defined routines.

## Building the code in Linux

1. Clone the repository and add executable permission to a script that will call CMake for you:

```shell
git clone https://github.com/lucasparada20/brp_exact.git
cd brp
chmod u+x cmake_script_exact.sh
```
2. The compiler needs an absolute path to your installed Cplex library. To provide the path, go into src/CMakeLists.txt and edit the following line:

```cmake
set(CPLEX_DIR "/some/path/to/Cplex")
```

for example, in my Ubuntu environment, the absolute path is as follows:

![nano src/CMakeLists.txt](https://github.com/lucasparada20/sbrp_exact/blob/main/images/image.png)

3. The compiler also needs an absolute path to the CVRPSEP package. To provide the path, go into src/CMakeLists.txt and edit the following line:

```cmake
set(CVRPSEP_SRC_DIR "/some/path/to/Cvrpsep")
```

4. The code has two implementations of separation algorithms to separate rounded capacity inequalities: User and CVRPSEP. To build using the User defined algorithms use the following command:

```bash
./cmake_script_exact.sh
```

To build using the CVRPSEP algorithms use the following command:

```bash
./cmake_script_exact.sh release cvrpsep
```

For details of CVRPSEP, refer to the [original paper](https://link.springer.com/article/10.1007/s10107-003-0481-8) from Lysgaard et al., 2003

## Instances

The code can solve two benchmark sets. 

### DHIN instances

This is the set of instances from this [article](https://www.sciencedirect.com/science/article/pii/S0305048313001187) with relatively small sizes in terms of stations: up to 120 stations. To call the instances, use the following sample:

```bash
build/exec_exact instance_type=dhin instance_file=instances_dhin/Boston30.txt
```
Since these instances are relatively small, the code will deploy the ALNS metaheuristic to compute an upper bound and pass a warm start to the exact algorithm. Even for the largest of these instances, the whole ALNS run should not take more than 5 minutes. 

### SLR instances

This is a real-life-sized set built from bike-sharing data in 2023 (details can be found in this [technical report](https://www.cirrelt.ca/documentstravail/cirrelt-2025-02.pdf)). The sizes range from the city of Boston, with 424 stations, to New York, with 2000 stations. An instance in this set has a set of targets and a station status JSON file, both provided in their respective directory. The station status file can be updated anytime; the code will parse the number of stations. To update the station status file, follow these steps

### Updating a station status file

1. Look for the JSON endpoint of the system you want to update data for. The endpoints are provided in this [repo](https://github.com/MobilityData/gbfs/blob/master/systems.csv). 

2. Hit the following curl command:

```bash
curl -s $URL1 -o $FILE1
```

URL1 is the endpoint of the bike-sharing system, and FILE1 is the newly created file with your updated data. For example, for Montreal's Bixi, the following will create a new file with the real and current station status:

```bash
curl -s https://gbfs.velobixi.com/gbfs/en/station_status.json -o montreal_station_status.json
```

3. Replace the old status file in the directory of this repo with the newly created one and call the previously built executable as follows:

```bash
build/exec_exact instance_type=slr instance_file=instances_slr/montreal801.txt targets_file_name=instances_slr/targets/targets_montreal.txt initial_capacities_file_name=instances_slr/status/montreal_station_status.json delta=1
```

I will later create a repo with fun commands and data science techniques for analyzing and scrubbing bike sharing system data. But for now, the `jq` command can tell you the timestamp of the update, the number of stations in your new Bixi file, inspect any given station (say station 20), and count the current number of regular and e-bikes in the system:

**Timestamp**
```bash
date -d @"$(jq '.last_updated' montreal_station_status.json)"
```
**Number of stations**
```bash
jq '.data.stations | length' montreal_station_status.json
```
**Inspect station 20**
```bash
jq '.data.stations[20]' montreal_station_status.json
```
**Count the number of regular and e-bikes**
```bash
jq '
  reduce .data.stations[] as $station ({"ebikes": 0, "bikes": 0};
    {
      "ebikes": (.ebikes + ($station.num_ebikes_available // 0)),
      "bikes": (.bikes + ($station.num_bikes_available // 0))
    }
  )' montreal_station_status.json
```
![Bixi data](https://github.com/lucasparada20/brp_exact/blob/main/images/Bixi%20data.jpg)

### Updating a station information file

The station information file is another endpoint commonly used by BSS. It contains snapshots of the stations locations and capacities in terms of docks, among other relevant data.

1. Look for the JSON endpoint of the system you want to update data for. The endpoints are provided in this [repo](https://github.com/MobilityData/gbfs/blob/master/systems.csv). 

2. Hit the same curl command as for the station status file. For example, for Québec's àVillo system:

```bash
curl -s https://quebec.publicbikesystem.net/customer/gbfs/v2/en/station_information -o quebec_station_information.json
```

Then, using the `jq` command as before, one can query the file to store the coordinates of all the stations in the system:

```bash
jq -r '.data.stations[] | "\(.lat),\(.lon)"' quebec_station_information.json > station_coords.txt

```


