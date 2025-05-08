# brp_exact
An exact algorithm for the bicycle repositioning problem

## Building the code in Linux

1. Clone the repository and add executable permission to a script that will call CMake for you:

```shell
git clone https://github.com/lucasparada20/sbrp_exact.git
cd sbrp
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

4. The code has 2 implementations of separation algorithms to separate rounded capacity inequalities: User and CVRPSEP. To use the User defined algorithms use:

```bash
./cmake_script_exact.sh
```

To use CVRPSEP:

```bash
./cmake_script_exact.sh release cvrpsep
```

For details of CVRPSEP, refer to the [original paper](https://link.springer.com/article/10.1007/s10107-003-0481-8) from Lysgaard et al., 2003

