# Test scripts
This directory contains test scripts.

## A test for inverse kinematic computation (Section 4)

### Main test scripts

- experiment-IKP_point-old.rr: a test script of inverse kinematic computation in Section 4.4 of the paper
- experiment-IKP_point-old-script.rr: a driver script

Usage in Risa/Asir
```
fep asir
load("experiment-IKP_point-old-script.rr")$
```
The log can be found at [../log/IA-64/experiment-IKP_point-old.log](../log/IA-64/experiment-IKP_point-old.log).

### Scripts for generating test sets

Generated data files are located in [../data/](../data/)

- gen-Old-Coordinate-01.rr
- gen-Old-Coordinate-02.rr
- gen-Old-Coordinate-03.rr
- gen-Old-Coordinate-04.rr
- gen-Old-Coordinate-05.rr
- gen-Old-Coordinate-06.rr
- gen-Old-Coordinate-07.rr
- gen-Old-Coordinate-08.rr
- gen-Old-Coordinate-09.rr
- gen-Old-Coordinate-10.rr

## A test for trajectory planning (Section 5.1)

- test_IKP_trajectory.rr: generating a trajectory
    - test_IKP_trajectory_data.rr: generated trajectory
    - test_IKP_trajectory_data.m: generated trajectory (converted to Mathematica list format)

Usage in Risa/Asir
```
fep asir
load("test_IKP_trajectory.rr")$
```
A sample log can be found at [../log/IA-64/test_IKP_trajectory.log](../log/IA-64/test_IKP_trajectory.log).

- test_IKP_trajectory-real.rr: generating a trajectory using a CGS whose segments are contained in the real number field

Usage in Risa/Asir
```
fep asir
load("test_IKP_trajectory-real.rr")$
```
A sample log can be found at [../log/IA-64/test_IKP_trajectory-real.log](../log/IA-64/test_IKP_trajectory-real.log).