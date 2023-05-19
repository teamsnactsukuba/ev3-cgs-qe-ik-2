# ev3-cgs-qe-ik-2
An inverse kinematics solver with path and trajectory planning based on the CGS-QE algorithm for an EV3 manipulator

This repository contains an implementation of an inverse kinematics solver with path and trajectory planning based on the CGS-QE algorithm for an EV3 manipulator, described in the following paper:

M. Yoshizawa, A. Terui, M. Mikawa. <br />
Inverse kinematics and path planning of manipulator using CGS-QE.
preprint, to be made public.

## Contents

The contents of this repository are as follows:

- src: Source code of the implementation
    - data: data files of test data
    - log: log files of the tests
    - test: test scripts

## Prerequisites

For executing the implementation here, you need the following:

- OpenXM infrastructure for communicating mathematical software systems: http://www.openxm.org/
    - Risa/Asir, a computer algebra system: http://www.math.kobe-u.ac.jp/Asir/ (included in the OpenXM distribution and installed automatically with OpenXM under default settings)
- CGS: a program for computing comprehensive Groebner systems in a polynomial ring by Prof. Katsusuke Nabeshima: https://www.rs.tus.ac.jp/~nabeshima/softwares.html
- X Window System server (for executing OpenXM)

Path planning requires additional conditions

- Wolfram Mathematica or Wolfram Engine (Version 13.1 has been tested):
- Run the above software on Linux.