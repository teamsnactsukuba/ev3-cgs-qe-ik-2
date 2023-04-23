# ev3-cgs-qe-ik-2
An inverse kinematics solver with path and trajectory planning based on the CGS-QE algorithm for an EV3 manipulator

This repository contains an implementation of an inverse kinematics solver with path and trajectory planning based on the CGS-QE algorithm for an EV3 manipulator, described in the following paper:

## Contents

The contents of this repository are as follows:

## Prerequisites

For executing the implementation here, you need the following:

- OpenXM infrastructure for communicating mathematical software systems: http://www.openxm.org/
    - Risa/Asir, a computer algebra system: http://www.math.kobe-u.ac.jp/Asir/ (included in the OpenXM distribution and installed automatically with OpenXM under default settings)
- CGS: a program for computing comprehensive Groebner systems in a polynomial ring by Prof. Katsusuke Nabeshima: https://www.rs.tus.ac.jp/~nabeshima/softwares.html
- X Window System server (for executing OpenXM)