# The inverse kinematic solver

## Definitions

- $F_1=[F_{11},F_{12},F_{13},F_{14},F_{15},F_{16}]$
    - the original system for the inverse kinematic problem
    - $c_1, s_1, c_4, s_4, c_7, s_7$: variables representing $c_i=\cos(\theta_i)$, $s_i=\sin(\theta_i)$
    - $(x,y,z)$: parameters representing the given position of the end-effector
- $F_2=[F_{21},F_{22},F_{23},F_{24},F_{25},F_{26}]$
    - $F_1$ with replacing $(x,y,z)$ with a path as $x=x_0(1-s)+x_fs$, $y=y_0(1-s)+y_fs$, $z=z_0(1-s)+z_fs$, $0\le s\le 1$
    - $(x_0,y_0,z_0)$: the initial position of the end-effector
    - $(x_f,y_f,z_f)$: the goal position of the end-effector
    - $c_1,s_1,c_4,s_4,c_7,s_7$: variables
    - $x_0,y_0,z_0,x_f,y_f,z_f,s$: parameters
- $F_3=[F_{31},F_{32},F_{33},F_{34},F_{35},F_{36}]$
    - $F_2$ with putting $(x_0,y_0,z_0)=(50,25,100)$ and 
    $(x_f,y_f,z_f)=(40,100,20)$ as the initial and the goal positions of the manipulator
    - $c_1,s_1,c_4,s_4,c_7,s_7$: variables
    - $s$: parameters

## Algorithm

## Main files

## Data files

## Subsidiary files
- 