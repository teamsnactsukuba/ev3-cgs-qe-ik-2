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
- $F_4=[F_{41},F_{42},F_{43},F_{44},F_{45},F_{46}]$
    - $F_2$ with putting $(x_0,y_0,z_0)=(10,40,80)$ and 
    $(x_f,y_f,z_f)=(40,100,20)$ as the initial and the goal positions of the manipulator
    - $c_1,s_1,c_4,s_4,c_7,s_7$: variables
    - $s$: parameters

## An algorithm for inverse kinematic computation

### Notation

* $F$: A generator of polynomial ideal
* Param: Parameters $x,y,z$ (the coordinates of the end-effector)
* Var: Variables $c_1, s_1, c_4, s_4, c_7, s_7$
* Ord: An order of variables used for Gröbner basis computation
* Coord: The values of parameters $x,y,z$

### Main files and functions

#### [IKP_Point.rr](./IKP_Point.rr): An algorithm for Solving inverse kinematic problem (Algorithm 1)

* ikp_point(CGS, Param, Var, Ord, Coordinate): A solver for inverse kinematic problem
    * generate_realcgs(CGS, Param, Ord): Extract non-empty segments over the real number
    * cgs_choose(CGS, Coordinate): (Algorithm 2) Choose the Groenber basis in the corresponding segment for the given coordinate
    * newmainqe(Set, Vari: Verification of the existence of a real root
    * varisolve(Set), main_varisolve(Poly, Kai): Solving the inverse kinematic equation
        * zeroset_subst(Set, Coordinate): A zero verification of the affine variety
        * nonzeroset_subst(Set, Coordinate): A nonzero verification of the affine variety
        * eigen_num(CPoly): Counting ((the number of positive eigenvalues) - (the number of negative eigenvalues))
        * nonzeroideal_ikp(G, Var): (Section 4.3) Process for the case when the ideal is not zero dimensional

#### [Segment.rr](./Segment.rr): An algorithm for eliminating segment(s) which does not have real affine varieties (Step 2)

* segment(ZeroSet, NonZeroSet, Param, Ord): Eliminate segment(s) which are not real affine varieties
    * first(ZeroSet, NonzeroSet, Var): Eliminate a segment that does not have a real affine variety by detecting parameters = 0 contained in the affine variety (in the case degree of the term is even)
    * second(Set): In the case ZeroSet contains a univariate polynomial of degree 2
    * third(Set): In the case ZeroSet contains a univariate polynomial of degree > 2
    * union(A,B): A union B
    * memberof(Element, Set): Test if the Element is a member of the Set
    * iseven(Number): Test if the number is even
    * even_degree_term(Term): Test if the monomial has even degree w.r.t. all the variables
    * even_degree_poly(Set, Vari): Extract polynomials consisting of the terms of even degrees
    * signs(Set, Var): Computing signs of the polynomails in Set w.r.t. Var
    * elementeq(Set): Test if all the elements in Set are equal
    * extract_zero_vars(Set, Var): Extract variables to be 0
    * zero_subst(Set, Vars): Substitute zeros to variables in polynomials
    * two_varideg(Poly): Test if Poly is a univariate polynomial of degree 2
    * discriminant(Poly): Computing the discriminant of quadratic polynomial
    * coefset(Poly): Computing the list of the coefficients of Poly
    * max(Set): Extract the maximum value in Set
    * signs_change(Set): Counting the number of sign changes of Set
    * sturm_generate(Poly): Generating the Sturm sequence
    * root_bound(Poly): Computing a bound on all the roots of Poly (Cauchy's bound)
    * sign_limit(Poly): Signs of the polynomial at [-Infinity, +Infinity]
    * three_varideg(Poly): Test if Poly is a univariate polynomial of degree >= 3
    * sturm(Poly): Real root counting with the Sturm's method



## An algorithm for path and trajectory planning

An algorithm for moving the end-effector from the initial position to the final position on a straight path.

### Definitions

* $p_d = (x, y, z)$: The coordinates of the end-effector
* $p_0 = (x_0, y_0, z_0)$: The initial position of the end-effector
* $p_f = (x_f, y_f, z_f)$: The final position of the end-effector
* $p_d = p_0(1-s) + p_f s$, $0 \le s \le 1$
    * $s=0$: the initial position 
    * $s=1$: the final posiiton

### A method for computing path and trajectory planning

1. For time $t$, express the parameter $s$ by a function of 
$t$.
1. For the parameter $s$, the initial position $p_0$, 
the final position $p_f$, calculate current position 
$p_d(x_d,y_d,z_d)$.
1. For $p_d(x_d,y_d,z_d)$, solve the inverse kinematic computation for computing the positions of joints 
$\theta_1$, $\theta_4$ and $\theta_7$ through
$c_1,s_1,c_4,s_4,c_7,s_7$.
If $c_1,s_1,c_4,s_4,c_7,s_7$ do not exist, report that 
the manipulator is not in feasible position.

### An algorithm for computing trajectory

1. For $t=0$, with $x_0,y_0,z_0$ solve the inverse kinematic problem.
1. Repeat solving the inverse kinematic problem for $t=1,\dots,T$ and output a series of the joint angles $\theta_1,\theta_4,\theta_7$.
1. If $c_1,s_1,c_4,s_4,c_7,s_7$ do not exist for a specific $s$, output the trajectory up to that $s$.

### An algorithm for path planning with the CGS-QE method

1. Put the coordinates of $p_d$ into the system of polynomial equations.
1. Put the coordinates of $p_0$ and $p_f$ into the 
system derived in the previous step. Now the system has
a parameter $s$.
1. Use MainQE algorithm in the CGS-QE method for verifying the system has a real root for 
$s\in[0,1]$.
1. If the above condition is satisfied, call the algorithm for computing trajectory.

### An implementation

#### [IKP_Trajectory.rr](./IKP_Trajectory.rr): An algorithm for path and trajectory planning
* ikp_trajectory(CGS, Parm, Var, Ord, P0, Pf, S, T): main function for path and trajectory planning 
  * coodenate_subst(P0, Pf, S): Calculate current position of the end-effector $p_d$ from $p_0,p_f,s$ 

#### [MainQE.rr](./MainQE.rr): an implementation of the MainQE algorithm in the CGS-QE method
* mainqe(CGS, Var, Param): MainQE algorithm

## Data files

### System of equations

* equation-1.rr: definition of $F_1$
* equation-2.rr: definition of $F_2$
* equation-3.rr: definition of $F_3$
* equation-4.rr: definition of $F_4$

### CGS

* F1_CGS.dat: The CGS of $\langle F_1\rangle$
* F1_CGS_real.dat: The CGS of $\langle F_1\rangle$ for only the segment with real affine varieties
* F3_CGS.dat: The CGS of $\langle F_3\rangle$
* F4_CGS.dat: The CGS of $\langle F_4\rangle$

## Scripts

* gen-F1_CGS.rr: a script for computing CGS of $\langle F_1\rangle$ and save the result as F1_CGS.rr
* gen-F1_CGS-nosave.rr a script for computing CGS of $\langle F_1\rangle$
without saving the result (for benchmark purpose)
* gen-F1_CGS_real.rr: a script for computing CGS of $\langle F_1\rangle$,
then choosing segments that exist in the real vector space and save the result as gen-F1_CGS_real.rr
* gen-F1_CGS_real-nosave.rr: a script for computing CGS of $\langle F_1\rangle$,
then choosing segments that exist in the real vector space without saving the result (for benchmark purpose)
* gen-F3_CGS.rr: a script for computing CGS of $\langle F_3\rangle$ and save the result as F3_CGS.rr
* gen-F4_CGS.rr: a script for computing CGS of $\langle F_3\rangle$ and save the result as F4_CGS.rr

## Subsidiary files

### Test.rr: Programs used in the tests