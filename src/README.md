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

### Notation

* $F$: A generator of polynomial ideal
* Param: Parameters $x,y,z$ (the coordinates of the end-effector)
* Var: Variables $c_1, s_1, c_4, s_4, c_7, s_7$
* Ord: An order of variables used for Gröbner basis computation
* Coord: The values of parameters $x,y,z$

## Main files

* Segment.rr: eliminate segment(s) which does not have real affine varieties (Step 2)
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

## Data files

## Subsidiary files
- 