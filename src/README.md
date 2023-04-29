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

* Segment.rr : 実数体上に存在しない分割部の削除 (手順 2)
    * segment(ZeroSet, NonZeroSet, Param, Ord): 実数体上に存在しない分割部の削除
    * first(ZeroSet, NonzeroSet, Vari): 分割部のうち0とする部分からパラメータの値が明らかに求まる場合(変数の次数が偶数の場合)
    * second(Set): In the case ZeroSet contains a univariate polynomial of degree 2
    * third(Set): In the case ZeroSet contains a univariate polynomial of degree > 2
    * union(A,B): A union B
    * memberof(Element, Set): Test if the Element is a member of the Set
    * iseven(Number): Test if the number is even
    * even_degree_term(Term): Test if the monomial has even degree w.r.t. all the variables
    * even_degree_poly(Set, Vari): Extract polynomials consisting of the terms of even degrees
    * signs(Set, Var): 符号の計算
    * elementeq(Set): Set の要素が等しいかのチェック
    * zerovars(Set, Var): 0になる変数の抽出
    * zero_subst(Set, Vars): 変数に0を代入
    * two_varideg(Poly): 1変数2次方程式を抽出
    * discriminant(Poly): Computing the discriminant of quadratic polynomial
    * coefset(Poly): 係数の集合の構成
    * max(Set): 集合の最大値を抽出
    * signs_change(Set): 符号の変化の数を数える
    * sturm_generate(Poly): Generating the Sturm sequence
    * genkai(Poly): 根の限界の計算
    * sign_limit(Poly): 正負の無限大における多項式の符号
    * three_varideg(Poly): 1変数の3次以上の多項式の抽出
    * sturm(Poly): Real root counting with the Sturm's method

## Data files

## Subsidiary files
- 