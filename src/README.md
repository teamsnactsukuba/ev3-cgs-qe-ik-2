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

## An algorithm for inverse kinematic computation

### Notation

* $F$: A generator of polynomial ideal
* Param: Parameters $x,y,z$ (the coordinates of the end-effector)
* Var: Variables $c_1, s_1, c_4, s_4, c_7, s_7$
* Ord: An order of variables used for Gröbner basis computation
* Coord: The values of parameters $x,y,z$

### Main files and functions

#### IKP_Point.rr: An algorithm for Solving inverse kinematic problem (Steps 2--5)

* ikp_point(CGS, Param, Var, Ord, Coordinate): 逆運動学問題を解くアルゴリズム
    * generate_realcgs(CGS, Para, Jun) : 実数上で空でない分割部のCGSの抽出
    * cgs_choose(CGS, Coordinate) : グレブナー基底の決定
    * newmainqe(Set, Vari) : 実根の存在判定
    * varisolve(Set), main_varisolve(Poly, Kai) : 変数の解の計算
        * zeroset_subst(Set, Coordinate) : 0になる分割部の代入のチェック
        * nonzeroset_subst(Set, Coordinate) : 0にならない分割部の代入のチェック
        * eigen_num(CPoly) : 正の固有値の個数-符の固有値の個数
        * nonzeroideal_ikp(G, Vari) : グレブナー基底が0次元イデアルでないときの処理

#### Segment.rr: An algorithm for eliminating segment(s) which does not have real affine varieties (Step 2)

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

### Data files

* F1_CGS.dat: The CGS of $\langle F_1\rangle$
* F1_CGS_real.dat: The CGS of $\langle F_1\rangle$ for only the segment with real affine varieties
* F3_CGS.dat: The CGS of $\langle F_3\rangle$

### Subsidiary files

#### Test.rr: Programs used in the tests

## An algorithm for path and trajectory planning

ロボットアームの手先が直線的な軌道上を初期位置から目標位置に向かって移動すると仮定する. \

### Definitions

* $p_d = (x, y, z)$: The coordinates of the end-effector
* $p_0 = (x_0, y_0, z_0)$: The initial position of the end-effector
* $p_f = (x_f, y_f, z_f)$: The final position of the end-effector

このとき, パラメータ $0 \le s \le 1$ を用いて, 次のように表す. 

* $p_d = p_0(1-s) + p_f s$

$s$ は初期位置で $s = 0$, 目標位置で $s = 1$ の値になる. 

## 時間$t$の関数を用いた軌道計画の解法
パラメータ $s$ を時間 $t$ の関数で表し, $x_0, y_0, z_0, x_f, y_f, z_f, s, T$ の入力に対して, 変数 $c_1, s_1, c_4, s_4, c_7, s_7$ を求める.  \
あるパラメータ $s$ に対して, $c_1, s_1, c_4, s_4, c_7, s_7$ が存在しないとき, 軌道上を移動できないことを出力する. \
$p_d = (x, y, z)$ に対して次が成り立つ.

  $x = x_0(1-s) + x_fs$ \
  $y = y_0(1-s) + y_fs$ \
  $z = z_0(1-s) + z_fs$  

### アルゴリズムの説明
以下の手順により逆運動学問題を解く. 

1.  $t = 0$ として, $x_0, y_0, z_0, x_f, y_f, z_f, s$ から $x, y, z$ を求め, 逆運動学問題の解法のアルゴリズムを適用する. 
2. $t = 1$ として, (1) と同様に逆運動学問題の解法のアルゴリズムを適用する.
3. これを $t = T$ まで繰り返す. 
4. $t = 0$ から $t = T$ までの解を出力する. 
   もし, ある $s$ に対して, $c_1, s_1, c_4, s_4, c_7, s_7$ が存在しないとき, 軌道 $p_d$ 上をロボットアームのエンドエフェクタが移動不可能なことを出力する. 

### アルゴリズムの実装

* IKP_Orbit_Time : 時間tの関数を用いた軌道計画の解法
  * ikp_orbit_time(CGS, Para, Vari, Jun, P0, Pf, S, T) : 時間tの関数を用いた軌道計画の解法
  * coodenate_subst(P0, Pf, S) : P0, Pf, s から座標(x,y,z)をを求める
* equation2.rr : 多項式イデアル F, パラメータ Para, 変数 Vari, 項順序 Jun, 初期位置 P0, 目標位置 Pf の定義

