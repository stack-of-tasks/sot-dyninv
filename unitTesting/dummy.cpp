/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
 *
 * This file is part of sot-dyninv.
 * sot-dyninv is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dyninv is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dyninvc.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Eigen/QR>
#include <Eigen/SVD>
#include <iostream>


int main (void)
{
  using namespace Eigen;
  using namespace std;
  const unsigned int n=36, m=24, p=12;
  MatrixXd Xhi = MatrixXd::Random(n,p);
  MatrixXd R = MatrixXd::Random(p,m);
  MatrixXd M = Xhi*R;




  VectorXd y = VectorXd::Random(n);
  cout << "Here is the matrix m:" << endl << M << endl;
  cout << "Here is the matrix y:" << endl << y << endl;

  VectorXd x;
  FullPivHouseholderQR<MatrixXd> Mqr;
  Mqr.compute(M);
  x = Mqr.solve(y);
  //assert(y.isApprox(M*x));
  cout << "Here is a solution x to the equation mx=y:" << endl << x << endl;



  JacobiSVD<MatrixXd> Msvd(M, ComputeFullV|ComputeThinU);
  cout << "sigma = " << Msvd.singularValues() << endl;

  // Rank revealing
  const int rank = (Msvd.singularValues().array() > 1e-5 ).count();
  cout << "Rank is " << rank << endl;

  x =
    Msvd.matrixU().leftCols(rank).transpose()*y;
  cout << "Uty = " << x << endl;
  x =
    Msvd.singularValues().array().head(rank).inverse().matrix().asDiagonal() *
    Msvd.matrixU().leftCols(rank).transpose()*y;
  cout << "SiUty = " << x << endl;
  x =
    Msvd.matrixV().leftCols(rank) *
    Msvd.singularValues().array().head(rank).inverse().matrix().asDiagonal() *
    Msvd.matrixU().leftCols(rank).transpose()*y;
  cout << "VSiUty = " << x << endl;



  return 0;
}
