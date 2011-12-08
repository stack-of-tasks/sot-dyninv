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

#ifndef __sot_dyninv_ColPivQRSolveInPlace_H__
#define __sot_dyninv_ColPivQRSolveInPlace_H__

#include <Eigen/QR>
#include <iostream>

namespace Eigen
{

  /* This class add two methods to the classical ColPiv: solveInPlace for full
   * col rank matrices (and the direct consequence of pseudoInverse for
   * f-c-r). And solveTranposeInPlace for full row rank matrices (and the
   * direct pseudoInverseTranspose).
   */
  class ColPivQRSolveInPlace
    : public ColPivHouseholderQR< MatrixXd >
  {
  public:

    /* Find a solution x to the problem G x = b. The input vector is b,
     * completed by the 0 tail to obtain the x size.  The solver only works for
     * full-col rank matrices, ie matrices G = Q [ R; 0 ], with R full-rank
     * (full diag) upper triangular. */
    template< typename D >
    void solveInPlace( MatrixBase<D>& Gp )
    {
      const int r = rank(), n=rows();
      assert( r==cols() );
      assert( Gp.rows() == cols() ); // TODO: if not proper size, resize.

      VectorXd workspace( Gp.rows() ); // Size of Gtp number of rows.
      /* P2*P1*P0 ... */
      for (int k = r-1; k >= 0 ; --k)
	{
	  int remainingSize = n-k;
	  Gp.rightCols( Gp.cols()-k )
	    .applyHouseholderOnTheRight(matrixQR().col(k).tail(remainingSize-1),
					hCoeffs().coeff(k), &workspace.coeffRef(0));
	}

      matrixQR()
	.topLeftCorner(r,r).triangularView<Upper>()
	.solveInPlace(Gp);

      Gp.applyOnTheLeft( colsPermutation() );
    }

    MatrixXd pseudoInverse( void )
    {
      MatrixXd res = MatrixXd::Identity( cols(),rows() );
      solveInPlace( res );
      return res;
    }



    /* Find a solution x to the problem G'x = b. The input vector is b,
     * completed by the 0 tail to obtain the x size.  The solver only works for
     * full-row rank matrices, ie matrices G' = [ L 0 ] Q, with L full-rank
     * (full diag) lower triangular. */
    template< typename D >
    void solveTransposeInPlace( MatrixBase<D>& Gtp )
    {
      /* r is the rank, nxm the size of the original matrix (ie whose transpose
       * has been decomposed. n is the number of cols of the original matrix,
       * thus number of rows of the transpose we want to inverse. */
      const int r = rank(), m=rows();
      assert( r==cols() );
      assert( Gtp.rows() == m ); // TODO: if not proper size, resize.

      /* G E = Q R  ::  E' G' = L Q', with L=R'
       * => G^+T = Q R^+T E' */

      /* Compute E'*X. */
      Gtp.applyOnTheLeft( colsPermutation().transpose() );

      /* Compute R^+T E'*X. */
      matrixQR()
	.topLeftCorner(r,r).transpose().triangularView<Lower>()
	.solveInPlace(Gtp.topRows(r));

      /* Compute Q R^+T E'*X. */
      /* Q = P1*P2* ... *Pn */
      VectorXd workspace( Gtp.cols() ); // size of Gtp number of cols.
      for (int k = r-1; k >= 0 ; --k)
	{
	  int remainingSize = m-k;
	  /* std::cout << " ---------------------------" << std::endl; */
	  /* std::cout << remainingSize << std::endl; */
	  /* std::cout << " ---------------------------" << std::endl; */
	  /* std::cout << Gtp.bottomRows( Gtp.rows()-k ) << std::endl; */
	  /* std::cout << " ---------------------------" << std::endl; */
	  /* std::cout << matrixQR().col(k).transpose() << std::endl; */
	  /* std::cout << " ---------------------------" << std::endl; */
	  /* std::cout << matrixQR().col(k).tail(remainingSize-1).transpose() << std::endl; */
	  /* std::cout << " ---------------------------" << std::endl; */
	  Gtp.bottomRows( remainingSize )
	    .applyHouseholderOnTheLeft(matrixQR().col(k).tail(remainingSize-1),
				       hCoeffs().coeff(k), &workspace.coeffRef(0));
	}
    }

    MatrixXd pseudoInverseTranspose( void )
    {
      MatrixXd Gtp = MatrixXd::Identity( rows(),cols() );
      solveTransposeInPlace( Gtp );
      return Gtp;
    }

  };
}


#endif // #ifndef __sot_dyninv_ColPivQRSolveInPlace_H__
