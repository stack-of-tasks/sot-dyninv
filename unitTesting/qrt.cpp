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
#include <soth/Algebra.hpp>
#include <iostream>

std::ostream& p( const char * n ) { return std::cout << n << " = "; }


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
    void solveInPlace( MatrixXd& Gp )
    {
      const int r = rank(), m=cols(), n=rows();
      assert( r==m );
      assert( Gp.rows() == m ); // TODO: if not proper size, resize.

      VectorXd workspace( n );
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
    void solveTransposeInPlace( MatrixXd& Gtp )
    {
      /* r is the rank, nxm the size of the original matrix (ie whose transpose
       * has been decomposed. n is the number of cols of the original matrix,
       * thus number of rows of the transpose we want to inverse. */
      const int r = rank(), n=cols(), m=rows();
      assert( r==n );
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
      VectorXd workspace( n );
      for (int k = r-1; k >= 0 ; --k)
	{
	  int remainingSize = m-k;
	  Gtp.bottomRows( Gtp.rows()-k )
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

int main (void)
{
  using namespace Eigen;
  using std::endl; using std::cout; using soth::MATLAB;

  { /* Compute the pseudo inverse of a full-col rank matrix: G = V [ R; 0 ]. */

    const unsigned int n=12, m=9, r=9;

    MatrixXd G = MatrixXd::Random(n,m);
    p("G") << (MATLAB)G << endl;

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> G_qr;
    G_qr.compute(G);
    p("QR") << (MATLAB)G_qr.matrixQR() << endl;

    {
      /* Q1 = ... */
      MatrixXd Q1 = MatrixXd::Identity( n,r );
      VectorXd workspace( n );
      /* P0*P1*P2 ... */
      for (int k = r-1; k >= 0 ; --k)
	{
	  int remainingSize = n-k;
	  Q1.bottomRows( Q1.rows()-k )
	    .applyHouseholderOnTheLeft(G_qr.matrixQR().col(k).tail(remainingSize-1),
				       G_qr.hCoeffs().coeff(k), &workspace.coeffRef(0));
	}
      p("Q1") << (MATLAB)Q1 << endl;
    }

    {
      /* Q1' = ... */
      MatrixXd Q1t = MatrixXd::Identity( r,n );
      VectorXd workspace( n );
      /* P2*P1*P0 ... */
      for (int k = r-1; k >= 0 ; --k)
	{
	  int remainingSize = n-k;
	  Q1t.rightCols( Q1t.cols()-k )
	    .applyHouseholderOnTheRight(G_qr.matrixQR().col(k).tail(remainingSize-1),
					G_qr.hCoeffs().coeff(k), &workspace.coeffRef(0));
	}
      p("Q1t") << (MATLAB)Q1t << endl;
    }

    {
      MatrixXd Gp = MatrixXd::Identity( r,n );
      VectorXd workspace( n );
      /* P2*P1*P0 ... */
      for (int k = r-1; k >= 0 ; --k)
	{
	  int remainingSize = n-k;
	  Gp.rightCols( Gp.cols()-k )
	    .applyHouseholderOnTheRight(G_qr.matrixQR().col(k).tail(remainingSize-1),
					G_qr.hCoeffs().coeff(k), &workspace.coeffRef(0));
	}

      G_qr.matrixQR()
	.topLeftCorner(r,r).triangularView<Upper>()
	.solveInPlace(Gp);
      p("EtGp") << (MATLAB)Gp << endl;

      Gp.applyOnTheLeft( G_qr.colsPermutation() );
      p("Gp") << (MATLAB)Gp << endl;
    }

    {
      ColPivQRSolveInPlace G_qr; G_qr.compute(G);
      p("Gp") << (MATLAB)G_qr.pseudoInverse() << endl;
    }


  }

  { /* Compute the pseudo inverse of a full-row rank matrix: G = [ L; 0 ] V'. */
    const unsigned int n=9, m=12, r=9;

    MatrixXd G = MatrixXd::Random(n,m);
    p("G") << (MATLAB)G << endl;

    Eigen::ColPivHouseholderQR< Eigen::MatrixXd > Gt_qr;
    Gt_qr.compute(G.transpose());
    p("QR") << (MATLAB)Gt_qr.matrixQR() << endl;
    p("Q") << (MATLAB)(MatrixXd)Gt_qr.householderQ() << endl;
    p("R") << (MATLAB)(MatrixXd)Gt_qr.matrixQR().triangularView<Upper>() << endl;
    p("E") << (MATLAB)Gt_qr.colsPermutation().toDenseMatrix() << endl;

    {
      MatrixXd Gp = MatrixXd::Identity( m,r );
      Gp.applyOnTheLeft( Gt_qr.colsPermutation().transpose() );
      p("Et") << (MATLAB)Gp << endl;

      Gt_qr.matrixQR()
	.topLeftCorner(r,r).transpose().triangularView<Lower>()
	.solveInPlace(Gp.topRows(r));
      p("RptEt") << (MATLAB)Gp << endl;

      /* Q = P1*P2* ... *Pn */
      VectorXd workspace( n );
      for (int k = r-1; k >= 0 ; --k)
	{
	  cout << k << endl;
	  int remainingSize = m-k;
	  Gp.bottomRows( Gp.rows()-k )
	    .applyHouseholderOnTheLeft(Gt_qr.matrixQR().col(k).tail(remainingSize-1),
				       Gt_qr.hCoeffs().coeff(k), &workspace.coeffRef(0));
	}
      p("Gp") << (MATLAB)Gp << endl;

    }

    {
      ColPivQRSolveInPlace Gt_qr; Gt_qr.compute(G.transpose());
      MatrixXd Gp = MatrixXd::Identity( m,r );
      Gt_qr.solveTransposeInPlace(Gp);
      p("Gp") << (MATLAB)Gp << endl;
      p("Gp") << (MATLAB)Gt_qr.pseudoInverseTranspose() << endl;
    }

  }

  return 0;
}
