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
 * with sot-dyninv.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __sot_dyninv_mal_to_eigen_H__
#define __sot_dyninv_mal_to_eigen_H__

#include <Eigen/LU>
#include <soth/Algebra.hpp>

namespace Eigen
{
  typedef Matrix<double,Dynamic,Dynamic,RowMajor> MatrixRXd;
  typedef Map<MatrixRXd> SigMatrixXd;
  typedef Map<VectorXd> SigVectorXd;
  typedef const Map<const MatrixRXd> const_SigMatrixXd;
  typedef const Map<const VectorXd> const_SigVectorXd;
}

#define EIGEN_CONST_MATRIX_FROM_SIGNAL(name,signal)	                 \
  Eigen::const_SigMatrixXd name						\
  (							                 \
   signal.accessToMotherLib().data().begin(),                      	 \
   signal.nbRows(),				               	         \
   signal.nbCols()				               	         \
						               	         )
#define EIGEN_MATRIX_FROM_SIGNAL(name,signal)	                         \
  Eigen::SigMatrixXd name			                         \
  (							                 \
   signal.accessToMotherLib().data().begin(),	                         \
   signal.nbRows(),				               	         \
   signal.nbCols()				               	         \
						               	         )
#define EIGEN_CONST_VECTOR_FROM_SIGNAL(name,signal)	                 \
  Eigen::const_SigVectorXd name		               	                 \
  (						               	         \
   signal.accessToMotherLib().data().begin(),	               	         \
   signal.size()				               	         \
						               	         )
#define EIGEN_VECTOR_FROM_SIGNAL(name,signal)	                         \
  Eigen::SigVectorXd name	 	               	                 \
  (						               	         \
   signal.accessToMotherLib().data().begin(),	               	         \
   signal.size()				               	         \
						               	         )


namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {

      template< typename D >
	inline void EIGEN_VECTOR_TO_VECTOR( const Eigen::MatrixBase<D>& in, ml::Vector & out )
	{
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY( Eigen::MatrixBase<D> );
	  out.resize(in.size());
	  memcpy( out.accessToMotherLib().data().begin(),in.derived().data(),
		  in.size()*sizeof(double));
	}

      template< typename MB >
	inline void EIGEN_ROWMAJOR_MATRIX_TO_MATRIX( const Eigen::MatrixBase<MB>& in,
						     ml::Matrix & out )
	{
	  out.resize( in.rows(),in.cols() );
	  memcpy( out.accessToMotherLib().data().begin(),in.derived().data(),
		  in.cols()*in.rows()*sizeof(double));
	}

      template< typename MB >
	inline void EIGEN_COLMAJOR_MATRIX_TO_MATRIX( const Eigen::MatrixBase<MB>& in,
						     ml::Matrix & out )
	{
	  // TODO: find a better way for that!
	  out.resize( in.rows(),in.cols() );
	  for( int i=0;i<in.rows();++i )
	    for( int j=0;j<in.cols();++j )
	      out(i,j)=in(i,j);
	}


#ifdef __SOT_MultiBound_H__
      inline void COPY_MB_VECTOR_TO_EIGEN( const VectorMultiBound& ddx,
					   soth::VectorBound& btask1 )
      {
	const int nx1 = ddx.size();
	for( int c=0;c<nx1;++c )
	  {
	    if( ddx[c].getMode() == MultiBound::MODE_SINGLE )
	      btask1[c] = ddx[c].getSingleBound();
	    else
	      {
		const bool binf = ddx[c].getDoubleBoundSetup( dg::sot::MultiBound::BOUND_INF );
		const bool bsup = ddx[c].getDoubleBoundSetup( dg::sot::MultiBound::BOUND_SUP );
		if( binf&&bsup )
		  {
		    const double xi = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_INF);
		    const double xs = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_SUP);
		    btask1[c] = std::make_pair( xi, xs );
		  }
		else if( binf )
		  {
		    const double xi = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_INF);
		    btask1[c] = soth::Bound( xi, soth::Bound::BOUND_INF );
		  }
		else
		  {
		    assert( bsup );
		    const double xs = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_SUP);
		    btask1[c] = soth::Bound( xs, soth::Bound::BOUND_SUP );
		  }
	      }
	  }
      }
#endif // #ifdef __SOT_MultiBound_H__

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph



#define EIGEN_MATRIX_FROM_MATRIX(eigName,mlName,r,c)	         \
  mlName.resize(r,c);						 \
  EIGEN_MATRIX_FROM_SIGNAL(eigName,mlName)

#define EIGEN_VECTOR_FROM_VECTOR(eigName,mlName,r)	         \
  mlName.resize(r);						 \
  EIGEN_VECTOR_FROM_SIGNAL(eigName,mlName)



#endif // __sot_dyninv_mal_to_eigen_H__
