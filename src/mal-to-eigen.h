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
}

#define EIGEN_MATRIX_FROM_SIGNAL(name,signal)	         \
  Eigen::Map<Eigen::MatrixRXd> name			 \
  (							 \
   const_cast<double*>(signal.accessToMotherLib().data().begin()),	\
   signal.nbRows(),					 \
   signal.nbCols()					 \
							 )
#define EIGEN_VECTOR_FROM_SIGNAL(name,signal)	         \
  Eigen::Map<Eigen::VectorXd> name			 \
  (							 \
   signal.accessToMotherLib().data().begin(),		 \
   signal.size()					 \
							 )

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



#define EIGEN_MATRIX_FROM_MATRIX(eigName,mlName,r,c)	         \
  mlName.resize(r,c);						 \
  EIGEN_MATRIX_FROM_SIGNAL(eigName,mlName)

#define EIGEN_VECTOR_FROM_VECTOR(eigName,mlName,r)	         \
  mlName.resize(r);						 \
  EIGEN_VECTOR_FROM_SIGNAL(eigName,mlName)



#endif // __sot_dyninv_mal_to_eigen_H__
