/*
 * Copyright 2011,
 * Nicolas Mansard
 * LAAS-CNRS
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

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
//#define VP_DEBUG
//#define VP_DEBUG_MODE 45

#include <sot-dyninv/feature-projected-line.h>

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot/core/debug.hh>
#include <sot/core/exception-feature.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-utheta.hh>

#include <sot/core/factory.hh>

namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureProjectedLine,"FeatureProjectedLine");

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */


      FeatureProjectedLine::
      FeatureProjectedLine( const std::string& pointName )
	: FeatureAbstract( pointName )

	,CONSTRUCT_SIGNAL_IN(xa,MatrixHomogeneous)
	,CONSTRUCT_SIGNAL_IN(xb,MatrixHomogeneous)
	,CONSTRUCT_SIGNAL_IN(Ja,ml::Matrix)
	,CONSTRUCT_SIGNAL_IN(Jb,ml::Matrix)
	,CONSTRUCT_SIGNAL_IN(xc,ml::Vector)
      {
	jacobianSOUT.addDependency( xaSIN );
	jacobianSOUT.addDependency( xbSIN );
	jacobianSOUT.addDependency( xcSIN );
	jacobianSOUT.addDependency( JaSIN );
	jacobianSOUT.addDependency( JbSIN );

	errorSOUT.addDependency( xaSIN );
	errorSOUT.addDependency( xbSIN );
	errorSOUT.addDependency( xcSIN );

	activationSOUT.removeDependency( desiredValueSIN );

	signalRegistration( xaSIN << xbSIN << xcSIN << JaSIN << JbSIN );
      }


      /* --------------------------------------------------------------------- */
      /* --------------------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      unsigned int& FeatureProjectedLine::
      getDimension( unsigned int & dim, int  )
      {
	dim = 2;
	return dim;
      }

      /** Compute the interaction matrix from a subset of
       * the possible features.
       */
      ml::Matrix& FeatureProjectedLine::
      computeJacobian( ml::Matrix& J,int time )
      {
	sotDEBUGIN(15);

	const MatrixHomogeneous & A = xaSIN(time), & B = xbSIN(time);
	const ml::Vector & C = xcSIN(time);
	const double
	  xa=A(0,3),xb=B(0,3),xc=C(0),
	  ya=A(1,3),yb=B(1,3),yc=C(1);

	const ml::Matrix & JA = JaSIN(time), & JB = JbSIN(time);

	const int nq=JA.nbCols();
	assert((int)JB.nbCols()==nq);
	J.resize(1,nq);
	for( int i=0;i<nq;++i )
	  {
	    const double
	      & dxa=JA(0,i),& dxb=JB(0,i),
	      & dya=JA(1,i),& dyb=JB(1,i);
	    J(0,i) = dxa*(yb-yc) - dxb*(ya-yc) - dya*(xb-xc) + dyb*(xa-xc);
	  }

	sotDEBUGOUT(15);
	return J;
      }

      /** Compute the error between two visual features from a subset
       * a the possible features.
       */
      ml::Vector&
      FeatureProjectedLine::computeError( ml::Vector& error,int time )
      {
	sotDEBUGIN(15);

	const MatrixHomogeneous & A = xaSIN(time),& B = xbSIN(time);
	const ml::Vector & C = xcSIN(time);
	const double
	  xa=A(0,3),xb=B(0,3),xc=C(0),
	  ya=A(1,3),yb=B(1,3),yc=C(1);

	error.resize(1);
	error(0) = (xb-xa)*(yc-ya)-(yb-ya)*(xc-xa);

	sotDEBUGOUT(15);
	return error ;
      }

      void FeatureProjectedLine::
      display( std::ostream& os ) const
      {
	os <<"ProjectedLine <"<<name<<">" ;
      }


    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
