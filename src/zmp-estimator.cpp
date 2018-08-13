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


#include <sot-dyninv/zmp-estimator.h>
#include <sot-dyninv/commands-helper.h>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {

      namespace dg = ::dynamicgraph;
      using namespace dg;
      using dg::SignalBase;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ZmpEstimator,"ZmpEstimator");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      ZmpEstimator::
      ZmpEstimator( const std::string & name )
	: Entity(name)
	,CONSTRUCT_SIGNAL_IN(fn,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(support,dg::Matrix)
	,CONSTRUCT_SIGNAL_OUT(zmp,dg::Vector, fnSIN << supportSIN)
      {
	signalRegistration( fnSIN << supportSIN << zmpSOUT );
      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      dg::Vector& ZmpEstimator::
      zmpSOUT_function( dg::Vector &zmp, int iter )
      {
	sotDEBUG(15) << " # In time = " << iter << std::endl;

	const dg::Vector& mlfn = fnSIN(iter);
	const dg::Matrix& support = supportSIN(iter);
	zmp.resize(2);;
	double numx=0., numy=0.;
	double Sfn=0.;
	for(int i=0;i<mlfn.size();i++)
	  {
	    numx += mlfn(i)*support(0,i);
	    numy += mlfn(i)*support(1,i);
	    Sfn += mlfn(i);
	  }
	zmp(0) = numx/Sfn;
	zmp(1) = numy/Sfn;

	sotDEBUG(1) << "zmp = " << zmp << std::endl;
	return zmp;
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */


      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */

      void ZmpEstimator::
      display( std::ostream& os ) const
      {
	os << "ZmpEstimator "<<getName() << "." << std::endl;
      }
    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

