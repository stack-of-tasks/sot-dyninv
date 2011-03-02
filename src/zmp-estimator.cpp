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
	,CONSTRUCT_SIGNAL_IN(fn,ml::Matrix)
	,CONSTRUCT_SIGNAL_IN(support,ml::Vector)
	,CONSTRUCT_SIGNAL_OUT(zmp,ml::Vector, fnSIN << supportSIN)
      {
	signalRegistration( fnSIN << supportSIN << zmpSOUT );
      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      ml::Vector& ZmpEstimator::
      zmpSOUT_function( ml::Vector &zmp, int iter )
      {
	sotDEBUG(15) << " # In time = " << iter << std::endl;

	const ml::Vector& mlfn = fnSIN(iter);
	const ml::Matrix& support = supportSIN(iter);
	zmp.resize(2);
	ml::Vector mlzmp;
	mlzmp.resize(2);
	double Sfn=0.;
	for(int i=0;i<4;i++)
	  {
	    mlzmp(0) += mlfn(i)*support(0,i);
	    mlzmp(1) += mlfn(i)*support(1,i);
	    Sfn += mlfn(i);
	  }
	zmp(0) = mlzmp(0)/Sfn;
	zmp(1) = mlzmp(1)/Sfn;

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

      void ZmpEstimator::
      commandLine( const std::string& cmdLine,
		   std::istringstream& cmdArgs,
		   std::ostream& os )
      {
	if( cmdLine == "help" )
	  {
	    os << "ZmpEstimator:\n"
	       << "\t- ." << std::endl;
	    Entity::commandLine( cmdLine,cmdArgs,os );
	  }
	else
	  {
	    Entity::commandLine( cmdLine,cmdArgs,os );
	  }
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

