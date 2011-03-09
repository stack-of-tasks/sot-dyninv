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


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

//#define VP_DEBUG
//#define VP_DEBUG_MODE 15
#include <sot/core/debug.hh>

#include <dynamic-graph/factory.h>
#include <sot-dyninv/task-joint-limits.h>


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {

      namespace dg = ::dynamicgraph;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskJointLimits,"TaskJointLimits");

      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      TaskJointLimits::
      TaskJointLimits( const std::string & name )
	: TaskAbstract(name)

	,CONSTRUCT_SIGNAL_IN(position,ml::Vector)
	,CONSTRUCT_SIGNAL_IN(referenceInf,ml::Vector)
	,CONSTRUCT_SIGNAL_IN(referenceSup,ml::Vector)
	,CONSTRUCT_SIGNAL_IN(dt,double)
	,CONSTRUCT_SIGNAL_IN(controlGain,double)
	,CONSTRUCT_SIGNAL_IN(selec,Flags)

	,CONSTRUCT_SIGNAL_OUT(normalizedPosition,ml::Vector,
			      positionSIN<<referenceInfSIN<<referenceSupSIN)
	,CONSTRUCT_SIGNAL_OUT(activeSize,int,
			      positionSIN<<selecSIN)
      {
	taskSOUT.setFunction( boost::bind(&TaskJointLimits::computeTask,this,_1,_2) );
	jacobianSOUT.setFunction( boost::bind(&TaskJointLimits::computeJacobian,this,_1,_2) );
	taskSOUT.clearDependencies();
	taskSOUT.addDependency( referenceSupSIN );
	taskSOUT.addDependency( referenceInfSIN );
	taskSOUT.addDependency( dtSIN );
	taskSOUT.addDependency( positionSIN );
	controlGainSIN = 1.0;
	selecSIN = true;
	signalRegistration( referenceSupSIN << referenceInfSIN << dtSIN << selecSIN
			    << activeSizeSOUT
			    << controlGainSIN << positionSIN << normalizedPositionSOUT );
      }

      /* ---------------------------------------------------------------------- */
      /* --- COMPUTATION ------------------------------------------------------ */
      /* ---------------------------------------------------------------------- */
      int& TaskJointLimits::
      activeSizeSOUT_function(int& res, int time)
      {
	const Flags & selec = selecSIN(time);
	const int size = positionSIN(time).size();
	res=0;
	for( int i=0;i<size;++i )
	  {
	    if(selec(i)) res++;
	  }
	return res;
      }


      dg::sot::VectorMultiBound& TaskJointLimits::
      computeTask( dg::sot::VectorMultiBound& res,int time )
      {
	const ml::Vector & position = positionSIN(time);
	const ml::Vector & refInf = referenceInfSIN(time);
	const ml::Vector & refSup = referenceSupSIN(time);
	const Flags & selec = selecSIN(time);
	const double K = 1.0/(dtSIN(time)*controlGainSIN(time));
	const int size = position.size(), activeSize=activeSizeSOUT(time);
	sotDEBUG(35) << "position = " << position << std::endl;

	res.resize(activeSize); int idx=0;
	for( int i=0;i<size;++i )
	  {
	    if( selec(i) )
	      res[idx++] = dg::sot::MultiBound( (refInf(i)-position(i))*K,
						(refSup(i)-position(i))*K );
	  }

	sotDEBUG(15) << "taskU = "<< res << std::endl;
	return res;
      }

      ml::Matrix& TaskJointLimits::
      computeJacobian( ml::Matrix& J,int time )
      {
	const Flags & selec = selecSIN(time);
	const int size = positionSIN(time).size(), activeSize=activeSizeSOUT(time);
	J.resize(activeSize,size);	J.setZero();

	int idx=0;
	for( int i=0;i<size;++i )
	  {
	    if( selec(i) ) J(idx++,i) = 1;
	  }

	return J;
      }

      ml::Vector& TaskJointLimits::
      normalizedPositionSOUT_function( ml::Vector& res, int time )
      {
	const ml::Vector & position = positionSIN(time);
	const ml::Vector & refInf = referenceInfSIN(time);
	const ml::Vector & refSup = referenceSupSIN(time);
	const Flags & selec = selecSIN(time);
	const int size = position.size(), activeSize=activeSizeSOUT(time);
	assert( size==(int)refInf.size() && size==(int)refSup.size() );

	res.resize( activeSize ); int idx=0;
	for( int i=0;i<size;++i )
	  { if(selec(i))
	      res(idx++) = ( position(i)-refInf(i) ) / ( refSup(i)-refInf(i) ); }
	return res;
      }

      /* ------------------------------------------------------------------------ */
      /* --- DISPLAY ENTITY ----------------------------------------------------- */
      /* ------------------------------------------------------------------------ */

      void TaskJointLimits::
      display( std::ostream& os ) const
      {
	os << "TaskJointLimits " << name << ": " << std::endl;
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

