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

#include <sot/core/feature-abstract.hh>

#include <sot-dyninv/commands-helper.h>
#include <sot-dyninv/task-dyn-limits.h>

#include <boost/foreach.hpp>


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
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskDynLimits,"TaskDynLimits");

      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      TaskDynLimits::
      TaskDynLimits( const std::string & name )
	: TaskDynPD(name)

	,CONSTRUCT_SIGNAL_IN(position,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(velocity,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(referencePosInf,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(referencePosSup,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(referenceVelInf,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(referenceVelSup,dg::Vector)

	,CONSTRUCT_SIGNAL_OUT(normalizedVelocity,dg::Vector,velocitySIN<<referenceVelInfSIN<<referenceVelSupSIN)
	,CONSTRUCT_SIGNAL_OUT(normalizedPosition,dg::Vector,positionSIN<<referencePosInfSIN<<referencePosSupSIN)

	,previousJ(0u,0u),previousJset(false)
      {
	taskSOUT.setFunction( boost::bind(&TaskDynLimits::computeTaskDynLimits,this,_1,_2) );
	jacobianSOUT.setFunction( boost::bind(&TaskDynLimits::computeTjlJacobian,this,_1,_2) );
	JdotSOUT.setFunction( boost::bind(&TaskDynLimits::computeTjlJdot,this,_1,_2) );
	taskSOUT.clearDependencies();
	taskSOUT.addDependency( referenceVelSupSIN );
	taskSOUT.addDependency( referenceVelInfSIN );
	taskSOUT.addDependency( referencePosSupSIN );
	taskSOUT.addDependency( referencePosInfSIN );
	taskSOUT.addDependency( dtSIN );
	taskSOUT.addDependency( positionSIN );
	taskSOUT.addDependency( velocitySIN );

	signalRegistration( referenceVelSupSIN << referenceVelInfSIN << velocitySIN
			    << referencePosSupSIN << referencePosInfSIN << positionSIN
			    << dtSIN << normalizedVelocitySOUT << normalizedPositionSOUT);

      }

      /* ---------------------------------------------------------------------- */
      /* --- COMPUTATION ------------------------------------------------------ */
      /* ---------------------------------------------------------------------- */

      dg::sot::VectorMultiBound& TaskDynLimits::
      computeTaskDynLimits( dg::sot::VectorMultiBound& res,int time )
      {
	sotDEBUGIN(45);

	const dg::Vector & position = positionSIN(time);
	const dg::Vector & velocity = velocitySIN(time);
	const dg::Vector & refPosInf = referencePosInfSIN(time);
	const dg::Vector & refPosSup = referencePosSupSIN(time);
	const dg::Vector & refVelInf = referenceVelInfSIN(time);
	const dg::Vector & refVelSup = referenceVelSupSIN(time);
	//const double & dt = dtSIN(time);
	const double & gain = controlGainSIN(time);
	const double & dt = dtSIN(time)/gain;

	sotDEBUG(35) << "position = " << position << std::endl;
	sotDEBUG(35) << "velocity = " << velocity << std::endl;

	const double kt=2/(dt*dt);
	double maxVel, maxPos, maxLim, minVel, minPos, minLim;

	res.resize(position.size());
	for( unsigned int i=0;i<res.size();++i )
	  {
	    maxVel = 1/dt*(refVelSup(i)-velocity(i));	// MAXimum acceleration value due to VELocity limits
	    minVel = 1/dt*(refVelInf(i)-velocity(i));	// MINimum acceleration value due to VELocity limits
	    maxPos = kt*(refPosSup(i)-position(i)-dt*velocity(i));	// MAXimum acceleration value due to POSition limits
	    minPos = kt*(refPosInf(i)-position(i)-dt*velocity(i));	// MINimum acceleration value due to POSition limits

	    maxLim = maxVel<maxPos ? maxVel : maxPos;
	    minLim = minVel>minPos ? minVel : minPos;

	    res[i] = dg::sot::MultiBound( minLim,maxLim );
	  }

	sotDEBUG(15) << "taskU = "<< res << std::endl;
	sotDEBUGOUT(45);
	return res;
      }

      dg::Matrix& TaskDynLimits::
      computeTjlJacobian( dg::Matrix& J,int time )
      {
	sotDEBUG(15) << "# In {" << std::endl;

	const dg::Vector& position = positionSIN(time);
	/*
	  if( featureList.empty())
	  { throw( sotExceptionTask(sotExceptionTask::EMPTY_LIST,"Empty feature list") );}

	  try {
	  unsigned int dimJ = J .rows();
	  unsigned int nbc = J.cols();
	  if( 0==dimJ ){ dimJ = 1; J.resize(dimJ,nbc); }
	*/
	J.resize(position.size(),position.size());
	J.setZero();
	for(unsigned int j=6; j<position.size(); ++j)
	  {  J(j,j) = 1; }

	sotDEBUG(15) << "# Out }" << std::endl;
	return J;
      }

      dg::Matrix& TaskDynLimits::
      computeTjlJdot( dg::Matrix& Jdot,int time )
      {
	sotDEBUGIN(15);

	const dg::Vector& velocity = velocitySIN(time);
	Jdot.resize(velocity.size(),velocity.size());
	Jdot.setZero();

	sotDEBUGOUT(15);
	return Jdot;
      }

      dg::Vector& TaskDynLimits::
      normalizedVelocitySOUT_function( dg::Vector& res, int time )
      {
	const dg::Vector & velocity = velocitySIN(time);
	const dg::Vector & refVelInf = referenceVelInfSIN(time);
	const dg::Vector & refVelSup = referenceVelSupSIN(time);

	const unsigned int & n = velocity.size();
	res.resize(n);
	assert( n==refVelInf.size() && n==refVelSup.size() );
	for( unsigned int i=0;i<n;++i )
	  {
	    res(i) = ( velocity(i)-refVelInf(i) ) / ( refVelSup(i)-refVelInf(i) );
	  }
	return res;
      }

      dg::Vector& TaskDynLimits::
      normalizedPositionSOUT_function( dg::Vector& res, int time )
      {
	const dg::Vector & position = positionSIN(time);
	const dg::Vector & refPosInf = referencePosInfSIN(time);
	const dg::Vector & refPosSup = referencePosSupSIN(time);

	const unsigned int & n = position.size();
	res.resize(n);
	assert( n==refPosInf.size() && n==refPosSup.size() );
	for( unsigned int i=0;i<n;++i )
	  {
	    res(i) = ( position(i)-refPosInf(i) ) / ( refPosSup(i)-refPosInf(i) );
	  }
	return res;
      }

      /* ------------------------------------------------------------------------ */
      /* --- DISPLAY ENTITY ----------------------------------------------------- */
      /* ------------------------------------------------------------------------ */

      void TaskDynLimits::
      display( std::ostream& os ) const
      {
	os << "TaskDynLimits " << name << ": " << std::endl;
      }

      /*
      void TaskDynLimits::
      display( std::ostream& os ) const
      {
	os << "TaskDynLimits " << name << ": " << std::endl;
	os << "--- LIST ---  " << std::endl;
	BOOST_FOREACH( dg::sot::FeatureAbstract* feature,featureList )
	  {
	    os << "-> " << feature->getName() << std::endl;
	  }
      }
      */

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

