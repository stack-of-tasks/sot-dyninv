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

#include <pinocchio/fwd.hpp>
#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>
#include <sot/core/feature-abstract.hh>

#include <sot-dyninv/task-dyn-pd.h>
#include <sot-dyninv/commands-helper.h>

#include <boost/foreach.hpp>

namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {

      namespace dg = ::dynamicgraph;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskDynPD,"TaskDynPD");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      TaskDynPD::
      TaskDynPD( const std::string & name )
	: Task(name)


	,CONSTRUCT_SIGNAL_IN(Kv,double)
	,CONSTRUCT_SIGNAL_IN(qdot,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(dt,double)

	,CONSTRUCT_SIGNAL_OUT(errorDot,dg::Vector,
			      qdotSIN<<jacobianSOUT )
	,CONSTRUCT_SIGNAL_OUT(KvAuto,double,
			      controlGainSIN)
	,CONSTRUCT_SIGNAL_OUT(Jdot,dg::Matrix,
			      jacobianSOUT)
	,CONSTRUCT_SIGNAL_OUT(taskVector,dg::Vector,
			      taskSOUT)

	,previousJ(0u,0u),previousJset(false)
      {
	taskSOUT.setFunction( boost::bind(&TaskDynPD::taskSOUT_function,this,_1,_2) );
	taskSOUT.addDependency( KvSIN );
	taskSOUT.addDependency( errorDotSOUT );

	KvSIN.plug(&KvAutoSOUT);

	signalRegistration( KvSIN << qdotSIN << dtSIN
			    << errorDotSOUT << KvAutoSOUT
			    << JdotSOUT  << taskVectorSOUT );

	addCommand("resetJacobianDerivative",
		   makeCommandVoid0(*this,&TaskDynPD::resetJacobianDerivative,
				    docCommandVoid0("Reset the memory of the numeric jacobian derivative.")));
      }


      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      dg::Vector& TaskDynPD::
      errorDotSOUT_function( dg::Vector& errorDot,int time )
      {
	sotDEBUGIN(15);

	const dg::Vector & qdot = qdotSIN(time);
	const dg::Matrix & J = jacobianSOUT(time);
	errorDot.resize( J.rows() );
	errorDot = J * qdot;

	sotDEBUGOUT(15);
	return errorDot;
      }

      double& TaskDynPD::
      KvAutoSOUT_function( double& Kv,int time )
      {
	sotDEBUGIN(15);

	const double & Kp = controlGainSIN(time);
	assert(Kp>=0);
	Kv = 2*sqrt(Kp);

	sotDEBUGOUT(15);
	return Kv;
      }

      dg::sot::VectorMultiBound& TaskDynPD::
      taskSOUT_function( dg::sot::VectorMultiBound& task,int time )
      {
	sotDEBUGIN(15);

	const dg::Vector & e = errorSOUT(time);
	const dg::Vector & edot_measured = errorDotSOUT(time);
	const dg::Vector & edot_ref = errorTimeDerivativeSOUT(time);
	const double& Kp = controlGainSIN(time);
	const double& Kv = KvSIN(time);

	assert( e.size() == edot_measured.size()  && e.size() == edot_ref.size() );
	task .resize( e.size() );
	for( unsigned int i=0;i<task.size(); ++i )
	  task[i] = - Kp*e(i) - Kv*(edot_measured(i)-edot_ref(i));

	sotDEBUGOUT(15);
	return task;
      }

      dg::Matrix& TaskDynPD::
      JdotSOUT_function( dg::Matrix& Jdot,int time )
      {
	sotDEBUGIN(15);

	const dg::Matrix& currentJ = jacobianSOUT(time);
	const double& dt = dtSIN(time);

	if( previousJ.rows()!=currentJ.rows() ) previousJset = false;

	if( previousJset )
	  {
	    assert( currentJ.rows()==previousJ.rows()
		    && currentJ.cols()==previousJ.cols() );

	    Jdot .resize( currentJ.rows(),currentJ.cols() );
	    Jdot = currentJ - previousJ;
	    Jdot *= 1/dt;
	  }
	else { Jdot.resize(currentJ.rows(),currentJ.cols() ); Jdot.setZero(); }

	previousJ = currentJ;
	previousJset = true;

	sotDEBUGOUT(15);
	return Jdot;
      }

      void TaskDynPD::resetJacobianDerivative( void ) { previousJset = false; }


      dg::Vector& TaskDynPD::
      taskVectorSOUT_function( dg::Vector& taskV, int time )
      {
	const dg::sot::VectorMultiBound & task = taskSOUT(time);
	taskV.resize(task.size());
	for( unsigned int i=0;i<task.size(); ++i )
	  taskV(i) = task[i].getSingleBound();

	return taskV;
      }


      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */

      void TaskDynPD::
      display( std::ostream& os ) const
      {
	os << "TaskDynPD " << name << ": " << std::endl;
	os << "--- LIST ---  " << std::endl;
	BOOST_FOREACH( dg::sot::FeatureAbstract* feature,featureList )
	  {
	    os << "-> " << feature->getName() << std::endl;
	  }
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

