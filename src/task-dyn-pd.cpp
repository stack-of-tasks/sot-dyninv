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

#include <dynamic-graph/factory.h>

#include <sot-core/debug.h>
#include <sot-core/feature-abstract.h>

#include <sot-dyninv/task-dyn-pd.h>
#include <sot-dyninv/commands-helper.h>

#include <boost/foreach.hpp>

namespace sot
{
  namespace dyninv
  {

    namespace dg = ::dynamicgraph;
    using namespace dg;

    /* --- DG FACTORY ------------------------------------------------------- */
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskDynPD,"TaskDynPD");

    /* --- CONSTRUCTION ----------------------------------------------------- */
    /* --- CONSTRUCTION ----------------------------------------------------- */
    /* --- CONSTRUCTION ----------------------------------------------------- */
    TaskDynPD::
    TaskDynPD( const std::string & name )
      : Task(name)


      ,CONSTRUCT_SIGNAL_IN(Kv,double)
      ,CONSTRUCT_SIGNAL_IN(qdot,ml::Vector)
      ,CONSTRUCT_SIGNAL_IN(dt,double)

      ,CONSTRUCT_SIGNAL_OUT(errorDot,ml::Vector,
			    qdotSIN<<jacobianSOUT )
      ,CONSTRUCT_SIGNAL_OUT(KvAuto,double,
       			     controlGainSIN)
      ,CONSTRUCT_SIGNAL_OUT(Jdot,ml::Matrix,
       			     jacobianSOUT)
      ,CONSTRUCT_SIGNAL_OUT(taskVector,ml::Vector,
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
    }


    /* --- SIGNALS ---------------------------------------------------------- */
    /* --- SIGNALS ---------------------------------------------------------- */
    /* --- SIGNALS ---------------------------------------------------------- */

    ml::Vector& TaskDynPD::
    errorDotSOUT_function( ml::Vector& errorDot,int time )
    {
      sotDEBUGIN(15);

      const ml::Vector & qdot = qdotSIN(time);
      const ml::Matrix & J = jacobianSOUT(time);
      errorDot.resize( J.nbRows() );
      J.multiply(qdot,errorDot);

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

    sot::VectorMultiBound& TaskDynPD::
    taskSOUT_function( sot::VectorMultiBound& task,int time )
    {
      sotDEBUGIN(15);

      const ml::Vector & e = errorSOUT(time);
      const ml::Vector & edot = errorDotSOUT(time);
      const double& Kp = controlGainSIN(time);
      const double& Kv = KvSIN(time);

      assert( e.size() == edot.size() );
      task .resize( e.size() );
      for( unsigned int i=0;i<task.size(); ++i )
	task[i] = - Kp*e(i) - Kv*edot(i);

      sotDEBUGOUT(15);
      return task;
    }

    ml::Matrix& TaskDynPD::
    JdotSOUT_function( ml::Matrix& Jdot,int time )
    {
      sotDEBUGIN(15);

      const ml::Matrix& currentJ = jacobianSOUT(time);
      const double& dt = dtSIN(time);

      if( previousJ.nbRows()!=currentJ.nbRows() ) previousJset = false;

      if( previousJset )
	{
	  assert( currentJ.nbRows()==previousJ.nbRows()
		  && currentJ.nbCols()==previousJ.nbCols() );

	  Jdot .resize( currentJ.nbRows(),currentJ.nbCols() );
	  Jdot = currentJ - previousJ;
	  Jdot *= 1/dt;
	}
      else { Jdot.resize(currentJ.nbRows(),currentJ.nbCols() ); Jdot.fill(0); }

      previousJ = currentJ;
      previousJset = true;

      sotDEBUGOUT(15);
      return Jdot;
    }

    ml::Vector& TaskDynPD::
    taskVectorSOUT_function( ml::Vector& taskV,             int time )
    {
      const sot::VectorMultiBound & task = taskSOUT(time);
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
      BOOST_FOREACH( sot::FeatureAbstract* feature,featureList )
	{
	  os << "-> " << feature->getName() << std::endl;
	}
    }

  } // namespace dyninv
} // namespace sot

