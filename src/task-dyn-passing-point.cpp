/*
 * Copyright 2012, Oscar E. Ramos Ponce, LAAS-CNRS
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

#include <sot/core/debug.hh>
#include <sot/core/feature-abstract.hh>

#include <sot-dyninv/task-dyn-passing-point.h>
#include <sot-dyninv/commands-helper.h>

#include <boost/foreach.hpp>
#include <Eigen/Dense>


namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {

      namespace dg = ::dynamicgraph;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskDynPassingPoint,"TaskDynPassingPoint");

      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      TaskDynPassingPoint::
      TaskDynPassingPoint( const std::string & name )
	: TaskDynPD(name)

	,CONSTRUCT_SIGNAL_IN(velocityDes, dg::Vector)
	,CONSTRUCT_SIGNAL_IN(duration, double)
	,CONSTRUCT_SIGNAL_IN(initialTime, double)

	,CONSTRUCT_SIGNAL_OUT(velocityCurrent, dg::Vector,
			      qdotSIN<<jacobianSOUT )
	,CONSTRUCT_SIGNAL_OUT(velocityDesired, dg::Vector,
			      velocityDesSIN<<controlSelectionSIN )

	,previousTask()
      {
	taskSOUT.setFunction( boost::bind(&TaskDynPassingPoint::computeTaskSOUT,this,_1,_2) );
	taskSOUT.addDependency( velocityCurrentSOUT );
	taskSOUT.addDependency( durationSIN );
	taskSOUT.addDependency( velocityDesiredSOUT );

	signalRegistration( velocityDesSIN << durationSIN << initialTimeSIN
			    << velocityCurrentSOUT << velocityDesiredSOUT );

      }


      /* ---------------------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      /* Current velocity using the Jacobian: dx_0 = J*dq */
      dg::Vector& TaskDynPassingPoint::
      velocityCurrentSOUT_function( dg::Vector& velocityCurrent, int time )
      {
      	sotDEBUGIN(15);

      	const dg::Vector & qdot = qdotSIN(time);
      	const dg::Matrix & J    = jacobianSOUT(time);
      	velocityCurrent.resize( J.rows() );
      	velocityCurrent = J*qdot;

      	sotDEBUGOUT(15);
      	return velocityCurrent;
      }


      /* Select the correct components of the desired velocity according to 'selec' */
      dg::Vector& TaskDynPassingPoint::
      velocityDesiredSOUT_function( dg::Vector& velocityDesired, int time )
      {
      	sotDEBUGIN(15);

	const dg::Vector & velocityDesiredFull = velocityDesSIN(time);
	const Flags &fl = controlSelectionSIN(time);

	unsigned int dim = 0;
	for( int i=0;i<6;++i ) if( fl(i) ) dim++;
	velocityDesired.resize( dim );

	unsigned int cursor = 0;
	for( unsigned int i=0;i<6;++i )
	  { if( fl(i) ) velocityDesired(cursor++) = velocityDesiredFull(i); }

      	sotDEBUGOUT(15);
      	return velocityDesired;
      }


      /* Task computation */
      dg::sot::VectorMultiBound& TaskDynPassingPoint::
      computeTaskSOUT( dg::sot::VectorMultiBound& task, int time )
      {
	sotDEBUGIN(15);

	const double& fullDuration     = durationSIN(time);
	const double& n0               = initialTimeSIN(time);
	const double& dt               = dtSIN(time);
	const dg::Vector & e           = errorSOUT(time);
	const dg::Vector & vel_current = velocityCurrentSOUT(time);
	const dg::Vector & vel_desired = velocityDesiredSOUT(time);

	double T = fabs( fullDuration-(time-n0)*dt );
	//double T = fullDuration-(time-n0)*dt;
	//std::cout << "duration left: " << T << std::endl;
	
	assert( e.size() == vel_current.size()  && e.size() == vel_desired.size() );
	task.resize( e.size() );

	if(previousTask.size() != task.size())
	  previousTask.resize( e.size() );

	/* --- Using a pseudoinverse --- */
	// Eigen::MatrixXd M_t = Eigen::MatrixXd::Zero(2*e.size(),2*e.size());
	// Eigen::VectorXd V_ddx;
	// Eigen::VectorXd b;

	// V_ddx.resize(2*e.size());
	// b.resize(2*e.size());

	// for( unsigned int i=0; i<task.size(); ++i)
	//   {
	//     M_t(i,i) = T*T/3.0;
	//     M_t(i,i+e.size())= T*T/6.0;
	//     M_t(i+e.size(),i) = T/2.0;
	//     M_t(i+e.size(),i+e.size()) = T/2;

	//     b(i) = -e(i) - vel_current(i)*T;
	//     b(i+e.size()) = vel_desired(i)-vel_current(i);
	//   }

	// //V_ddx = M_t.colPivHouseholderQr().solve(b);
	// Eigen::ColPivHouseholderQR<Eigen::MatrixXd> colPiv(M_t);
	// colPiv.setThreshold(1e-3);
	// V_ddx = colPiv.solve(b);

	// V_ddx = M_t.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	// std::cout << " == M_t:" << std::endl << M_t << std::endl;
	// std::cout << " == b:" << std::endl << b << std::endl;
	// std::cout << " == V_ddx:" << std::endl<< V_ddx << std::endl;

	// for (unsigned int i=0; i<task.size(); ++i)
	//   {
	//     task[i] = V_ddx(i);
	//   }
	
	/* --- Computing ddx(0) "manually" --- */
	for( unsigned int i=0;i<task.size(); ++i )
	  {
	    if (T>dt)
	      {
		task[i] = - 6*e(i)/(T*T) - 2/T*(vel_desired(i)+2*vel_current(i)) ;
		previousTask(i) = - 6*e(i)/(T*T) - 2/T*(vel_desired(i)+2*vel_current(i)) ;
	      }
	    else
	      {
		task[i] = previousTask(i);
		//task[i] = 0;
	      }
	  }

	sotDEBUGOUT(15);
	return task;
      }


      /* ---------------------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      void TaskDynPassingPoint::
      display( std::ostream& os ) const
      {
      	os << "TaskDynPassingPoint " << name << ": " << std::endl;
      	os << "--- LIST ---  " << std::endl;
      	BOOST_FOREACH( dg::sot::FeatureAbstract* feature,featureList )
      	  {
      	    os << "-> " << feature->getName() << std::endl;
      	  }
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

