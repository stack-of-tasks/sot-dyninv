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

//#define VP_DEBUG
//#define VP_DEBUG_MODE 50
#include <sot/core/debug.hh>
#include <exception>
#ifdef VP_DEBUG
class solver_op_space__INIT
{
  public:solver_op_space__INIT( void ) { dynamicgraph::sot::DebugTrace::openFile("/tmp/sot-kine-deb.txt"); }
};
solver_op_space__INIT solver_op_space_initiator;
#endif //#ifdef VP_DEBUG


#include <sot-dyninv/solver-kine.h>
#include <sot-dyninv/commands-helper.h>
#include <dynamic-graph/factory.h>
#include <boost/foreach.hpp>
#include <sot-dyninv/commands-helper.h>
#include <dynamic-graph/pool.h>
#include <soth/HCOD.hpp>
#include <sot-dyninv/task-dyn-pd.h>
#include <sot/core/feature-point6d.hh>
#include <sstream>
#include <soth/Algebra.hpp>
#include <Eigen/QR>
#include <sot-dyninv/mal-to-eigen.h>
#include <sys/time.h>

namespace soth
{
  Bound& operator -= (Bound& xb, const double & )
  {
    return xb;
  }

  const Bound operator - (const Bound& a, const Bound & b )
  {
    assert( b.getType()==Bound::BOUND_TWIN || a.getType()==b.getType() );

    if( b.getType() ==Bound::BOUND_TWIN )
      {
	switch( a.getType() )
	  {
	  case Bound::BOUND_TWIN:
	  case Bound::BOUND_INF:
	  case Bound::BOUND_SUP:
	    return Bound(a.getBound(a.getType())-b.getBound(Bound::BOUND_TWIN),
			 a.getType());
	    break;
	  case Bound::BOUND_DOUBLE:
	    return Bound(a.getBound(Bound::BOUND_INF)-b.getBound(Bound::BOUND_TWIN),
			 a.getBound(Bound::BOUND_SUP)-b.getBound(Bound::BOUND_TWIN));
	    break;
	  }
      }
    else
      {
	// TODO
	throw "TODO";
      }


    return a;
  }
}

namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {

      namespace dg = ::dynamicgraph;
      using namespace dg;
      using dg::SignalBase;

      static bool isLH(boost::shared_ptr<soth::Stage> s)
      {
	return s->name == "tasklh";
      }
      static bool isRH(boost::shared_ptr<soth::Stage> s)
      {
	return s->name == "taskrhorient";
      }


     /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SolverKine,"SolverKine");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      SolverKine::
      SolverKine( const std::string & name )
	: Entity(name)
	,stack_t()

	,CONSTRUCT_SIGNAL_IN(damping,double)
	,CONSTRUCT_SIGNAL_IN(velocity,ml::Vector)
	,CONSTRUCT_SIGNAL_OUT(control,ml::Vector,
			      dampingSIN )

	,controlFreeFloating(true)
	,secondOrderKinematics_(false)

	,hsolver()

	,Ctasks(),btasks()
	,solution()
	,activeSet(),relevantActiveSet(false)
	,ddxdriftInit_(false)
	  
      {
	signalRegistration(  controlSOUT 
			    << dampingSIN << velocitySIN );

	/* Command registration. */
	addCommand("debugOnce",
		   makeCommandVoid0(*this,&SolverKine::debugOnce,
				    docCommandVoid0("open trace-file for next iteration of the solver.")));

	addCommand("resetAset",
		   makeCommandVoid0(*this,&SolverKine::resetAset,
				    docCommandVoid0("Reset the active set.")));

	addCommand("decompo",
		   makeCommandVoid1(*this,&SolverKine::getDecomposition,
				    docCommandVoid1("Return the decomposition of the given level.","Stage level")));

	addCommand("setControlFreeFloating",
		   makeDirectSetter(*this,&controlFreeFloating,
				    docDirectSetter("ouput control includes the ff (ie, size nbDof). Oterwise, size is nbDof-6. FF is supposed to be at the head.","bool")));

	std::string docstring =
	  "Set second order kinematic inversion\n"
	  "\n"
	  "  Input: bool\n"
	  "\n"
	  "  If true, check that the solver is empty, since second order\n"
	  "  kinematics requires tasks to be of type TaskDynPD.";
	addCommand("setSecondOrderKinematics",
		   makeCommandVoid0(*this,&SolverKine::setSecondOrderKinematics,
				    docstring));

	addCommand("getSecondOrderKinematics",
		   makeDirectGetter(*this,&secondOrderKinematics_,
				    docDirectGetter("second order kinematic inversion","bool")));

	ADD_COMMANDS_FOR_THE_STACK;
      }

      /* --- STACK ----------------------------------------------------------- */
      /* --- STACK ----------------------------------------------------------- */
      /* --- STACK ----------------------------------------------------------- */
      
      void SolverKine::push (TaskAbstract& task)
      {
	if (secondOrderKinematics_) {
	  checkDynamicTask (task);
	}
	sot::Stack< TaskAbstract >::push (task);
      }

      void SolverKine::checkDynamicTask (const TaskAbstract& task) const
      {
	try {
	  dynamic_cast <const TaskDynPD &> (task);
	} catch (const std::bad_cast& esc) {
	  std::string taskName = task.getName ();
	  std::string className = task.getClassName ();
	  std::string msg ("Type " + className + " of task \"" + taskName +
			   "\" does not derive from TaskDynPD");
	  throw std::runtime_error (msg);
	}
      }

      void SolverKine::setSecondOrderKinematics ()
      {
	if (stack.size() != 0) {
	  throw std::runtime_error
	    ("The solver should contain no task before switching to second order mode.");
	}
	secondOrderKinematics_ = true;
      }

      SolverKine::TaskDependancyList_t SolverKine::
      getTaskDependancyList( const TaskAbstract& task )
      {
	TaskDependancyList_t res;
	res.push_back( &task.taskSOUT );
	res.push_back( &task.jacobianSOUT );
	return res;
      }
      void SolverKine::
      addDependancy( const TaskDependancyList_t& depList )
      {
	BOOST_FOREACH( const SignalBase<int>* sig, depList )
	  { controlSOUT.addDependency( *sig ); }
      }
      void  SolverKine::
      removeDependancy( const TaskDependancyList_t& depList )
      {
	BOOST_FOREACH( const SignalBase<int>* sig, depList )
	  { controlSOUT.removeDependency( *sig ); }
      }
      void SolverKine::
      resetReady( void )
      {
	controlSOUT.setReady();
      }


      /* --- INIT SOLVER ------------------------------------------------------ */
      /* --- INIT SOLVER ------------------------------------------------------ */
      /* --- INIT SOLVER ------------------------------------------------------ */

      /** Force the update of all the task in-signals, in order to fix their
       * size for resizing the solver.
       */
      void SolverKine::
      refreshTaskTime( int time )
      {
	BOOST_FOREACH( TaskAbstract* task, stack )
	  {
	    task->taskSOUT( time );
	  }
      }

      /** Knowing the sizes of all the stages (except the task ones),
       * the function resizes the matrix and vector of all stages (except...).
       */
      void SolverKine::
      resizeSolver( void )
      {
	hsolver = hcod_ptr_t(new soth::HCOD( nbDofs,stack.size() ));

	Ctasks.resize(stack.size());
	btasks.resize(stack.size());
	relevantActiveSet = false;

	int i=0;
	BOOST_FOREACH( TaskAbstract* task, stack )
	  {
	    const int nx = task->taskSOUT.accessCopy().size();
	    Ctasks[i].resize(nx,nbDofs);
	    btasks[i].resize(nx);

	    hsolver->pushBackStage( Ctasks[i],btasks[i] );
	    hsolver->stages.back()->name = task->getName();
	    i++;
	  }

	solution.resize( nbDofs );
      }

      /* Return true iff the solver sizes fit to the task set. */
      bool SolverKine::
      checkSolverSize( void )
      {
	sotDEBUGIN(15);

	assert( nbDofs>0 );

	if(! hsolver ) return false;
	if( stack.size() != hsolver->nbStages() ) return false;

	bool toBeResized=false;
	for( int i=0;i<(int)stack.size();++i )
	  {
	    assert( Ctasks[i].cols() == nbDofs && Ctasks[i].rows() == btasks[i].size() );
	    TaskAbstract & task = *stack[i];
	    if( btasks[i].size() != (int)task.taskSOUT.accessCopy().size() )
	      {
		toBeResized = true;
		break;
	      }
	  }

	return !toBeResized;
      }


      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

#define COLS_Q leftCols( nbDofs )
#define COLS_TAU leftCols( nbDofs+ntau ).rightCols( ntau )
#define COLS_F rightCols( nfs )

      std::ofstream fout("/tmp/kine2-chrono.dat");

      ml::Vector& SolverKine::
      controlSOUT_function( ml::Vector &mlcontrol, int t )
      {
	struct timeval t0,t1,t2,t3,t4;	
	gettimeofday(&t0,NULL);

	sotDEBUG(15) << " # In time = " << t << std::endl;

	refreshTaskTime( t );
	if(! checkSolverSize() ) resizeSolver();

	using namespace soth;

	if( dampingSIN ) //damp?
	  {
	    sotDEBUG(5) << "Using damping. " << std::endl;
	    /* Only damp the final stage of the stack, 'cose of the solver known limitation. */
	    hsolver->setDamping( 0 );
	    hsolver->useDamp( true );
	    if (hsolver->stages.size()!=0)
	      hsolver->stages.back()->damping( dampingSIN(t) );
	  }
	else
	  {
	    sotDEBUG(5) << "Without damping. " << std::endl;
	    hsolver->useDamp( false );
	  }

	/* -Tasks 1:n- */
	/* Ctaski = [ Ji 0 0 0 0 0 ] */

	gettimeofday(&t1,NULL);

	if( !secondOrderKinematics_ )
	  {
	  for( int i=0;i<(int)stack.size();++i )
	    {
	      TaskAbstract & task = * stack[i];
	      MatrixXd & Ctask = Ctasks[i];
	      VectorBound & btask = btasks[i];

	      EIGEN_CONST_MATRIX_FROM_SIGNAL(J,task.jacobianSOUT(t));
	      const dg::sot::VectorMultiBound & dx = task.taskSOUT(t);
	      const int nx = dx.size();

	      assert( Ctask.rows() == nx && btask.size() == nx );
	      assert( J.rows()==nx && J.cols()==nbDofs && (int)dx.size()==nx );

	      Ctask = J;  COPY_MB_VECTOR_TO_EIGEN(dx,btask);

	      sotDEBUG(15) << "Ctask"<<i<<" = "     << (MATLAB)Ctask << std::endl;
	      sotDEBUG(1) << "btask"<<i<<" = "     << btask << std::endl;

	    } //for
	  } //if
	else
	  {

	    for( int i=0;i<(int)stack.size();++i )
	      {
		TaskAbstract & taskAb = * stack[i];
		TaskDynPD & task = dynamic_cast<TaskDynPD &>(taskAb); //it can be static_cast cause of type control
		
		MatrixXd & Ctask = Ctasks[i];
		VectorBound & btask = btasks[i];
		
		EIGEN_CONST_MATRIX_FROM_SIGNAL(Jdot,task.JdotSOUT(t));
		EIGEN_CONST_MATRIX_FROM_SIGNAL(J,task.jacobianSOUT(t));
		const dg::sot::VectorMultiBound & ddx = task.taskSOUT(t);
		EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,velocitySIN(t));

		const int nx1 = ddx.size();

		sotDEBUG(5) << "ddx"<<i<<" = " << ddx << std::endl;
		sotDEBUG(25) << "J"<<i<<" = " << J << std::endl;
		sotDEBUG(45) << "Jdot"<<i<<" = " << Jdot << std::endl;
		sotDEBUG(1) << "dq = " << (MATLAB)dq << std::endl;

		assert( Ctask.rows() == nx1 && btask.size() == nx1 );
		assert( J.rows()==nx1 && J.cols()==nbDofs && (int)ddx.size()==nx1 );
		assert( Jdot.rows()==nx1 && Jdot.cols()==nbDofs );
		
		Ctask = J;

		if (!ddxdriftInit_)
		  {
		    ddxdrift_= VectorXd(nx1);
		    ddxdriftInit_=true;
		  }
		ddxdrift_ = - (Jdot*dq);
		    
		for( int c=0;c<nx1;++c )
		  {
		    if( ddx[c].getMode() == dg::sot::MultiBound::MODE_SINGLE )
		      btask[c] = ddx[c].getSingleBound() + ddxdrift_[c];
		    else
		      {
			const bool binf = ddx[c].getDoubleBoundSetup( dg::sot::MultiBound::BOUND_INF );
			const bool bsup = ddx[c].getDoubleBoundSetup( dg::sot::MultiBound::BOUND_SUP );
			if( binf&&bsup )
			  {
			    const double xi = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_INF);
			    const double xs = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_SUP);
			    btask[c] = std::make_pair( xi+ddxdrift_[c], xs+ddxdrift_[c] );
			  }
			else if( binf )
			  {
			    const double xi = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_INF);
			    btask[c] = Bound( xi+ddxdrift_[c], Bound::BOUND_INF );
			  }
			else
			  {
			    assert( bsup );
			    const double xs = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_SUP);	
			    btask[c] = Bound( xs+ddxdrift_[c], Bound::BOUND_SUP );
			  } //else
		      } //else
		  } //for c
		
		sotDEBUG(15) << "Ctask"<<i<<" = "     << (MATLAB)Ctask << std::endl;
		sotDEBUG(1) << "btask"<<i<<" = "     << btask << std::endl;
	      } //for i
	  } //else

	/* --- */
	sotDEBUG(1) << "Initial config." << std::endl;

	hsolver->reset();
	if(relevantActiveSet) 
	  hsolver->setInitialActiveSet(activeSet);
	else hsolver->setInitialActiveSet();

	sotDEBUG(1) << "Run for a solution." << std::endl;

	gettimeofday(&t2,NULL);

	hsolver->activeSearch(solution);

	gettimeofday(&t3,NULL);

	sotDEBUG(1) << "solution = " << (MATLAB)solution << std::endl;

	activeSet = hsolver->getOptimalActiveSet(); relevantActiveSet = true;

	
	if( controlFreeFloating )
	  {
	    EIGEN_VECTOR_FROM_VECTOR( control,mlcontrol,nbDofs );
	    control=solution;
	  }
	else
	  {
	    EIGEN_VECTOR_FROM_VECTOR( control,mlcontrol,nbDofs-6 );
	    control=solution.tail( nbDofs-6 );
	  }

	gettimeofday(&t4,NULL);

	double dt1,dt2,dt3,dt4,dt;
	dt1 = (t1.tv_sec-t0.tv_sec)*1000.0 + (t1.tv_usec-t0.tv_usec)/1000.0;
	dt2 = (t2.tv_sec-t1.tv_sec)*1000.0 + (t2.tv_usec-t1.tv_usec)/1000.0;
	dt3 = (t3.tv_sec-t2.tv_sec)*1000.0 + (t3.tv_usec-t2.tv_usec)/1000.0;
	dt4 = (t4.tv_sec-t3.tv_sec)*1000.0 + (t4.tv_usec-t3.tv_usec)/1000.0;
	dt = dt1 + dt2 + dt3 + dt4;

	//fout << "At time " << t << ": dt1=" << dt1 << " dt2=" << dt2 << " dt3=" << dt3 << " dt4=" << dt4 << std::endl;

	fout << dt1 << " " << dt2 << " " << dt3 << " " << dt4 << " " << dt << std::endl;

	sotDEBUG(1) << "control = " << mlcontrol << std::endl;
	return mlcontrol;
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */

      void SolverKine::
      debugOnce( void )
      {
	std::cout << "Open the trace"<<std::endl;
	dg::sot::DebugTrace::openFile("/tmp/sot.txt");
	hsolver->debugOnce("/tmp/soth.txt",true);
      }
      void SolverKine::
      resetAset( void )
      {
	relevantActiveSet = false;
      }

      void SolverKine::
      getDecomposition(const int & i)
      {
	using namespace soth;
	std::cout << "M"<<i<<" = " << (MATLAB) hsolver -> stage(i).getM() << std::endl;
	std::cout << "L"<<i<<" = " << (MATLAB)(MatrixXd) (hsolver -> stage(i).getLtri()) << std::endl;
      }

      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */

      void SolverKine::
      display( std::ostream& os ) const
      {
	os << "SolverKine "<<getName() << ": " << nbDofs <<" joints." << std::endl;
	try{
	  os <<"control = "<<controlSOUT.accessCopy() <<std::endl << std::endl;
	}  catch (dynamicgraph::ExceptionSignal e) {}
	stack_t::display(os);

      }

      void SolverKine::
      commandLine( const std::string& cmdLine,
		   std::istringstream& cmdArgs,
		   std::ostream& os )
      {
	if( cmdLine == "help" )
	  {
	    os << "SolverKine:\n"
	       << "\t- debugOnce: open trace-file for next iteration of the solver." << std::endl;
	    stackCommandLine( cmdLine,cmdArgs,os );
	    Entity::commandLine( cmdLine,cmdArgs,os );
	  }
	else if( cmdLine == "debugOnce" )
	  {
	    debugOnce();
	  }
	/* TODO: add controlFreeFloating g/setter. */
	else if( stackCommandLine( cmdLine,cmdArgs,os ) );
	else
	  {
	    Entity::commandLine( cmdLine,cmdArgs,os );
	  }
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

