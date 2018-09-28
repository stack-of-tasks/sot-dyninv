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
#define VP_DEBUG_MODE 50
#include <sot/core/debug.hh>
#ifdef VP_DEBUG
class solver_op_space__INIT
{
public:solver_op_space__INIT( void )
  {
    //    dynamicgraph::sot::DebugTrace::openFile("/tmp/dynred.txt");
  }
};
solver_op_space__INIT solver_op_space_initiator;
#endif //#ifdef VP_DEBUG


#include <sot-dyninv/solver-dyn-reduced.h>
#include <sot-dyninv/commands-helper.h>
#include <dynamic-graph/factory.h>
#include <boost/foreach.hpp>
#include <sot-dyninv/commands-helper.h>
#include <dynamic-graph/pool.h>
#include <soth/HCOD.hpp>
#include <sot-dyninv/task-dyn-pd.h>
#include <sot/core/feature-point6d.hh>
#include <iostream>
#include <sstream>
#include <soth/Algebra.hpp>
#include <Eigen/QR>

namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {
#include <Eigen/Cholesky>

      namespace dg = ::dynamicgraph;
      using namespace dg;
      using dg::SignalBase;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SolverDynReduced,"SolverDynReduced");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      SolverDynReduced::
      SolverDynReduced( const std::string & name )
	: Entity(name)
	,stack_t()

	,CONSTRUCT_SIGNAL_IN(matrixInertia,dg::Matrix)
	,CONSTRUCT_SIGNAL_IN(inertiaSqroot,dg::Matrix)
	,CONSTRUCT_SIGNAL_IN(inertiaSqrootInv,dg::Matrix)
	,CONSTRUCT_SIGNAL_IN(velocity,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(dyndrift,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(damping,double)
	,CONSTRUCT_SIGNAL_IN(breakFactor,double)
	,CONSTRUCT_SIGNAL_IN(posture,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(position,dg::Vector)

	,CONSTRUCT_SIGNAL_OUT(precompute,int,
			      inertiaSqrootInvSIN << inertiaSqrootSIN )
	,CONSTRUCT_SIGNAL_OUT(inertiaSqrootOut,dg::Matrix,
			      matrixInertiaSIN)
	,CONSTRUCT_SIGNAL_OUT(inertiaSqrootInvOut,dg::Matrix,
			      inertiaSqrootSIN)
	,CONSTRUCT_SIGNAL_OUT(sizeForcePoint,int,
			      precomputeSOUT )
	,CONSTRUCT_SIGNAL_OUT(sizeForceSpatial,int,
			      precomputeSOUT )
	,CONSTRUCT_SIGNAL_OUT(sizeConfiguration,int,
			      velocitySIN )

	,CONSTRUCT_SIGNAL_OUT(Jc,dg::Matrix, sotNOSIGNAL)
	,CONSTRUCT_SIGNAL_OUT(forceGenerator,dg::Matrix, sotNOSIGNAL)
	,CONSTRUCT_SIGNAL_OUT(freeMotionBase,dg::Matrix,
			      JcSOUT << inertiaSqrootInvSIN)
	,CONSTRUCT_SIGNAL_OUT(freeForceBase,dg::Matrix,
			      forceGeneratorSOUT << JcSOUT)
	,CONSTRUCT_SIGNAL_OUT(driftContact,dg::Vector,
			      freeMotionBaseSOUT<<JcSOUT )

	,CONSTRUCT_SIGNAL_OUT(sizeMotion,int,
			      freeMotionBaseSOUT )
	,CONSTRUCT_SIGNAL_OUT(sizeActuation,int,
			      freeForceBaseSOUT )


	,CONSTRUCT_SIGNAL_OUT(solution,dg::Vector,
			      freeMotionBaseSOUT << inertiaSqrootInvSIN << inertiaSqrootSIN
			      << dyndriftSIN << velocitySIN
			      << dampingSIN << breakFactorSIN
			      << postureSIN << positionSIN)
	,CONSTRUCT_SIGNAL_OUT(reducedControl,dg::Vector,
			      solutionSOUT)
	,CONSTRUCT_SIGNAL_OUT(reducedForce,dg::Vector,
			      solutionSOUT)
	,CONSTRUCT_SIGNAL_OUT(acceleration,dg::Vector,
			      reducedControlSOUT << inertiaSqrootSIN << freeMotionBaseSOUT)
	,CONSTRUCT_SIGNAL_OUT(forces,dg::Vector,
			      reducedForceSOUT)
	,CONSTRUCT_SIGNAL_OUT(torque,dg::Vector,
			      JcSOUT << forcesSOUT << reducedControlSOUT << inertiaSqrootSIN )

	,CONSTRUCT_SIGNAL_OUT(forcesNormal,dg::Vector,
			      solutionSOUT)
	,CONSTRUCT_SIGNAL_OUT(activeForces,dg::Vector,
			      solutionSOUT)


	,CONSTRUCT_SIGNAL(Jcdot,OUT,dg::Matrix)

	,hsolver()

	,Cforce(),Czero()
	,bforce(),bzero()
	,Ctasks(),btasks()
	,solution()
      {
	signalRegistration( matrixInertiaSIN
			    << inertiaSqrootSIN
			    << inertiaSqrootInvSIN
			    << velocitySIN
			    << dyndriftSIN
			    << dampingSIN
			    << breakFactorSIN
			    << postureSIN
			    << positionSIN

			    );
	signalRegistration(
			   inertiaSqrootOutSOUT
			   << inertiaSqrootInvOutSOUT
			   << JcSOUT
			   << freeMotionBaseSOUT
			   << freeForceBaseSOUT
			   << driftContactSOUT

			   << sizeActuationSOUT
			   << sizeMotionSOUT
			   << sizeForceSpatialSOUT
			   << sizeForcePointSOUT
			   << forceGeneratorSOUT
			   << solutionSOUT
			   << reducedControlSOUT
			   << reducedForceSOUT
			   << accelerationSOUT
			   << forcesSOUT
			   << torqueSOUT

			   << JcdotSOUT
			   );
	signalRegistration( forcesNormalSOUT << activeForcesSOUT );

	inertiaSqrootInvSIN.plug( &inertiaSqrootInvOutSOUT );
	inertiaSqrootSIN.plug( &inertiaSqrootOutSOUT );

	/* Command registration. */
	boost::function<void(SolverDynReduced*,const std::string&)> f_addContact
	  =
	  boost::bind( &SolverDynReduced::addContact,_1,_2,
		       (dynamicgraph::Signal<dg::Matrix, int>*)NULL,
		       (dynamicgraph::Signal<dg::Matrix, int>*)NULL,
		       (dynamicgraph::Signal<dg::Vector, int>*)NULL,
		       (dynamicgraph::Signal<dg::Matrix, int>*)NULL);
	addCommand("addContact",
		   makeCommandVoid1(*this,f_addContact,
				    docCommandVoid1("create the contact signals, unpluged.",
						    "string")));
	addCommand("addContactFromTask",
		   makeCommandVoid2(*this,&SolverDynReduced::addContactFromTask,
				    docCommandVoid2("Add a contact from the named task. Remmeber to plug __p.",
						    "string(task name)","string (contact name)")));
	addCommand("rmContact",
		   makeCommandVoid1(*this,&SolverDynReduced::removeContact,
				    docCommandVoid1("remove the contact named in arguments.",
						    "string")));
	addCommand("dispContacts",
		   makeCommandVerbose(*this,&SolverDynReduced::dispContacts,
				      docCommandVerbose("Guess what?")));
	addCommand("debugOnce",
		   makeCommandVoid0(*this,&SolverDynReduced::debugOnce,
				    docCommandVoid0("open trace-file for next iteration of the solver.")));

	ADD_COMMANDS_FOR_THE_STACK;
      }

      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */

      void SolverDynReduced::
      debugOnce( void )
      {
	dg::sot::DebugTrace::openFile("/tmp/dynred.txt");
	//hsolver->debugOnce();
      }

      /* --- STACK ----------------------------------------------------------- */
      /* --- STACK ----------------------------------------------------------- */
      /* --- STACK ----------------------------------------------------------- */

      SolverDynReduced::TaskDependancyList_t SolverDynReduced::
      getTaskDependancyList( const TaskDynPD& task )
      {
	TaskDependancyList_t res;
	res.push_back( &task.taskSOUT );
	res.push_back( &task.jacobianSOUT );
	res.push_back( &task.JdotSOUT );
	return res;
      }
      void SolverDynReduced::
      addDependancy( const TaskDependancyList_t& depList )
      {
	BOOST_FOREACH( const SignalBase<int>* sig, depList )
	  { solutionSOUT.addDependency( *sig ); }
      }
      void  SolverDynReduced::
      removeDependancy( const TaskDependancyList_t& depList )
      {
	BOOST_FOREACH( const SignalBase<int>* sig, depList )
	  { solutionSOUT.removeDependency( *sig ); }
      }
      void SolverDynReduced::
      resetReady( void )
      {
	solutionSOUT.setReady();
      }

      /* --- CONTACT LIST ---------------------------------------------------- */
      /* --- CONTACT LIST ---------------------------------------------------- */
      /* --- CONTACT LIST ---------------------------------------------------- */

      /* TODO: push this method directly in signal. */
      static std::string signalShortName( const std::string & longName )
      {
	std::istringstream iss( longName );
	const int SIZE = 128;
	char buffer[SIZE];
	while( iss.good() )
	  { iss.getline(buffer,SIZE,':'); }
	return std::string( buffer );
      }

      void SolverDynReduced::
      addContactFromTask( const std::string & taskName,const std::string & contactName )
      {
	using namespace dynamicgraph::sot;

	TaskDynPD & task = dynamic_cast<TaskDynPD&> ( g_pool().getEntity( taskName ) );
	assert( task.getFeatureList().size() == 1 );
	BOOST_FOREACH( FeatureAbstract* fptr, task.getFeatureList() )
	  {
	    FeaturePoint6d* f6 = dynamic_cast< FeaturePoint6d* >( fptr );
	    assert( NULL!=f6 );
	    f6->positionSIN.recompute(solutionSOUT.getTime());
	    f6->servoCurrentPosition();
	    f6->FeatureAbstract::selectionSIN = true;
	  }
	addContact( contactName, &task.jacobianSOUT, &task.JdotSOUT,&task.taskVectorSOUT, NULL );
      }

      void SolverDynReduced::
      addContact( const std::string & name,
		  Signal<dg::Matrix,int> * jacobianSignal,
		  Signal<dg::Matrix,int> * JdotSignal,
		  Signal<dg::Vector,int> * corrSignal,
		  Signal<dg::Matrix,int> * contactPointsSignal )
      {
	if( contactMap.find(name) != contactMap.end())
	  {
	    std::cerr << "!! Contact " << name << " already exists." << std::endl;
	    return;
	  }

	contactMap[name].jacobianSIN
	  = matrixSINPtr( new SignalPtr<dg::Matrix,int>
			  ( jacobianSignal,
			    "sotDynInvWB("+getName()+")::input(matrix)::_"+name+"_J" ) );
	signalRegistration( *contactMap[name].jacobianSIN );
	JcSOUT.addDependency( *contactMap[name].jacobianSIN );
	precomputeSOUT.addDependency( *contactMap[name].jacobianSIN );
	JcSOUT.setReady();
	precomputeSOUT.setReady();

	contactMap[name].JdotSIN
	  = matrixSINPtr( new SignalPtr<dg::Matrix,int>
			  ( JdotSignal,
			    "sotDynInvWB("+getName()+")::input(matrix)::_"+name+"_Jdot" ) );
	signalRegistration( *contactMap[name].JdotSIN );

	contactMap[name].supportSIN
	  = matrixSINPtr( new SignalPtr<dg::Matrix,int>
			  ( contactPointsSignal,
			    "sotDynInvWB("+getName()+")::input(matrix)::_"+name+"_p" ) );
	signalRegistration( *contactMap[name].supportSIN );
	forceGeneratorSOUT.addDependency( *contactMap[name].supportSIN );

	contactMap[name].correctorSIN
	  = vectorSINPtr( new SignalPtr<dg::Vector,int>
			  ( corrSignal,
			    "sotDynInvWB("+getName()+")::input(vector)::_"+name+"_x" ) );
	signalRegistration( *contactMap[name].correctorSIN );

	contactMap[name].forceSOUT
	  = vectorSOUTPtr( new Signal<dg::Vector,int>
			   ( "sotDynInvWB("+getName()+")::output(vector)::_"+name+"_f" ) );
	signalRegistration( *contactMap[name].forceSOUT );

	contactMap[name].fnSOUT
	  = vectorSOUTPtr( new Signal<dg::Vector,int>
			   ( "sotDynInvWB("+getName()+")::output(vector)::_"+name+"_fn" ) );
	signalRegistration( *contactMap[name].fnSOUT );

      }

      void SolverDynReduced::
      removeContact( const std::string & name )
      {
	if( contactMap.find(name) == contactMap.end() )
	  {
	    std::cerr << "!! Contact " << name << " does not exist." << std::endl;
	    return;
	  }

	JcSOUT.removeDependency( *contactMap[name].jacobianSIN );
	precomputeSOUT.removeDependency( *contactMap[name].jacobianSIN );
	forceGeneratorSOUT.removeDependency( *contactMap[name].supportSIN );

	JcSOUT.setReady();
	precomputeSOUT.setReady();
	forceGeneratorSOUT.setReady();

	signalDeregistration( signalShortName(contactMap[name].jacobianSIN->getName()) );
	signalDeregistration( signalShortName(contactMap[name].supportSIN->getName()) );
	signalDeregistration( signalShortName(contactMap[name].forceSOUT->getName()) );
	signalDeregistration( signalShortName(contactMap[name].fnSOUT->getName()) );
	signalDeregistration( signalShortName(contactMap[name].JdotSIN->getName()) );
	signalDeregistration( signalShortName(contactMap[name].correctorSIN->getName()) );
	contactMap.erase(name);
      }

      void SolverDynReduced::
      dispContacts( std::ostream& os ) const
      {
	os << "+-----------------\n+   Contacts\n+-----------------" << std::endl;
	// TODO BOOST FOREACH
	for( contacts_t::const_iterator iter=contactMap.begin();
	     iter!=contactMap.end(); ++iter )
	  {
	    os << "| " << iter->first <<std::endl;
	  }
      }

      SolverDynReduced::matrixSINPtr SolverDynReduced::getSupportSIN( const std::string & contacName )
      {
	return contactMap[contacName].supportSIN;
      }



      /* --- INIT SOLVER ------------------------------------------------------ */
      /* --- INIT SOLVER ------------------------------------------------------ */
      /* --- INIT SOLVER ------------------------------------------------------ */

      /** Force the update of all the task in-signals, in order to fix their
       * size for resizing the solver.
       */
      void SolverDynReduced::
      refreshTaskTime( int time )
      {
	// TODO BOOST_FOREACH
	for( StackIterator_t iter=stack.begin();stack.end()!=iter;++iter )
	  {
	    TaskDynPD& task = **iter;
	    task.taskSOUT( time );
	  }
      }



     /* --------------------------------------------------------------------- */
      /* --- STATIC INTERNAL ------------------------------------------------- */
      /* --------------------------------------------------------------------- */
      namespace sotSolverDyn
      {
	template< typename D1,typename D2 >
	void preCross( const Eigen::MatrixBase<D1> & M,Eigen::MatrixBase<D2> & Tx )
	{
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<D1>);
	  assert( Tx.cols()==3 && Tx.rows()==3 && M.size()==3 );
	  Tx( 0,0 ) = 0   ;  Tx( 0,1 )=-M[2];  Tx( 0,2 ) = M[1];
	  Tx( 1,0 ) = M[2];  Tx( 1,1 )= 0   ;  Tx( 1,2 ) =-M[0];
	  Tx( 2,0 ) =-M[1];  Tx( 2,1 )= M[0];  Tx( 2,2 ) = 0   ;
	}

	//soth::VectorBound& vbAssign ( soth::VectorBound& vb, const Eigen::VectorXd& vx )
	template<typename D1,typename D2>
	Eigen::MatrixBase<D1>& vbAssign ( Eigen::MatrixBase<D1>& vb,
					  const Eigen::MatrixBase<D2>& vx )
	{
	  vb.resize(vx.size());
	  for( int i=0;i<vx.size();++i ){ vb[i] = vx[i]; }
	  return vb;
	}
	soth::VectorBound& vbAssign ( soth::VectorBound& vb,
				      const dg::sot::VectorMultiBound& vx )
	{
	  vb.resize(vx.size());
	  for( unsigned int c=0;c<vx.size();++c )
	    {
	      if( vx[c].getMode() == dg::sot::MultiBound::MODE_SINGLE )
		vb[c] = vx[c].getSingleBound();
	      else
		{
		  using dg::sot::MultiBound;
		  using namespace soth;
		  const bool binf = vx[c].getDoubleBoundSetup( MultiBound::BOUND_INF ),
		    bsup = vx[c].getDoubleBoundSetup( MultiBound::BOUND_SUP );
		  if( binf&&bsup )
		    {
		      vb[c]
			= std::make_pair( vx[c].getDoubleBound(MultiBound::BOUND_INF),
					  vx[c].getDoubleBound(MultiBound::BOUND_SUP) );
		    }
		  else if( binf )
		    {
		      vb[c] = Bound( vx[c].getDoubleBound(MultiBound::BOUND_INF),
				     Bound::BOUND_INF );
		    }
		  else
		    {
		      assert( bsup );
		      vb[c] = Bound( vx[c].getDoubleBound(MultiBound::BOUND_SUP),
				     Bound::BOUND_SUP );
		    }

		}
	    }
	  return vb;
	}
	template<typename D>
	soth::VectorBound& vbSubstract ( soth::VectorBound& vb,
					 const Eigen::MatrixBase<D> &vx )
	{
	  using namespace soth;
	  assert( vb.size() == vx.size() );
	  for( int c=0;c<vx.size();++c )
	    {
	      const Bound::bound_t & type = vb[c].getType();
	      switch( type )
		{
		case Bound::BOUND_INF:
		case Bound::BOUND_SUP:
		  vb[c] = Bound( type,vb[c].getBound(type)-vx[c] );
		  break;
		case Bound::BOUND_DOUBLE:
		  vb[c] = std::make_pair( vb[c].getBound(Bound::BOUND_INF)-vx[c],
					  vb[c].getBound(Bound::BOUND_SUP)-vx[c] );
		  break;
		case Bound::BOUND_TWIN:
		  vb[c] = vb[c].getBound(type) - vx[c];
		  break;
		case Bound::BOUND_NONE:
		  assert( false &&"This switch should not happen." );
		  break;
		}
	    }
	  return vb;
	}
	template<typename D>
	soth::VectorBound& vbAdd ( soth::VectorBound& vb,
				   const Eigen::MatrixBase<D> &vx )
	{
	  using namespace soth;
	  assert( vb.size() == vx.size() );
	  for( int c=0;c<vx.size();++c )
	    {
	      const Bound::bound_t & type = vb[c].getType();
	      switch( type )
		{
		case Bound::BOUND_INF:
		case Bound::BOUND_SUP:
		  vb[c] = Bound( type,vb[c].getBound(type)+vx[c] );
		  break;
		case Bound::BOUND_DOUBLE:
		  vb[c] = std::make_pair( vb[c].getBound(Bound::BOUND_INF)+vx[c],
					  vb[c].getBound(Bound::BOUND_SUP)+vx[c] );
		  break;
		case Bound::BOUND_TWIN:
		  vb[c] = vb[c].getBound(type) + vx[c];
		  break;
		case Bound::BOUND_NONE:
		  assert( false &&"This switch should not happen." );
		  break;
		}
	    }
	  return vb;
	}

	/* TODO: inherite from JacobiSVD a structure where rank can be computed dynamically. */
	// inline int svdRankDefEval( const Eigen::JacobiSVD<Eigen::MatrixXd >& Msvd,
	// 			   const double threshold = 1e-5 )
	// {
	//   return (Msvd.singularValues().array() > threshold ).count();
	// }
  	// template<typename D2>
	// Eigen::VectorXd svdRankDefSolve( const Eigen::JacobiSVD<Eigen::MatrixXd >& Msvd,
	// 				 const Eigen::MatrixBase<D2>& y,
	// 				 const int rank )
	// {
	//   assert( Msvd.computeU() && Msvd.computeV() );
	//   return
	//     Msvd.matrixV().leftCols(rank) *
	//     Msvd.singularValues().array().head(rank).inverse().matrix().asDiagonal() *
	//     Msvd.matrixU().leftCols(rank).transpose()*y;
	// }
      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      // TODO: the following should be replace by the middleColumn and
      // middleRow functions.
#define COLS(__ri,__rs) leftCols(__rs).rightCols((__rs)-(__ri))
#define ROWS(__ri,__rs) topRows(__rs).bottomRows((__rs)-(__ri))

      dg::Matrix& SolverDynReduced::
      inertiaSqrootOutSOUT_function( dg::Matrix& mlAsq, int t )
      {
	const Eigen::MatrixXd A = matrixInertiaSIN(t);
	mlAsq.resize(A.rows(),A.cols());

	sotDEBUG(1) << "A = " << (soth::MATLAB)A << std::endl;

	mlAsq = A.llt().matrixU();

	/* Asq is such that Asq^T Asq = A. */
	return mlAsq;
      }

      dg::Matrix& SolverDynReduced::
      inertiaSqrootInvOutSOUT_function( dg::Matrix& mlAsqi,int t)
      {
	const Eigen::MatrixXd Asq = inertiaSqrootSIN(t);
	const long int n = Asq.rows();
	mlAsqi.resize(n,n);
	mlAsqi = Asq.triangularView<Eigen::Upper>().solve( Eigen::MatrixXd::Identity(n,n));

	/* Asqi is such that Asq^-1 = Asqi and Asqi Asqi^T = A^-1. Asqi is upper triangular. */
	return mlAsqi;
      }

      /* This function compute all the input matrix and vector signals that
       * will be needed by the solver. The main objective of this precomputation
       * is to get the size of all the elements. */
      int& SolverDynReduced::
      precomputeSOUT_function( int& dummy, int t )
      {
	//if( t==1000 )
	//dynamicgraph::sot::DebugTrace::openFile("/tmp/dynred1000.txt");

	/* Precompute the dynamic data. */
	inertiaSqrootInvSIN.recompute(t);
	inertiaSqrootSIN.recompute(t);

	/* Precompute the contact data. */
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    contact.jacobianSIN->recompute(t);
	    contact.JdotSIN->recompute(t);
	    contact.supportSIN->recompute(t);
	    contact.correctorSIN->recompute(t);
	  }

	/* Precompute the tasks data. */
	for( int i=0;i<(int)stack.size();++i )
	  {
	    stack[i]->taskSOUT.recompute(t);
	    stack[i]->jacobianSOUT.recompute(t);
	  }

	return dummy;
      }

      /* --- SIZES ---------------------------------------------------------- */
      /* --- SIZES ---------------------------------------------------------- */
      /* --- SIZES ---------------------------------------------------------- */
      void SolverDynReduced::computeSizesForce( int /* time */ )
      {
      }

      int& SolverDynReduced::
      sizeForceSpatialSOUT_function( int& nf, int t )
      {
	precomputeSOUT(t);

	int nbb=0;
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    contact.position = nbb++;
	  }
	nf=nbb*6;

	return nf;
      }
      int& SolverDynReduced::
      sizeForcePointSOUT_function( int& nf, int t )
      {
	precomputeSOUT(t);

	nf=0;
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    const int nfi = (int) (*contact.supportSIN)(t).cols()*3;
	    contact.range = std::make_pair(nf,nf+nfi);
	    nf+=nfi;
	  }
	return nf;
      }
 
      int& SolverDynReduced::
      sizeConfigurationSOUT_function( int& nq, int t )
      {
	nq = (int) velocitySIN(t).size();
	return nq;
      }

      int& SolverDynReduced::
      sizeMotionSOUT_function( int& nu, int t )
      {
	nu = (int) freeMotionBaseSOUT(t).cols();
	return nu;
      }

      int& SolverDynReduced::
      sizeActuationSOUT_function( int& nphi, int t )
      {
	nphi = (int) freeForceBaseSOUT(t).cols();
	return nphi;
      }

      /* --- FORCES MATRICES ------------------------------------------------ */
      /* --- FORCES MATRICES ------------------------------------------------ */
      /* --- FORCES MATRICES ------------------------------------------------ */
      /* Compute the Jacobian of the contact bodies, along with the drift. */
      dg::Matrix& SolverDynReduced::
      JcSOUT_function( dg::Matrix& mlJ,int t )
      {
	using namespace Eigen;

	const Eigen::VectorXd qdot = velocitySIN(t);
	const int &nq= sizeConfigurationSOUT(t),
	  nphi = sizeForceSpatialSOUT(t);
	mlJ.resize(nphi,nq);
	forceDrift.resize(nphi);

	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    const Eigen::MatrixXd Ji = (*contact.jacobianSIN)(t);
	    const Eigen::MatrixXd Jdoti = (*contact.JdotSIN)(t);
	    const Eigen::VectorXd corrector = (*contact.correctorSIN)(t);
	    const int r = 6*contact.position;
	    assert( Ji.rows()==6 && Ji.cols()==nq );
	    assert( r+6<=mlJ.rows() && r>=0 );

	    mlJ.ROWS(r,r+6) = Ji;
	    forceDrift.ROWS(r,r+6) = corrector - Jdoti*qdot;
	  }

	return mlJ;
      }

      /* Compute the matrix X such that Aqddot + Jc'*X'*fc = tau.
       * Xc' is the matrix that pass from the ponctual forces to
       * the 6D torques expressed at the body center.
       * X has a diagonal-block structure, that is not preserve by the
       * current data structure.
       */
      dg::Matrix& SolverDynReduced::
      forceGeneratorSOUT_function( dg::Matrix& mlX,int t )
      {
	using namespace Eigen;

	const int& nf = sizeForcePointSOUT(t), nphi = sizeForceSpatialSOUT(t);
	mlX.resize(nf,nphi);

	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    const Eigen::MatrixXd support = (*contact.supportSIN)(t);
	    const int n0 = contact.range.first, n1 = contact.range.second,
	      r=6*contact.position, nbp = (int) support.cols();
	    assert( ((n1-n0) % 3 == 0) && nbp == (n1-n0)/3);

	    for( int i=0;i<nbp;++i )
	      {
		assert( n0+3*(i+1)<=mlX.rows() && r+6<=mlX.cols() );
		mlX.block(n0+3*i,r, 3,3) = Matrix3d::Identity();
		Block<Eigen::MatrixXd> px = mlX.block(n0+3*i,r+3, 3,3);
		sotSolverDyn::preCross(-support.col(i),px);
	      }
	  }

	return mlX;
      }

      dg::Matrix& SolverDynReduced::
      freeMotionBaseSOUT_function( dg::Matrix& mlV,int t )
      {
	using namespace Eigen;
	const Eigen::MatrixXd B = inertiaSqrootInvSIN(t);
	const Eigen::MatrixXd J = JcSOUT(t);
	const int & nq = sizeConfigurationSOUT(t);
	assert( J.cols()==nq && B.rows() == nq );

	MatrixXd Gt = (J*B.triangularView<Eigen::Upper>()).transpose();
	sotDEBUG(40) << "Gt = " << (soth::MATLAB)Gt << std::endl;
	Gt_qr.compute( Gt );
	Gt_qr.setThreshold(1e-3);
	G_rank = Gt_qr.rank();
	const unsigned int freeRank = nq-G_rank;
	sotDEBUG(40) << "Q = " << (soth::MATLAB)(MatrixXd)Gt_qr.householderQ()  << std::endl;

	/*
	  J = [ L 0 ] Q' = [L 0] [ V_perp' ; V' ]
	  J' = Q [ R; 0 ] = [ V_perp V ] [ R; 0 ]
	  V = Q [ 0 ; I ].
	 */
	mlV.resize(nq,freeRank);
	assert( G_rank == J.rows() );
	assert( Gt_qr.householderQ().cols()==nq && Gt_qr.householderQ().rows()==nq );
	mlV.topRows(nq-freeRank).setZero();
	mlV.bottomRows(freeRank) = MatrixXd::Identity(freeRank,freeRank);
	mlV.applyOnTheLeft( Gt_qr.householderQ() );

	return mlV;
      }

      dg::Matrix& SolverDynReduced::
      freeForceBaseSOUT_function( dg::Matrix& mlK,int t )
      {
	using namespace Eigen;
	using soth::MATLAB;
	using std::endl;
	const Eigen::MatrixXd X = forceGeneratorSOUT(t);
	const Eigen::MatrixXd Jc = JcSOUT(t);
	const int & nf = sizeForcePointSOUT(t);
	assert( X.rows()==nf );

	/* J[:,1:6] = [ R_1  X_1 ; ... ; R_N X_N ]
	 * with R_i the rotation of the frame where contact point i is expressed
	 * and X_i the skew of the vector from waist to contact point. */
	X_qr.compute( X*(Jc.leftCols(6)) );
	X_qr.setThreshold(1e-3);
	const unsigned int freeRank = nf-X_qr.rank();
	assert( X_qr.rank()==6 );

	sotDEBUG(5) << "JSb = " << (MATLAB)( (MatrixXd)((X*Jc).leftCols(6)) ) << endl;
	sotDEBUG(5) << "Q = " << (MATLAB)(MatrixXd)X_qr.householderQ()  << endl;

	mlK.resize(nf,freeRank);
	mlK.topRows(X_qr.rank()).setZero();
	mlK.bottomRows(freeRank) = MatrixXd::Identity(freeRank,freeRank);
	mlK.applyOnTheLeft( X_qr.householderQ() );

	return mlK;
      }

      void SolverDynReduced::
      resizeSolver( void )
      {
	sotDEBUGIN(15);
	const int & npsi = sizeActuationSOUT.accessCopy(),
	  & nf = sizeForcePointSOUT.accessCopy(),
	  & nu = sizeMotionSOUT.accessCopy(),
	  & nq = sizeConfigurationSOUT.accessCopy(),
	  nbTasks = stack.size(),
	  nx = npsi+nu;

	bool toBeResize = hsolver==NULL
	  || (nu+npsi)!=(int)hsolver->sizeProblem
	  || stack.size()+2!= hsolver->nbStages();

	/* Resize the force level. */
	assert( (nf%3)==0 );
	if( Cforce.rows()!=nf/3 || Cforce.cols()!=nx || bforce.size()!=nf/3)
	  {
	    Cforce.resize(nf/3,nx);
	    bforce.resize(nf/3);
	    toBeResize = true;
	  }

	/* Resize the task levels. */
	if( Ctasks.size()!=nbTasks || btasks.size()!=nbTasks )
	  {
	    Ctasks.resize(nbTasks);
	    btasks.resize(nbTasks);
	  }
	for( int i=0;i<(int)stack.size();++i )
	  {
	    const int ntask = stack[i]->taskSOUT.accessCopy().size();
	    if( ntask != btasks[i].size()
		|| ntask != Ctasks[i].rows()
		|| nx != Ctasks[i].cols() )
	      {
		Ctasks[i].resize( ntask,nx );
		btasks[i].resize( ntask );
		toBeResize = true;
	      }
	  }

	/* Resize the final level. */
	if( Czero.cols()!=nx || Czero.rows()!=nq-6 || bzero.size()!=nq-6 )
	  {
	    Czero.resize(nq-6,nx);
	    bzero.resize(nq-6);
	    toBeResize = true;
	  }

	/* Rebuild the solver. */
	if( toBeResize )
	  {
	    sotDEBUG(1) << "Resize all." << std::endl;
	    hsolver = hcod_ptr_t(new soth::HCOD(nx,nbTasks+2));

	    hsolver->pushBackStage( Cforce,    bforce );
	    hsolver->stages.back()->name = "force";

	    for( int i=0;i<(int)stack.size();++i )
	      {
		hsolver->pushBackStage( Ctasks[i],    btasks[i] );
		hsolver->stages.back()->name = stack[i]->getName();
	      }

	    hsolver->pushBackStage( Czero,    bzero );
	    hsolver->stages.back()->name = "zero";

	    solution.resize( nx );
	  }
      }

      /* The drift is equal to: d = Gc^+ ( xcddot - Jcdot qdot ). */
      dg::Vector& SolverDynReduced::
      driftContactSOUT_function( dg::Vector &mlres, int t )
      {
	/* BV has already been computed, but I don't know if it is the best
	 * idea to go for it a second time. This suppose that the matrix has
	 * not been modified in between. It should work, but start with that if
	 * you are looking for a bug in ddq. */
	/* Same for Gt_qr. */
	using soth::MATLAB;
	using namespace sotSolverDyn;
	using namespace Eigen;

	const int nq = sizeConfigurationSOUT(t);
	JcSOUT(t); // To force the computation of forceDrift.
	freeMotionBaseSOUT(t); // To force the computation of G_svd.
	mlres.resize(nq);
	//EIGEN_VECTOR_FROM_VECTOR(res,mlres,nq);
	sotDEBUG(40) << "fdrift = " << (MATLAB)forceDrift << std::endl;

	const int nphi = sizeForceSpatialSOUT(t);
	mlres.head(nphi) = forceDrift; mlres.tail( nq-nphi ).setZero();
	Gt_qr.solveTransposeInPlace( mlres );
	sotDEBUG(40) << "drift = " << (MATLAB)mlres << std::endl;

	return mlres;
      }
      dg::Vector& SolverDynReduced::
      solutionSOUT_function( dg::Vector &mlres, int t )
      {
	sotDEBUG(15) << " # In time = " << t << std::endl;
	using namespace soth;
	using namespace Eigen;
	using namespace sotSolverDyn;

	precomputeSOUT(t);
	const Eigen::MatrixXd sB = inertiaSqrootInvSIN(t);
	const Eigen::MatrixXd sBi = inertiaSqrootSIN(t);
	const Eigen::MatrixXd V = freeMotionBaseSOUT(t);
	const Eigen::MatrixXd K = freeForceBaseSOUT(t);
	const Eigen::MatrixXd Jc = JcSOUT(t);
	const Eigen::MatrixXd Xc = forceGeneratorSOUT(t);
	const Eigen::VectorXd b = dyndriftSIN(t);
	const Eigen::VectorXd qdot = velocitySIN(t);
	const Eigen::VectorXd drift = driftContactSOUT(t);
	Eigen::TriangularView<const Eigen::MatrixXd,Eigen::Upper> B(sB);
	Eigen::TriangularView<const Eigen::MatrixXd,Eigen::Upper> Bi(sBi);

	const int & nf = sizeForcePointSOUT(t), &nphi = sizeForceSpatialSOUT(t),
	  & npsi = sizeActuationSOUT(t), & nq = sizeConfigurationSOUT(t),
	  & nu = sizeMotionSOUT(t), nbForce = nf/3, nx=npsi+nu;
	resizeSolver();

	assert( (nf%3)==0 );

	sotDEBUG(1) << "b = " << (MATLAB)b << std::endl;
	sotDEBUG(1) << "B = " << (MATLAB)sB << std::endl;
	sotDEBUG(1) << "Bi = " << (MATLAB)sBi << std::endl;
	sotDEBUG(1) << "V = " << (MATLAB)V << std::endl;
	sotDEBUG(1) << "K = " << (MATLAB)K << std::endl;
	sotDEBUG(1) << "Jc = " << (MATLAB)Jc << std::endl;
	sotDEBUG(1) << "Xc = " << (MATLAB)Xc << std::endl;

	if( dampingSIN ) //damp?
	  {
	    sotDEBUG(5) << "Using damping. " << std::endl;
	    hsolver->setDamping( 0 );
	    hsolver->useDamp( true );
	    hsolver->stages.back()->damping( dampingSIN(t) );
	  }
	else
	  {
	    sotDEBUG(5) << "Without damping. " << std::endl;
	    hsolver->useDamp( false );
	  }

	/* SOT:
	 * 1.b Sf f > 0
	 * 2... Ji B u = ddxi
	 * 3 ...


	 * 1.a [ S'*B^-T*V  Sb J' ] =  -Sb b
	 * 1.b [ 0          Sf    ] >= 0
	 * 2.. [ Ji*B       0     ] = xddi
	 */

#define COLS_U leftCols( nu )
#define COLS_F rightCols( npsi )
#define ROWS_FF topRows( 6 )
#define ROWS_ACT bottomRows( nq-6 )

	/* -1- */
	{
	  MatrixXd XJSptSBV2( nf,nu );
	  XJSptSBV2.ROWS_FF
	    = - (sBi.transpose().topLeftCorner(6,6).triangularView<Lower>() * V.ROWS_FF);
	  XJSptSBV2.bottomRows( nf-6 ).setZero();
	  sotDEBUG(15) << "SBitV = " << (MATLAB)XJSptSBV2 << std::endl;
	  X_qr.solveTransposeInPlace( XJSptSBV2 );
	  sotDEBUG(15) << "XJSptSBitV = " << (MATLAB)XJSptSBV2 << std::endl;

	  VectorXd XJSptdelta( nf );
	  XJSptdelta.ROWS_FF
	    = b.ROWS_FF
	    + sBi.transpose().topLeftCorner(6,6).triangularView<Lower>()*drift.ROWS_FF;
	  XJSptdelta.bottomRows( nf-6 ).setZero();
	  sotDEBUG(15) << "SBitdelta = " << (MATLAB)XJSptdelta << std::endl;
	  X_qr.solveTransposeInPlace( XJSptdelta );
	  sotDEBUG(15) << "XJSptSBitdelta = " << (MATLAB)XJSptdelta << std::endl;

	  assert( XJSptSBV2.rows()==nbForce*3 && K.rows()==nbForce*3
		  && XJSptdelta.size()==nbForce*3 );
	  for( int i=0;i<nbForce;++i )
	    {
	      Cforce.COLS_U.row(i) = XJSptSBV2.row(3*i+2);
	      Cforce.COLS_F.row(i) = K.row(3*i+2);
	      bforce[i] = Bound( XJSptdelta[3*i+2], Bound::BOUND_SUP );
	    }
	}
	sotDEBUG(15) << "Cforce = "     << (MATLAB)Cforce << std::endl;
	sotDEBUG(1) << "bforce = "     << bforce << std::endl;

	/* The matrix B*V has to be stored to avoid unecessary
	 * recomputation. */
	BV = B*V; // TODO: compute B*drift the same way.
	sotDEBUG(15) << "BV = "     << (MATLAB)BV << std::endl;
	sotDEBUG(15) << "B = "     << (MATLAB)(MatrixXd)B << std::endl;

	/* -2- */
	for( int i=0;i<(int)stack.size();++i )
	  {
	    TaskDynPD & task = * stack[i];
	    MatrixXd & Ctask1 = Ctasks[i];
	    VectorBound & btask1 = btasks[i];

	    const Eigen::MatrixXd Jdot = task.JdotSOUT(t);
	    const Eigen::MatrixXd J = task.jacobianSOUT(t);
	    const dg::sot::VectorMultiBound & ddx = task.taskSOUT(t);

	    sotDEBUG(5) << "ddx"<<i<<" = " << ddx << std::endl;
	    sotDEBUG(25) << "J"<<i<<" = " << (MATLAB)J << std::endl;
	    sotDEBUG(45) << "Jdot"<<i<<" = " << (MATLAB)Jdot << std::endl;

	    assert( Ctask1.rows() == ddx.size() && btask1.size() == ddx.size() );
	    assert( J.rows()==ddx.size() && J.cols()==nq && (int)ddx.size()==ddx.size() );
	    assert( Jdot.rows()==ddx.size() && Jdot.cols()==nq );

	    Ctask1.COLS_U = J*BV;	    Ctask1.COLS_F.fill(0);
	    VectorXd Jdqd = Jdot*qdot;
	    vbAssign(btask1,ddx);
	    vbSubstract(btask1,Jdqd);
	    sotDEBUG(45) << "Jdqd"<<i<<" = " << (MATLAB)Jdqd << std::endl;
	    sotDEBUG(45) << "JBdc"<<i<<" = " << (MATLAB)(MatrixXd)(J*B*drift) << std::endl;
	    vbSubstract(btask1,(VectorXd)(J*B*drift));

	    /* TODO: account for the contact drift. */

	    sotDEBUG(45) << "Ctask"<<i<<" = " << (MATLAB)Ctask1 << std::endl;
	    sotDEBUG(45) << "btask"<<i<<" = " << btask1 << std::endl;
	  }

	/* -3- */
	/* Czero = [ BV 0 ] */
	assert( Czero.cols() == nx && Czero.rows()==nq-6 && bzero.size() ==nq-6 );
	assert( nbDofs+6 == nq );
	Czero.COLS_U = BV.ROWS_ACT;
	Czero.COLS_F.setZero();
	const double & Kv = breakFactorSIN(t);
	const Eigen::VectorXd dq = velocitySIN(t);
	if( postureSIN && positionSIN )
	   {
	     const Eigen::VectorXd qref = postureSIN(t);
	     const Eigen::VectorXd q = positionSIN(t);
	     const double Kp = .25*Kv*Kv;
	     vbAssign( bzero,(-Kp*(q-qref)-Kv*dq).tail(nbDofs) );
	   }
	else
	  {	vbAssign(bzero,(-Kv*dq).tail(nbDofs));	  }
	/* TODO: account for the contact drift. */
	vbSubstract(bzero,((VectorXd)(B*drift)).ROWS_ACT  );
	sotDEBUG(15) << "Czero = "     << (MATLAB)Czero << std::endl;
	sotDEBUG(1) << "bzero = "     << bzero << std::endl;

	/* Run solver */
	/* --- */
	sotDEBUG(1) << "Initial config." << std::endl;
	hsolver->reset();
	hsolver->setInitialActiveSet();
	sotDEBUG(1) << "Run for a solution." << std::endl;
	hsolver->activeSearch(solution);
	sotDEBUG(1) << "solution = " << (MATLAB)solution << std::endl;
	mlres.resize(nx);
	mlres=solution;

	// Small verif:
	if( Ctasks.size()>0 )
	  {
	    sotDEBUG(1) << "ddx0 = " << (MATLAB)(VectorXd)(Ctasks[0]*solution) << std::endl;
	    sotDEBUG(1) << "ddx0 = " << btasks[0] << std::endl;
	  }

	sotDEBUGOUT(15);
	return mlres;
      }

      dg::Vector& SolverDynReduced::
      reducedControlSOUT_function( dg::Vector& res,int t )
      {
	const Eigen::VectorXd x = solutionSOUT(t);
	const int & nu = sizeMotionSOUT(t);
	res.resize(nu);
	res = x.head(nu);

	return res;
      }
      dg::Vector& SolverDynReduced::
      reducedForceSOUT_function( dg::Vector& res,int t )
      {
	const Eigen::VectorXd x = solutionSOUT(t);
	const int & npsi = sizeActuationSOUT(t);
	res.resize(npsi);
	res = x.tail(npsi);

	return res;
      }
      dg::Vector& SolverDynReduced::
      accelerationSOUT_function( dg::Vector& mlddq,int t )
      {
	const int & nq = sizeConfigurationSOUT(t);
	const Eigen::VectorXd u = reducedControlSOUT(t);
	mlddq.resize(nq);
	/* BV has already been computed, but I don't know if it is the best
	 * idea to go for it a second time. This suppose that the matrix has
	 * not been modified in between. It should work, but start with that if
	 * you are looking for a bug in ddq. */

	using soth::MATLAB;
	using namespace sotSolverDyn;
	using namespace Eigen;

	const Eigen::VectorXd drift = driftContactSOUT(t);
	const Eigen::MatrixXd sB = inertiaSqrootInvSIN(t);
	Eigen::TriangularView<const Eigen::MatrixXd,Eigen::Upper> B(sB);
	sotDEBUG(40) << "drift = " << (MATLAB)drift << std::endl;
	sotDEBUG(40) << "BV = " << (MATLAB)BV << std::endl;
	sotDEBUG(40) << "B = " << (MATLAB)sB << std::endl;
	sotDEBUG(40) << "u = " << (MATLAB)u << std::endl;
	mlddq = BV*u + B*drift;

	/* --- verif --- */
	{
	  const Eigen::MatrixXd Jc = JcSOUT(t);
	  sotDEBUG(40) << "fdrift = " << (MATLAB)forceDrift << std::endl;
	  sotDEBUG(40) << "Jqdd = " << (MATLAB)(MatrixXd)(Jc*mlddq)  << std::endl;
	  sotDEBUG(40) << "diff = " << (MATLAB)(MatrixXd)(Jc*mlddq-forceDrift)  << std::endl;
	  sotDEBUG(40) << "ndiff = " << (Jc*mlddq-forceDrift).norm() << std::endl;
	}

	return mlddq;
      }
      dg::Vector& SolverDynReduced::
      forcesSOUT_function( dg::Vector& res,int t )
      {
	using namespace Eigen;
	using namespace soth;
	const Eigen::MatrixXd sB = inertiaSqrootInvSIN(t);
	const Eigen::MatrixXd sBi = inertiaSqrootSIN(t);
	const Eigen::MatrixXd V = freeMotionBaseSOUT(t);
	const Eigen::MatrixXd K = freeForceBaseSOUT(t);
	const Eigen::MatrixXd Jc = JcSOUT(t);
	const Eigen::MatrixXd Xc = forceGeneratorSOUT(t);
	const Eigen::VectorXd b = dyndriftSIN(t);
	const Eigen::VectorXd drift = driftContactSOUT(t);
	Eigen::TriangularView<const Eigen::MatrixXd,Eigen::Upper> B(sB);
	Eigen::TriangularView<const Eigen::MatrixXd,Eigen::Upper> Bi(sBi);
	const Eigen::VectorXd u = reducedControlSOUT(t);
	const Eigen::VectorXd psi = reducedForceSOUT(t);

	const int & nphi = sizeForceSpatialSOUT(t),
	  & nf = sizeForcePointSOUT(t);
	res.resize(nf);

	res.ROWS_FF	// = Sb( B^-T( -Vu-delta ) - b )
	  = sBi.transpose().topLeftCorner(6,6).triangularView<Lower>()
	  * ( -V.ROWS_FF*u - drift.ROWS_FF )
	  - b.ROWS_FF;
	res.tail( nf-6 ).setZero();
	sotDEBUG(15) << "SBVu = "     << (MATLAB)res << std::endl;

	X_qr.solveTransposeInPlace(res);
	sotDEBUG(15) << "SJptSBVu = "     << (MATLAB)res << std::endl;

	res += K*psi;
	sotDEBUG(15) << "res = "     << (MATLAB)res << std::endl;

	/* phi = X' f */
	Eigen::VectorXd phi = Xc.transpose()*res;
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    const int r = 6*contact.position;
	    dg::Vector mlphii;
	    mlphii.resize(6);
	    assert( r+6<=phi.size() );

	    mlphii = phi.segment<6>(r);
	    (*contact.forceSOUT) = mlphii;
	  }

	return res;
      }
      dg::Vector& SolverDynReduced::
      forcesNormalSOUT_function( dg::Vector& res,int t )
      {
	using namespace Eigen;
	using namespace soth;
	const Eigen::VectorXd solution = solutionSOUT(t);
	const int & nfn = Cforce.rows();
	res.resize(nfn);
	res = Cforce*solution;

	for( int i=0;i<nfn;++i )
	  {
	    res[i] -= bforce[i].getBound( bforce[i].getType() );
	  }

	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    const int nf0 = contact.range.first / 3;
	    const int nff = contact.range.second / 3;
	    assert( (contact.range.first%3) == 0);
	    assert( (contact.range.second%3) == 0);
	    assert( nff<=res.size() );

	    dg::Vector mlfni;
	    mlfni.resize(nff-nf0 );
	    mlfni = res.segment(nf0,nff-nf0);
	    (*contact.fnSOUT) = mlfni;
	  }


	return res;
      }
      dg::Vector& SolverDynReduced::
      activeForcesSOUT_function( dg::Vector& res,int t )
      {
	using namespace soth;
	const Eigen::VectorXd solution = solutionSOUT(t);
	Stage & stf = *hsolver->stages.front();
	//TODO: CHECK! Output!!
	res.conservativeResize(stf.sizeA());

	Eigen::VectorXd atmp(stf.nbConstraints()); 
	atmp=stf.eactive(atmp);

	return res;
      }
      dg::Vector& SolverDynReduced::
      torqueSOUT_function( dg::Vector& res,int ) { return res; }

      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */

      void SolverDynReduced::
      display( std::ostream& os ) const
      {
	os << "SolverDynReduced "<<getName() << ": " << nbDofs <<" joints." << std::endl;
	try{
	  os <<"solution = "<< solutionSOUT.accessCopy() <<std::endl << std::endl;
	}  catch (dynamicgraph::ExceptionSignal e) {}
	stack_t::display(os);
	dispContacts(os);
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

