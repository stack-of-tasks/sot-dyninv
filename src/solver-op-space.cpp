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
public:solver_op_space__INIT( void ) { dynamicgraph::sot::DebugTrace::openFile(); }
};
solver_op_space__INIT solver_op_space_initiator;
#endif //#ifdef VP_DEBUG


#include <sot-dyninv/solver-op-space.h>
#include <sot-dyninv/commands-helper.h>
#include <dynamic-graph/factory.h>
#include <boost/foreach.hpp>
#include <sot-dyninv/commands-helper.h>
#include <dynamic-graph/pool.h>
#include <sot-dyninv/mal-to-eigen.h>
#include <soth/HCOD.hpp>
#include <sot-dyninv/mal-to-eigen.h>
#include <sot-dyninv/task-dyn-pd.h>
#include <sot/core/feature-point6d.hh>
#include <sstream>
#include <soth/Algebra.hpp>
#include <Eigen/QR>


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
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SolverOpSpace,"SolverOpSpace");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      SolverOpSpace::
      SolverOpSpace( const std::string & name )
	: Entity(name)
	,stack_t()

	,CONSTRUCT_SIGNAL_IN(matrixInertia,ml::Matrix)
	,CONSTRUCT_SIGNAL_IN(velocity,ml::Vector)
	,CONSTRUCT_SIGNAL_IN(dyndrift,ml::Vector)
	,CONSTRUCT_SIGNAL_IN(damping,double)
	,CONSTRUCT_SIGNAL_IN(breakFactor,double)
	,CONSTRUCT_SIGNAL_IN(posture,ml::Vector)
	,CONSTRUCT_SIGNAL_IN(position,ml::Vector)

	,CONSTRUCT_SIGNAL_OUT(control,ml::Vector,
			      matrixInertiaSIN << dyndriftSIN
			      << velocitySIN )
	,CONSTRUCT_SIGNAL(zmp,OUT,ml::Vector)
	,CONSTRUCT_SIGNAL(acceleration,OUT,ml::Vector)

	,nbParam(0), nq(0),ntau(0),nfs(0)
	,hsolver()

	,Cdyn(),Ccontact(),Czmp(),Czero()
	,bdyn(),bcontact(),bzmp(),bzero()
	,Ctasks(),btasks()
	,solution()
      {
	signalRegistration( matrixInertiaSIN << dyndriftSIN
			    << velocitySIN << controlSOUT
			    << zmpSOUT << accelerationSOUT
			    << dampingSIN << breakFactorSIN
			    << postureSIN << positionSIN );

	/* Command registration. */
	boost::function<void(SolverOpSpace*,const std::string&)> f_addContact
	  =
	  boost::bind( &SolverOpSpace::addContact,_1,_2,
		       (dynamicgraph::Signal<ml::Matrix, int>*)NULL,
		       (dynamicgraph::Signal<ml::Matrix, int>*)NULL,
		       (dynamicgraph::Signal<ml::Vector, int>*)NULL,
		       (dynamicgraph::Signal<ml::Matrix, int>*)NULL);
	addCommand("add.Contact",
		   makeCommandVoid1(*this,f_addContact,
				    docCommandVoid1("create the contact signals, unpluged.",
						    "string")));
	addCommand("addContactFromTask",
		   makeCommandVoid2(*this,&SolverOpSpace::addContactFromTask,
				    docCommandVoid2("Add a contact from the named task. Remmeber to plug __p.",
						    "string(task name)","string (contact name)")));
	addCommand("rmContact",
		   makeCommandVoid1(*this,&SolverOpSpace::removeContact,
				    docCommandVoid1("remove the contact named in arguments.",
						    "string")));
	addCommand("dispContacts",
		   makeCommandVerbose(*this,&SolverOpSpace::dispContacts,
				      docCommandVerbose("Guess what?")));
	addCommand("debugOnce",
		   makeCommandVoid0(*this,&SolverOpSpace::debugOnce,
				    docCommandVoid0("open trace-file for next iteration of the solver.")));

	ADD_COMMANDS_FOR_THE_STACK;
      }

      /* --- STACK ----------------------------------------------------------- */
      /* --- STACK ----------------------------------------------------------- */
      /* --- STACK ----------------------------------------------------------- */

      SolverOpSpace::TaskDependancyList_t SolverOpSpace::
      getTaskDependancyList( const TaskDynPD& task )
      {
	TaskDependancyList_t res;
	res.push_back( &task.taskSOUT );
	res.push_back( &task.jacobianSOUT );
	res.push_back( &task.JdotSOUT );
	return res;
      }
      void SolverOpSpace::
      addDependancy( const TaskDependancyList_t& depList )
      {
	BOOST_FOREACH( const SignalBase<int>* sig, depList )
	  { controlSOUT.addDependency( *sig ); }
      }
      void  SolverOpSpace::
      removeDependancy( const TaskDependancyList_t& depList )
      {
	BOOST_FOREACH( const SignalBase<int>* sig, depList )
	  { controlSOUT.removeDependency( *sig ); }
      }
      void SolverOpSpace::
      resetReady( void )
      {
	controlSOUT.setReady();
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

      void SolverOpSpace::
      addContactFromTask( const std::string & taskName,const std::string & contactName )
      {
	using namespace dynamicgraph::sot;

	TaskDynPD & task = dynamic_cast<TaskDynPD&> ( g_pool.getEntity( taskName ) );
	assert( task.getFeatureList().size() == 1 );
	BOOST_FOREACH( FeatureAbstract* fptr, task.getFeatureList() )
	  {
	    FeaturePoint6d* f6 = dynamic_cast< FeaturePoint6d* >( fptr );
	    assert( NULL!=f6 );
	    f6->positionSIN.recompute(controlSOUT.getTime());
	    f6->servoCurrentPosition();
	    f6->FeatureAbstract::selectionSIN = true;
	  }
	addContact( contactName, &task.jacobianSOUT, &task.JdotSOUT,&task.taskVectorSOUT, NULL );
      }

      void SolverOpSpace::
      addContact( const std::string & name,
		  Signal<ml::Matrix,int> * jacobianSignal,
		  Signal<ml::Matrix,int> * JdotSignal,
		  Signal<ml::Vector,int> * corrSignal,
		  Signal<ml::Matrix,int> * contactPointsSignal )
      {
	if( contactMap.find(name) != contactMap.end())
	  {
	    std::cerr << "!! Contact " << name << " already exists." << std::endl;
	    return;
	  }

	contactMap[name].jacobianSIN
	  = matrixSINPtr( new SignalPtr<ml::Matrix,int>
			  ( jacobianSignal,
			    "sotDynInvWB("+getName()+")::input(matrix)::_"+name+"_J" ) );
	signalRegistration( *contactMap[name].jacobianSIN );

	contactMap[name].JdotSIN
	  = matrixSINPtr( new SignalPtr<ml::Matrix,int>
			  ( JdotSignal,
			    "sotDynInvWB("+getName()+")::input(matrix)::_"+name+"_Jdot" ) );
	signalRegistration( *contactMap[name].JdotSIN );

	contactMap[name].supportSIN
	  = matrixSINPtr( new SignalPtr<ml::Matrix,int>
			  ( contactPointsSignal,
			    "sotDynInvWB("+getName()+")::input(matrix)::_"+name+"_p" ) );
	signalRegistration( *contactMap[name].supportSIN );

	contactMap[name].correctorSIN
	  = vectorSINPtr( new SignalPtr<ml::Vector,int>
			  ( corrSignal,
			    "sotDynInvWB("+getName()+")::input(vector)::_"+name+"_x" ) );
	signalRegistration( *contactMap[name].correctorSIN );

	contactMap[name].forceSOUT
	  = vectorSOUTPtr( new Signal<ml::Vector,int>
			   ( "sotDynInvWB("+getName()+")::output(vector)::_"+name+"_f" ) );
	signalRegistration( *contactMap[name].forceSOUT );

	contactMap[name].fnSOUT
	  = vectorSOUTPtr( new Signal<ml::Vector,int>
			   ( "sotDynInvWB("+getName()+")::output(vector)::_"+name+"_fn" ) );
	signalRegistration( *contactMap[name].fnSOUT );

      }

      void SolverOpSpace::
      removeContact( const std::string & name )
      {
	if( contactMap.find(name) == contactMap.end() )
	  {
	    std::cerr << "!! Contact " << name << " does not exist." << std::endl;
	    return;
	  }

	signalDeregistration( signalShortName(contactMap[name].jacobianSIN->getName()) );
	signalDeregistration( signalShortName(contactMap[name].supportSIN->getName()) );
	signalDeregistration( signalShortName(contactMap[name].forceSOUT->getName()) );
	signalDeregistration( signalShortName(contactMap[name].fnSOUT->getName()) );
	signalDeregistration( signalShortName(contactMap[name].JdotSIN->getName()) );
	signalDeregistration( signalShortName(contactMap[name].correctorSIN->getName()) );
	contactMap.erase(name);
      }

      void SolverOpSpace::
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

      /* --------------------------------------------------------------------- */
      /* --- STATIC INTERNAL ------------------------------------------------- */
      /* --------------------------------------------------------------------- */
      namespace sotOPH
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

	template< typename D1, typename D2  >
	void computeForceNormalConversion( Eigen::MatrixBase<D1> & Ci,
					   Eigen::MatrixBase<D2> & positions )
	{
	  /* General Constraint is: phi^0 = Psi.fi, with Psi = [ I; skew(OP1);
	     skew(OP2); skew(OP3); skew(OP4) ].  But phi^0 = X_c^0*phi^c (both feet
	     are 0.105cm above the ground so X_c^0 is the transformation matri from
	     actual ankle force to the actual contact force).  * X_c^0*phi^c - Psi*fi
	     = 0 Since phi_{x,y,Rz}^0 ~ fi_{x,y} and phi_{z,Rx,Ry} ~ fi_{z,Rx,Ry}: *
	     Reduced Constraint Ci is applied as:[ X_c^0*phi^c - Psi*fi ]_{z,Rx,Ry} =
	     0 [ C_phi -C_fi ]_{z,Rx,Ry}*[phi^c; fi_{z,Rx,Ry}] = 0, so Ci = [ C_phi
	     -C_fi ]_{z,Rx,Ry} with C_phi = X_c^0 and C_fi = Psi.
	  */

	  using namespace Eigen;

	  const int nbPoint = positions.cols();
	  assert( positions.rows()==3 );
	  assert( Ci.rows()==3 && Ci.cols()==6+nbPoint );

	  //Ci.leftCols(6) = -MatrixXd::Identity(6,6);
	  /* Ci = [ X_c^0_{z,Rx,Ry} -Psi_{z,Rx,Ry}];
	   *   with X_c^0_{z,Rx,Ry} = [ 0 0 1 0 0 0; 0 -z 0 1 0 0; z 0 0 0 1 0 ]
	   *   and Psi_{z,Rx,Ry} = [ 1 1 1 1; y_1 y_2 y_3 y_4; -x_1 -x_2 -x_3 -x_4 ]. */
	  const double z = positions(2,0);
	  Ci.setZero();
	  Ci(0,2)=1;
	  Ci(1,1)=-z; Ci(1,3)=1;
	  Ci(2,0)=+z; Ci(2,4)=1;

	  typename D1::ColsBlockXpr Phi = Ci.rightCols( nbPoint );
	  for( int i=0;i<nbPoint;++i )
	    {
	      const double x = positions(0,i);
	      const double y = positions(1,i);
	      Phi.col(i) << -1,-y,+x;
	    }
	  sotDEBUG(5) << "Psi = " << (soth::MATLAB) Phi << std::endl;
	}
      }


      /* --- INIT SOLVER ------------------------------------------------------ */
      /* --- INIT SOLVER ------------------------------------------------------ */
      /* --- INIT SOLVER ------------------------------------------------------ */

      /** Force the update of all the task in-signals, in order to fix their
       * size for resizing the solver.
       */
      void SolverOpSpace::
      refreshTaskTime( int time )
      {
	// TODO BOOST_FOREACH
	for( StackIterator_t iter=stack.begin();stack.end()!=iter;++iter )
	  {
	    TaskDynPD& task = **iter;
	    task.taskSOUT( time );
	  }
      }

      /** Knowing the sizes of all the stages (except the task ones),
       * the function resizes the matrix and vector of all stages (except...).
       */
      void SolverOpSpace::
      initialResizeSolver( void )
      {
	const int nbContactCst = 3*nbContactBodies+nbContactPoints;

	Cdyn.resize( nbDofs+6,nbParam );              bdyn.resize( nbDofs+6 );
	Ccontact.resize( 6*nbContactBodies,nbParam ); bcontact.resize( 6*nbContactBodies );
	Czmp.resize( nbContactCst,nbParam );          bzmp.resize( nbContactCst );
	Czero.resize( nbDofs,nbParam );               bzero.resize( nbDofs );
      }

      void SolverOpSpace::
      resizeSolver( void )
      {
	sotDEBUGIN(15);

	assert( nbDofs>0 );
	nq = nbDofs+6; ntau=nbDofs;
	nbContactBodies = contactMap.size();
	nbContactPoints = 0;
	int range = 0;
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact & contact = pContact.second;
	    const int nbi = contact.supportSIN->accessCopy().nbCols();
	    nbContactPoints += nbi;
	    int sizeVar = 6+nbi;
	    contact.range = std::make_pair( range,range+sizeVar );
	    range+=sizeVar;
	  }
	nfs=6*nbContactBodies+nbContactPoints;
	nbParam = nq+ntau+nfs;

	const int nbDynConstraint = 3;
	/* constraint [B]efore and [A]fter the tasks. */
	const int nbCstB = nbDynConstraint, nbCstA = 1;
	const int nbCst = nbCstB+stack.size()+nbCstA;

	bool toBeResized =
	  (hsolver==NULL
	   || (int)hsolver->nbStages()!=nbCst
	   || (int)hsolver->sizeProblem!=nbParam
	   // || (int)hsolver->stage(1).nbConstraints()!=nbContactBodies*6
	   // || (int)hsolver->stage(2).nbConstraints()!=3*nbContactBodies+nbContactPoints
	   );
	if(! toBeResized )
	  for( int i=0;i<(int)stack.size();++i )
	    if( stack[i]->taskSOUT.accessCopy().size()
		!=hsolver->stage(nbCstB+i).nbConstraints() )
	      { toBeResized = true; break; }

	if( toBeResized )
	  {
	    std::cout << "Resize all..." << std::endl;
	    sotDEBUG(1) << "Resize all." << std::endl;
	    hsolver = hcod_ptr_t(new soth::HCOD(nbParam,nbCst));
	    initialResizeSolver();

	    hsolver->pushBackStage( Cdyn,    bdyn );
	    hsolver->stages.back()->name = "dyn";
	    hsolver->pushBackStage( Ccontact,bcontact );
	    hsolver->stages.back()->name = "contact";
	    hsolver->pushBackStage( Czmp,    bzmp );
	    hsolver->stages.back()->name = "zmp";

	    Ctasks.clear(); Ctasks.resize( stack.size() );
	    btasks.clear(); btasks.resize( stack.size() );

	    for( int i=0;i<(int)stack.size();++i )
	      {
		TaskDynPD & task = *stack[i];
		const int nx = task.taskSOUT.accessCopy().size();
		Ctasks[i].resize(nx,nbParam);
		btasks[i].resize(nx);

		hsolver->pushBackStage( Ctasks[i],btasks[i] );
		hsolver->stages.back()->name = task.getName();
	      }

	    hsolver->pushBackStage( Czero, bzero );
	    hsolver->stages.back()->name = "zero";

	    solution.resize( nbDofs );
	  }

	sotDEBUGOUT(15);
      }


      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      ml::Vector& SolverOpSpace::
      controlSOUT_function( ml::Vector &control, int t )
      {
	sotDEBUG(15) << " # In time = " << t << std::endl;

	refreshTaskTime( t );
	resizeSolver();

	// if( t==1112 ) { hsolver->debugOnce(); }

	EIGEN_VECTOR_FROM_SIGNAL(b,dyndriftSIN(t));
	EIGEN_MATRIX_FROM_SIGNAL(A,matrixInertiaSIN(t));
	EIGEN_VECTOR_FROM_SIGNAL(dq,velocitySIN(t));

	using namespace sotOPH;
	using namespace soth;

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
	sotDEBUG(1) << "A = " << (MATLAB)A << std::endl;
	sotDEBUG(1) << "b = " << (MATLAB)b << std::endl;
	sotDEBUG(1) << "dq = " << (MATLAB)dq << std::endl;

	/* SOT:
	 * -1- A ddq + b + J'f = S' tau
	 * -2- J ddq = 0
	 * -3- Xp f  > eps
	 * -i- Ji qddot = ddxi - Jidot qdot
	 *
	 * -1- [ A -S' J' ] [ ddq; tau; f ] = -b
	 * -2- [ J  0  0  ] [ ddq; tau; f ] =  0
	 * -3- [ 0  0  Xp ] [ ddq; tau; f ] > EPS
	 * -3- [ Ji 0  0  ] [ ddq; tau; f ] = ddxi - Jidot qdot
	 */


#define COLS_Q leftCols( nq )
#define COLS_TAU leftCols( nq+ntau ).rightCols( ntau )
#define COLS_F rightCols( nfs )
#define ROWS_FF topRows( 6 )
#define ROWS_ACT bottomRows( nbDofs )

#define COLS(__ri,__rs) leftCols(__rs).rightCols((__rs)-(__ri))
#define ROWS(__ri,__rs) topRows(__rs).bottomRows((__rs)-(__ri))

	/* -1- */
	/* Cdyn = [ A 0 [0(6x30);-I(30x30)] J1.transpose 0 J2.transpose 0 ] */
	assert( Cdyn.rows() == A.rows() && bdyn.size() == A.rows() );
	Cdyn.COLS_Q = A;
	Cdyn.COLS_TAU.ROWS_FF.setZero();
	Cdyn.COLS_TAU.ROWS_ACT = -MatrixXd::Identity(nbDofs,nbDofs);
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact & contact = pContact.second;
	    EIGEN_MATRIX_FROM_SIGNAL(Jc,(*contact.jacobianSIN)(t));
	    const int ri = contact.range.first;
	    Cdyn.COLS_F.COLS(ri,ri+6) = Jc.transpose();
	  }

	for( int i=0;i<nbDofs+6;++i ) bdyn[i] = -b[i];
	sotDEBUG(15) << "Cdyn = "     << (MATLAB)Cdyn << std::endl;
	sotDEBUG(1) << "bdyn = "     << bdyn << std::endl;

	/* -2- */
	/* Ccontact = [ Jc1 0 0 0 0 0 ; Jc2 0 0 0 0 0 ]  */
	{
	  assert( Ccontact.rows() == nbContactBodies*6
		  && bcontact.size() == nbContactBodies*6 );
	  Ccontact.COLS_TAU.setZero();
	  Ccontact.COLS_F.setZero();
	  int nci = 0;
	  BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	    {
	      Contact& contact = pContact.second;  const int n6 = nci*6;
	      EIGEN_MATRIX_FROM_SIGNAL(Jc,(*contact.jacobianSIN)(t));
	      Ccontact.COLS_Q.ROWS(n6,n6+6) = Jc;

	      VectorXd reference = VectorXd::Zero(6);
	      if( (*contact.JdotSIN) )
		{
		  sotDEBUG(5) << "Accounting for Jcontact_dot. " << std::endl;
		  EIGEN_MATRIX_FROM_SIGNAL(Jcdot,(*contact.JdotSIN)(t));
		  reference -= Jcdot*dq;
		}
	      if( (*contact.correctorSIN) )
		{
		  sotDEBUG(5) << "Accounting for contact_xddot. " << std::endl;
		  EIGEN_VECTOR_FROM_SIGNAL(xdd,(*contact.correctorSIN)(t));
		  reference += xdd;
		}
	      for( int r=0;r<6;++r ) bcontact[n6+r] = reference[r];
	      nci++;
	    }
	  sotDEBUG(15) << "Ccontact = " << (MATLAB)Ccontact << std::endl;
	  sotDEBUG(1) << "bcontact = " << bcontact << std::endl;
	}

	/* -3- */
	/* Czmp = [ 0 0 -X_c^0_{z,Rx,Ry} Psi_{z,Rx,Ry} 0  0; 0 0 0 Iz 0 0 ] */
	{
	  Czmp.setZero();
	  int rows=0;
	  BOOST_FOREACH(const contacts_t::value_type& pContact, contactMap)
	    {
	      const Contact & contact = pContact.second;
	      EIGEN_MATRIX_FROM_SIGNAL(support,(*contact.supportSIN)(t));
	      const int nbP = support.cols();
	      const int ri = contact.range.first, rs=contact.range.second;

	      assert( nq+ntau+rs<=Czmp.cols() && rows+3+nbP<=Czmp.rows() );
	      assert( bzmp.size() == Czmp.rows() );

	      MatrixXd::ColsBlockXpr::ColsBlockXpr::ColsBlockXpr
		::RowsBlockXpr::RowsBlockXpr    Czi
		= Czmp.COLS_F.COLS(ri,rs). ROWS(rows,rows+3);
	      computeForceNormalConversion(Czi,support);
	      for( int i=0;i<3;++i ) bzmp. ROWS(rows,rows+3)[i] = 0;

	      for( int p=0;p<nbP;++p )
		{
		  Czmp.COLS_F.COLS(ri+6,rs).ROWS(rows+3,rows+3+nbP)
		    (p,p) = 1;
		  bzmp.ROWS(rows+3,rows+3+nbP)
		    [p] = soth::Bound(0,soth::Bound::BOUND_SUP);
		}
	      rows += 3+nbP;
	    }
	  sotDEBUG(15) << "Czmp = "     << (MATLAB)Czmp << std::endl;
	  sotDEBUG(1) << "bzmp = "     << bzmp << std::endl;
	}

	/* -Tasks 1:n- */
	/* Ctaski = [ Ji 0 0 0 0 0 ] */
	for( int i=0;i<(int)stack.size();++i )
	  {
	    TaskDynPD & task = * stack[i];
	    MatrixXd & Ctask1 = Ctasks[i];
	    VectorBound & btask1 = btasks[i];

	    EIGEN_MATRIX_FROM_SIGNAL(Jdot,task.JdotSOUT(t));
	    EIGEN_MATRIX_FROM_SIGNAL(J,task.jacobianSOUT(t));
	    const dg::sot::VectorMultiBound & ddx = task.taskSOUT(t);

	    const int nx1 = ddx.size();

	    sotDEBUG(5) << "ddx"<<i<<" = " << ddx << std::endl;
	    sotDEBUG(25) << "J"<<i<<" = " << J << std::endl;
	    sotDEBUG(45) << "Jdot"<<i<<" = " << Jdot << std::endl;

	    assert( Ctask1.rows() == nx1 && btask1.size() == nx1 );
	    assert( J.rows()==nx1 && J.cols()==nq && (int)ddx.size()==nx1 );
	    assert( Jdot.rows()==nx1 && Jdot.cols()==nq );
	    Ctask1.COLS_Q = J;
	    Ctask1.COLS_TAU.setZero();
	    Ctask1.COLS_F.setZero();

	    VectorXd ddxdrift = - (Jdot*dq);
	    for( int c=0;c<nx1;++c )
	      {
		if( ddx[c].getMode() == dg::sot::MultiBound::MODE_SINGLE )
		  btask1[c] = ddx[c].getSingleBound() + ddxdrift[c];
		else
		  {
		    const bool binf = ddx[c].getDoubleBoundSetup( dg::sot::MultiBound::BOUND_INF );
		    const bool bsup = ddx[c].getDoubleBoundSetup( dg::sot::MultiBound::BOUND_SUP );
		    if( binf&&bsup )
		      {
			const double xi = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_INF);
			const double xs = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_SUP);
			btask1[c] = std::make_pair( xi+ddxdrift[c], xs+ddxdrift[c] );
		      }
		    else if( binf )
		      {
			const double xi = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_INF);
			btask1[c] = Bound( xi+ddxdrift[c], Bound::BOUND_INF );
		      }
		    else
		      {
			assert( bsup );
			const double xs = ddx[c].getDoubleBound(dg::sot::MultiBound::BOUND_SUP);
			btask1[c] = Bound( xs+ddxdrift[c], Bound::BOUND_SUP );
		      }
		  }
	      }

	    sotDEBUG(15) << "Ctask"<<i<<" = "     << (MATLAB)Ctask1 << std::endl;
	    sotDEBUG(1) << "btask"<<i<<" = "     << btask1 << std::endl;
	  }

	/* -Last stage- */
	/* Czero = [ [0(30x6) I(30x30)] 0 0 0 0 0 ] */
	Czero.COLS_Q.leftCols(6).setZero();
	Czero.COLS_Q.rightCols(nbDofs).setIdentity();
	Czero.COLS_TAU.setZero();
	Czero.COLS_F.setZero();
	VectorXd ref;
	const double Kv = breakFactorSIN(t);
	if( postureSIN && positionSIN )
	  {
	    EIGEN_VECTOR_FROM_SIGNAL(qref,postureSIN(t));
	    EIGEN_VECTOR_FROM_SIGNAL(q,positionSIN(t));
	    const double Kp = .25*Kv*Kv;
	    ref = (-Kp*(q-qref)-Kv*dq).tail(nbDofs);
	  }
	else
	  {	    ref = (-Kv*dq).tail(nbDofs);	  }
	for( int i=0;i<nbDofs;++i )
	  bzero[i] = ref[i];

	sotDEBUG(15) << "Czero = "     << (MATLAB)Czero << std::endl;
	sotDEBUG(1) << "bzero = "     << bzero << std::endl;

	/* --- */
	sotDEBUG(1) << "Initial config." << std::endl;
	hsolver->reset();
	hsolver->setInitialActiveSet();

	sotDEBUG(1) << "Run for a solution." << std::endl;
	hsolver->activeSearch(solution);
	sotDEBUG(1) << "solution = " << (MATLAB)solution << std::endl;

	EIGEN_VECTOR_FROM_VECTOR( tau,control,nbDofs );
	tau = solution.transpose().COLS_TAU;

	/* --- forces signal --- */
	BOOST_FOREACH(contacts_t::value_type& pContact, contactMap)
	  {
	    Contact& contact = pContact.second;
	    const int nbP = (*contact.supportSIN)(t).nbCols();
	    const int ri = contact.range.first;
	    /* range is an object of the struct Contact containing 2 integers:
	       first: the position of the 1st contact in the parameters vector
	       second: the position of the 2nd contact in the parameters vector */
	    ml::Vector mlf6;
	    EIGEN_VECTOR_FROM_VECTOR(f6,mlf6,6 );
	    ml::Vector mlfn;
	    EIGEN_VECTOR_FROM_VECTOR(fn,mlfn,nbP);

	    f6 = solution.transpose().COLS_F.COLS(ri,ri+6);
	    (*contact.forceSOUT) = mlf6;
	    contact.forceSOUT->setTime(t);

	    for( int i=0;i<nbP;++i ) fn[i] = solution.transpose().COLS_F[ri+6+i];
	    (*contact.fnSOUT) = mlfn;
	    contact.fnSOUT->setTime(t);
	  }

	/* ACC signal */
	{
	  ml::Vector mlacc;
	  EIGEN_VECTOR_FROM_VECTOR( acc,mlacc,nbDofs+6 );
	  acc = solution.transpose().COLS_Q;
	  accelerationSOUT = mlacc;
	}

	/* --- verif --- */
	sotDEBUG(1) << "Vdyn = " << (MATLAB)(VectorXd)(Cdyn*solution) << std::endl;
	sotDEBUG(1) << "Vcontact = " << (MATLAB)(VectorXd)(Ccontact*solution) << std::endl;
	sotDEBUG(1) << "Vzmp = " << (MATLAB)(VectorXd)(Czmp*solution) << std::endl;

	sotDEBUG(1) << "control = " << control << std::endl;
	return control;
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */

      void SolverOpSpace::
      debugOnce( void )
      {
	dg::sot::DebugTrace::openFile("/tmp/sot.txt");
	hsolver->debugOnce();
      }

      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */

      void SolverOpSpace::
      display( std::ostream& os ) const
      {
	os << "SolverOpSpace "<<getName() << ": " << nbDofs <<" joints." << std::endl;
	try{
	  os <<"control = "<<controlSOUT.accessCopy() <<std::endl << std::endl;
	}  catch (dynamicgraph::ExceptionSignal e) {}
	stack_t::display(os);
	dispContacts(os);
      }

      void SolverOpSpace::
      commandLine( const std::string& cmdLine,
		   std::istringstream& cmdArgs,
		   std::ostream& os )
      {
	if( cmdLine == "help" )
	  {
	    os << "SolverOpSpace:\n"
	       << "\t- debugOnce: open trace-file for next iteration of the solver." << std::endl
	       << "\t- addContact: create the contact signals, unpluged." << std::endl
	       << "\t- addContactFromTask <taskName>: Add a contact from the named task." << std::endl
	       << "\t- rmContact <name>: remove a contact." << std::endl
	       << "\t- dispContacts: guess what?." << std::endl;
	    stackCommandLine( cmdLine,cmdArgs,os );
	    Entity::commandLine( cmdLine,cmdArgs,os );
	  }
	else if( cmdLine == "debugOnce" )
	  {
	    debugOnce();
	  }
	else if( cmdLine == "addContact" )
	  {
	    if( cmdArgs.good() )
	      {
		std::string name; cmdArgs >> name;
		addContact( name,NULL,NULL,NULL,NULL );
	      }
	    else { os << "!! A name must be specified. " << std::endl; }
	  }
	else if( cmdLine == "addContactFromTask" )
	  {
	    if( cmdArgs.good() )
	      {
		std::string name; cmdArgs >> name;
		addContactFromTask( name,name );
	      }
	    else { os << "!! A name must be specified. " << std::endl; }
	  }
	else if( cmdLine == "rmContact" )
	  {
	    cmdArgs >> std::ws;
	    if( cmdArgs.good() )
	      {
		std::string name; cmdArgs >> name;
		removeContact( name );
	      }
	    else { os << "!! A name must be specified. " << std::endl; }
	  }
	else if( cmdLine == "dispContacts" ) { dispContacts( os ); }
	// TODO: cf sot-v1: else if( cmdLine == "listen" )
	// 	{
	// 	  hsolver->notifiorRegistration( sotOpSpaceH::hsolverListener );
	// 	}
	else if( stackCommandLine( cmdLine,cmdArgs,os ) );
	else
	  {
	    Entity::commandLine( cmdLine,cmdArgs,os );
	  }
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

