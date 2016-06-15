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

#ifndef __sot_dyninv_SolverOpSpace_H__
#define __sot_dyninv_SolverOpSpace_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (solver_op_space_EXPORTS)
#    define SOTSOLVEROPSPACE_EXPORT __declspec(dllexport)
#  else
#    define SOTSOLVEROPSPACE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSOLVEROPSPACE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot-dyninv/stack-template.h>
#include <sot-dyninv/task-dyn-pd.h>
#include <soth/HCOD.hpp>

namespace dynamicgraph {
  namespace sot {
    namespace dyninv {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTSOLVEROPSPACE_EXPORT SolverOpSpace
	:public ::dynamicgraph::Entity
	,public ::dynamicgraph::EntityHelper<SolverOpSpace>
	,public sot::Stack< TaskDynPD >
	{
	  DYNAMIC_GRAPH_ENTITY_DECL();
	public: /* --- CONSTRUCTOR ---- */

	  SolverOpSpace( const std::string & name );

	public: /* --- STACK INHERITANCE --- */

	  typedef sot::Stack<TaskDynPD> stack_t;
	  using stack_t::TaskDependancyList_t;
	  using stack_t::StackIterator_t;
	  using stack_t::StackConstIterator_t;
	  using stack_t::stack;

	  virtual TaskDependancyList_t getTaskDependancyList( const TaskDynPD& task );
	  virtual void addDependancy( const TaskDependancyList_t& depList );
	  virtual void removeDependancy( const TaskDependancyList_t& depList );
	  virtual void resetReady( void );

	public: /* --- ENTITY INHERITANCE --- */

	  virtual void display( std::ostream& os ) const;

	public:  /* --- SIGNALS --- */

	  DECLARE_SIGNAL_IN(matrixInertia,dg::Matrix);
	  DECLARE_SIGNAL_IN(velocity,dg::Vector);
	  DECLARE_SIGNAL_IN(dyndrift,dg::Vector);
	  DECLARE_SIGNAL_IN(damping,double);
	  DECLARE_SIGNAL_IN(breakFactor,double);
	  DECLARE_SIGNAL_IN(posture,dg::Vector);
	  DECLARE_SIGNAL_IN(position,dg::Vector);

	  DECLARE_SIGNAL_OUT(control,dg::Vector);

	  DECLARE_SIGNAL(zmp,OUT,dg::Vector);
	  DECLARE_SIGNAL(acceleration,OUT,dg::Vector);

	private:  /* --- CONTACT POINTS --- */

	  typedef boost::shared_ptr<dynamicgraph::SignalPtr<dg::Matrix,int> > matrixSINPtr;
	  typedef boost::shared_ptr<dynamicgraph::SignalPtr<dg::Vector,int> > vectorSINPtr;
	  typedef boost::shared_ptr<dynamicgraph::Signal<dg::Vector,int> > vectorSOUTPtr;
	  struct Contact
	  {
	    matrixSINPtr jacobianSIN;
	    matrixSINPtr JdotSIN;
	    matrixSINPtr supportSIN;
	    vectorSINPtr correctorSIN;
	    vectorSOUTPtr forceSOUT,fnSOUT;
	    std::pair<int,int> range;
	  };
	  typedef std::map< std::string,Contact > contacts_t;
	  contacts_t contactMap;

	public:
	  void addContact( const std::string & name,
			   dynamicgraph::Signal<dg::Matrix,int> * jacobianSignal,
			   dynamicgraph::Signal<dg::Matrix,int> * JdotSignal,
			   dynamicgraph::Signal<dg::Vector,int> * corrSignal,
			   dynamicgraph::Signal<dg::Matrix,int> * contactPointsSignal );
	  void addContactFromTask( const std::string & taskName, const std::string & contactName );
	  void removeContact( const std::string & name );
	  void dispContacts( std::ostream& os ) const;


	public: /* --- COMMANDS --- */
	  void debugOnce( void );


	private: /* --- INTERNAL COMPUTATIONS --- */
	  void refreshTaskTime( int time );
	  void initialResizeSolver( void );
	  void resizeSolver( void );

	private:
	  int nbParam, nq,ntau;
	  int nbContactBodies,nbContactPoints,nfs;

	  typedef boost::shared_ptr<soth::HCOD> hcod_ptr_t;
	  hcod_ptr_t hsolver;

	  Eigen::MatrixXd Cdyn,Ccontact,Czmp,Czero;
	  soth::VectorBound bdyn,bcontact,bzmp,bzero;
	  std::vector< Eigen::MatrixXd > Ctasks;
	  std::vector< soth::VectorBound > btasks;

	  Eigen::VectorXd solution;

	  static const int nbPoint = 4;

	}; // class SolverOpSpace

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_dyninv_SolverOpSpace_H__
