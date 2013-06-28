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

#ifndef __sot_dyninv_SolverKine_H__
#define __sot_dyninv_SolverKine_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (solver_kine_EXPORTS)
#    define SOTSOLVERKINE_EXPORT __declspec(dllexport)
#  else
#    define SOTSOLVERKINE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSOLVERKINE_EXPORT
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

      class SOTSOLVERKINE_EXPORT SolverKine
	:public ::dynamicgraph::Entity
	,public ::dynamicgraph::EntityHelper<SolverKine>
	,public sot::Stack< TaskAbstract >
	{

	public: /* --- CONSTRUCTOR ---- */

	  SolverKine( const std::string & name );

	public: /* --- STACK INHERITANCE --- */

	  typedef sot::Stack<TaskAbstract> stack_t;
	  using stack_t::TaskDependancyList_t;
	  using stack_t::StackIterator_t;
	  using stack_t::StackConstIterator_t;
	  using stack_t::stack;

	  virtual TaskDependancyList_t getTaskDependancyList( const TaskAbstract& task );
	  virtual void addDependancy( const TaskDependancyList_t& depList );
	  virtual void removeDependancy( const TaskDependancyList_t& depList );
	  virtual void resetReady( void );

	public: /* --- ENTITY INHERITANCE --- */

	  static const std::string CLASS_NAME;
	  virtual void display( std::ostream& os ) const;
	  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

	  virtual void commandLine( const std::string& cmdLine,
				    std::istringstream& cmdArgs,
				    std::ostream& os );

	public:  /* --- SIGNALS --- */

	  DECLARE_SIGNAL_IN(damping,double);
	  DECLARE_SIGNAL_IN(velocity,ml::Vector); //only used for second order kinematics
	  DECLARE_SIGNAL_OUT(control,ml::Vector);

	public: /* --- COMMANDS --- */
	  void debugOnce( void );
	  void resetAset( void );
	  void getDecomposition( const int &stage );
	  bool controlFreeFloating;
	  bool secondOrderKinematics;
	  /// Push the task in the stack.
	  /// Call parent implementation anc check that task is
	  /// of type dynamic if necessary
	  virtual void push( TaskAbstract& task );
	  void setSecondOrderKine (const bool& secondOrder);

	private: /* --- INTERNAL COMPUTATIONS --- */
	  void refreshTaskTime( int time );
	  bool checkSolverSize( void );
	  void resizeSolver( void );
	  void checkDynamicTask (const TaskAbstract& task) const;

	private:
	  typedef boost::shared_ptr<soth::HCOD> hcod_ptr_t;
	  hcod_ptr_t hsolver;

	  std::vector< Eigen::MatrixXd > Ctasks;
	  std::vector< soth::VectorBound > btasks;

	  Eigen::VectorXd solution;
	  std::vector<soth::cstref_vector_t> activeSet;
	  bool relevantActiveSet;
	  
	  bool ddxdriftInit_;
	  Eigen::VectorXd ddxdrift_ ;



	}; // class SolverKine

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_dyninv_SolverKine_H__
