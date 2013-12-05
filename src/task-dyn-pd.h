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

#ifndef __sot_dyninv_TaskDynPD_H__
#define __sot_dyninv_TaskDynPD_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (task_dyn_pd_EXPORTS)
#    define SOTTASKDYNPD_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKDYNPD_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKDYNPD_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot/core/task.hh>

namespace dynamicgraph {
  namespace sot {
    namespace dyninv {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */


      class SOTTASKDYNPD_EXPORT TaskDynPD
	:public ::dynamicgraph::sot::Task
	,public ::dynamicgraph::EntityHelper<TaskDynPD>
	{
	  DYNAMIC_GRAPH_ENTITY_DECL();
	public: /* --- CONSTRUCTOR ---- */

	  TaskDynPD( const std::string & name );

	public: /* --- ENTITY INHERITANCE --- */

	  virtual void display( std::ostream& os ) const;

	public:  /* --- SIGNALS --- */

	  DECLARE_SIGNAL_IN(Kv,double);
	  DECLARE_SIGNAL_IN(qdot,ml::Vector);
	  DECLARE_SIGNAL_IN(dt,double);

	  DECLARE_SIGNAL_OUT(errorDot,ml::Vector);
	  DECLARE_SIGNAL_OUT(KvAuto,double);
	  DECLARE_SIGNAL_OUT(Jdot,ml::Matrix);
	  DECLARE_SIGNAL_OUT(taskVector,ml::Vector);

	protected:
	  dynamicgraph::sot::VectorMultiBound&
	    taskSOUT_function( dynamicgraph::sot::VectorMultiBound& task,int iter );

	protected:
	  ml::Matrix previousJ;
	  bool previousJset;

	public: /* COMMANDS */
	  void resetJacobianDerivative( void );

	}; // class TaskDynPD

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_dyninv_TaskDynPD_H__
