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

/*! \file task-dyn-passing-point.h                                                                                                                                                                                 
  \brief Defines a task based on time constraints as well as the initial 
  and final position and velocity.
*/


#ifndef __sot_dyninv_TaskDynPassingPoint_H__
#define __sot_dyninv_TaskDynPassingPoint_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (task_dyn_passing_point_EXPORTS)
#    define SOTTASKDYNPASSINGPOINT_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKDYNPASSINGPOINT_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKDYNPASSINGPOINT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot-dyninv/task-dyn-pd.h>
#include <sot/core/task.hh>

namespace dynamicgraph {
  namespace sot {
    namespace dyninv {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */


      class SOTTASKDYNPASSINGPOINT_EXPORT TaskDynPassingPoint
	:public ::dynamicgraph::sot::dyninv::TaskDynPD
	,public ::dynamicgraph::EntityHelper<TaskDynPassingPoint>
	{

	public:
	  /*! Constructor                                                                                                                                                                               
	    @param name: Name of the task (string)                                                                                                             
	  */
	  TaskDynPassingPoint( const std::string & name );

	  /* --- ENTITY INHERITANCE --- */
	  typedef TaskDynPassingPoint EntityClassName;
	  static const std::string CLASS_NAME;
	  virtual void display( std::ostream& os ) const;
	  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

	  /* --- SIGNALS --- */
	  DECLARE_SIGNAL_IN(velocityDes, ml::Vector);
	  DECLARE_SIGNAL_IN(duration, double);
	  DECLARE_SIGNAL_IN(initialTime, double);

	  DECLARE_SIGNAL_OUT(velocityCurrent, ml::Vector);
	  DECLARE_SIGNAL_OUT(velocityDesired, ml::Vector);


	protected:
	  /* --- COMPUTATION --- */
	  dg::sot::VectorMultiBound& computeTaskSOUT( dg::sot::VectorMultiBound& task, int iter );

	private:
	  ml::Vector previousTask;

	}; // class  TaskDynPassingPoint

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_dyninv_TaskDynPassingPoint_H__
