
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

#ifndef __sot_dyninv_TaskWeight_H__
#define __sot_dyninv_TaskWeight_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (task_weight_EXPORTS)
#    define SOTTASKDYNWEIGHT_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKDYNWEIGHT_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKDYNWEIGHT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot/core/task.hh>
#include <sot/core/flags.hh>

namespace dynamicgraph {
  namespace sot {
    namespace dyninv {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */


      class SOTTASKDYNWEIGHT_EXPORT TaskWeight
	:public Task
	,public EntityHelper<TaskWeight>
	{

	public: /* --- CONSTRUCTOR ---- */

	  TaskWeight( const std::string& name );

	public: /* --- ENTITY INHERITANCE --- */

	  static const std::string CLASS_NAME;
	  virtual const std::string& getClassName( void ) { return CLASS_NAME; }
	  virtual void display( std::ostream& os ) const;

	public:  /* --- SIGNALS --- */

	public:  /* --- COMPUTATION --- */
	  dg::sot::VectorMultiBound&
	    computeTask( dg::sot::VectorMultiBound& res,int time );
	  ml::Matrix&
	    computeJacobian( ml::Matrix& res,int time );


	  struct TaskContener
	  {
	    Task* task;
	    double weight;
	  };
	  std::list< TaskContener > taskList;

	  void addTask(const std::string& name,const double & w);
	  void setWeight(const std::string& name,const double & w);


	}; // class TaskWeight

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_dyninv_TaskWeight_H__
