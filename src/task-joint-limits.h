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

#ifndef __sot_dyninv_TaskJointLimits_H__
#define __sot_dyninv_TaskJointLimits_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (task_joint_limits_EXPORTS)
#    define SOTTASKDYNJOINTLIMITS_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKDYNJOINTLIMITS_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKDYNJOINTLIMITS_EXPORT
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


      class SOTTASKDYNJOINTLIMITS_EXPORT TaskJointLimits
	:public TaskAbstract
	,public EntityHelper<TaskJointLimits>
	{
	public: /* --- CONSTRUCTOR ---- */

	  TaskJointLimits( const std::string& name );

	public: /* --- ENTITY INHERITANCE --- */

	  DYNAMIC_GRAPH_ENTITY_DECL();
	  virtual void display( std::ostream& os ) const;

	public:  /* --- SIGNALS --- */

	  DECLARE_SIGNAL_IN(position,ml::Vector);
	  DECLARE_SIGNAL_IN(referenceInf,ml::Vector);
	  DECLARE_SIGNAL_IN(referenceSup,ml::Vector);
	  DECLARE_SIGNAL_IN(dt,double);
	  DECLARE_SIGNAL_IN(controlGain,double);
	  DECLARE_SIGNAL_IN(selec,Flags);

	  DECLARE_SIGNAL_OUT(normalizedPosition,ml::Vector);
	  DECLARE_SIGNAL_OUT(activeSize,int);

	public:  /* --- COMPUTATION --- */
	  dg::sot::VectorMultiBound&
	    computeTask( dg::sot::VectorMultiBound& res,int time );
	  ml::Matrix& computeJacobian( ml::Matrix& J,int time );

	}; // class TaskJointLimits

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_dyninv_TaskJointLimits_H__
