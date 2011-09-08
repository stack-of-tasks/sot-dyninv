/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
 *
 * This file is part of sot-dyninv add ons.
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

#ifndef __sot_dyninv_TaskDynLimits_H__
#define __sot_dyninv_TaskDynLimits_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (task_dyn_limits_EXPORTS)
#    define SOTTASKDYNLIMITS_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKDYNLIMITS_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKDYNLIMITS_EXPORT
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


      class SOTTASKDYNLIMITS_EXPORT TaskDynLimits
	:public ::dynamicgraph::sot::dyninv::TaskDynPD
	,public ::dynamicgraph::EntityHelper<TaskDynLimits>
	{

	public: /* --- CONSTRUCTOR ---- */

	  TaskDynLimits( const std::string& name );

	public: /* --- ENTITY INHERITANCE --- */
	  typedef TaskDynLimits EntityClassName;
	  static const std::string CLASS_NAME;
	  virtual const std::string& getClassName( void ) { return CLASS_NAME; }
	  //virtual void display( std::ostream& os ) const;

	public:  /* --- SIGNALS --- */

	  DECLARE_SIGNAL_IN(position,ml::Vector);
	  DECLARE_SIGNAL_IN(velocity,ml::Vector);

	  DECLARE_SIGNAL_IN(referencePosInf,ml::Vector);
	  DECLARE_SIGNAL_IN(referencePosSup,ml::Vector);
	  DECLARE_SIGNAL_IN(referenceVelInf,ml::Vector);
	  DECLARE_SIGNAL_IN(referenceVelSup,ml::Vector);

	  DECLARE_SIGNAL_OUT(normalizedVelocity,ml::Vector);
	  DECLARE_SIGNAL_OUT(normalizedPosition,ml::Vector);

	public:  /* --- COMPUTATION --- */
	  dg::sot::VectorMultiBound& computeTaskDynLimits( dg::sot::VectorMultiBound& res,int time );
	  ml::Matrix& computeTjlJacobian( ml::Matrix& J,int time );
	  ml::Matrix& computeTjlJdot( ml::Matrix& Jdot,int time );

	protected:
	  ml::Matrix previousJ;
	  bool previousJset;

	private:   /* --- DISPLAY --- */
	  void display( std::ostream& os ) const;

	}; // class TaskDynLimits

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_oscar_TaskDynLimits_H__
