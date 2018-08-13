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

#ifndef __sot_dyninv_PseudoRobotDynamic_H__
#define __sot_dyninv_PseudoRobotDynamic_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (pseudo_robot_dynamic_EXPORTS)
#    define SOTPSEUDOROBOTDYNAMIC_EXPORT __declspec(dllexport)
#  else
#    define SOTPSEUDOROBOTDYNAMIC_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTPSEUDOROBOTDYNAMIC_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot-dyninv/dynamic-integrator.h>

namespace dynamicgraph {
  namespace sot {
    namespace dyninv {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      /* Inside this entity is an integrator, taking position, velocity and
       * acceleration at time t, and computing pos and vel at time t+1.  Around
       * this integrator, a wrapper is made to make the stuff take the place of
       * the OpenHRP kinematic entity. it thus take a "control" signal, to triger
       * the computations, and possesses all the functionnalities of the OpenHRP
       * entity.  Plus, it has the signals to feed the OpenHRP entity that
       * wrappes the simulator.  All the computations are triggered by computing
       * the "qdot" signal.
       */

      class SOTPSEUDOROBOTDYNAMIC_EXPORT PseudoRobotDynamic
	:public DynamicIntegrator
	,public ::dynamicgraph::EntityHelper<PseudoRobotDynamic>
	{
	  DYNAMIC_GRAPH_ENTITY_DECL();
	public: /* --- CONSTRUCTOR ---- */

	  PseudoRobotDynamic( const std::string & name );

	public: /* --- ENTITY INHERITANCE --- */

	  virtual void display( std::ostream& os ) const;

	  typedef ::dynamicgraph::EntityHelper<PseudoRobotDynamic>::EntityClassName
	    EntityClassName;

	public:  /* --- SIGNALS --- */

	  DECLARE_SIGNAL_IN( control,dg::Vector );
	  DECLARE_SIGNAL_OUT( qdot,dg::Vector );

	  DECLARE_SIGNAL(rotation,OUT,dg::Vector);
	  DECLARE_SIGNAL(translation,OUT,dg::Vector);
	  //sotSignal< dg::Vector,int > rotationSOUT;
	  //sotSignal< dg::Vector,int > translationSOUT;
	  ::dynamicgraph::SignalPtr< dg::Vector,int > stateSOUT;

	public:  /* --- SIGNALS --- */

	  void replaceSimulatorEntity( const std::string& formerName,
				       const bool& plug = false );
	  void setRoot( const dg::Matrix & M );

	public:  /* --- COMMAND --- */
	  template< typename T1 >
	    void forwardVoidCommandToSimu( const std::string& cmdName,
					   const T1& arg1 );
	  void addForward( const std::string& cmdName );

	private:
	  ::dynamicgraph::Entity* formerOpenHRP;

	}; // class PseudoRobotDynamic
    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_dyninv_PseudoRobotDynamic_H__
