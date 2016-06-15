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

#ifndef __sot_dyninv_RobotDynSimu_H__
#define __sot_dyninv_RobotDynSimu_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (robot_dyn_simu_EXPORTS)
#    define SOTROBOTDYNSIMU_EXPORT __declspec(dllexport)
#  else
#    define SOTROBOTDYNSIMU_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTROBOTDYNSIMU_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot-dyninv/dynamic-integrator.h>
#include <sot/core/device.hh>


namespace dynamicgraph {
  namespace sot {
    namespace dyninv {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTROBOTDYNSIMU_EXPORT RobotDynSimu
	:public ::dynamicgraph::sot::Device
	,public ::dynamicgraph::EntityHelper<RobotDynSimu>
	{
	  DYNAMIC_GRAPH_ENTITY_DECL();
	public: /* --- CONSTRUCTOR ---- */

	  RobotDynSimu( const std::string & name );

	public: /* --- ENTITY INHERITANCE --- */

	  virtual void display( std::ostream& os ) const;

	  typedef ::dynamicgraph::EntityHelper<RobotDynSimu>::EntityClassName
	    EntityClassName;

	public:  /* --- SIGNALS --- */

	  DECLARE_SIGNAL_IN( acceleration,dg::Vector );
	  DECLARE_SIGNAL_OUT( velocity,dg::Vector );

	protected:
	  virtual void integrate( const double & dt );

	private:
	  dg::Vector velocity_;

	public:
	  void setVelocity( const dg::Vector& v );

	}; // class RobotDynSimu
    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_dyninv_RobotDynSimu_H__
