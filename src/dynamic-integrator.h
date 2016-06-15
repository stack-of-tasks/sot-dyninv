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

#ifndef __sot_dyninv_DynamicIntegrator_H__
#define __sot_dyninv_DynamicIntegrator_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (dynamic_integrator_EXPORTS)
#    define SOTDYNAMICINTEGRATOR_EXPORT __declspec(dllexport)
#  else
#    define SOTDYNAMICINTEGRATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTDYNAMICINTEGRATOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>


namespace dynamicgraph {
  namespace sot {
    namespace dyninv {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTDYNAMICINTEGRATOR_EXPORT DynamicIntegrator
	:public ::dynamicgraph::Entity
	,public ::dynamicgraph::EntityHelper<DynamicIntegrator>
	{
	  DYNAMIC_GRAPH_ENTITY_DECL();
	public: /* --- CONSTRUCTOR ---- */

	  DynamicIntegrator( const std::string & name );

	public: /* --- ENTITY INHERITANCE --- */

	  virtual void display( std::ostream& os ) const;

	  virtual void commandLine( const std::string& cmdLine,
				    std::istringstream& cmdArgs,
				    std::ostream& os );

	public:  /* --- SIGNALS --- */

	  DECLARE_SIGNAL_IN( acceleration,dg::Vector );
	  DECLARE_SIGNAL_IN( dt,double );

	  DECLARE_SIGNAL_OUT( velocity,dg::Vector );
	  DECLARE_SIGNAL_OUT( position,dg::Vector );

	public: /* --- MODIFIORS --- */
	  void integrate( const dg::Vector& acceleration, const double& dt,
			  dg::Vector & velocity, dg::Vector & position );
	  void integrateFromSignals( const int & time );
	  void integrateFromSignals( void );

	  void setPosition( const dg::Vector& p );
	  void setVelocity( const dg::Vector& v );
	  void setState( const dg::Vector& p,const dg::Vector& v );

	protected:
	  dg::Vector position,velocity;

	}; // class DynamicIntegrator

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_dyninv_DynamicIntegrator_H__
