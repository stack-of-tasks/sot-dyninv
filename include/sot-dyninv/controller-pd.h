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

#ifndef __sot_dyninv_ControllerPD_H__
#define __sot_dyninv_ControllerPD_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (controller_pd_EXPORTS)
#    define SOTCONTROLLERPD_EXPORT __declspec(dllexport)
#  else
#    define SOTCONTROLLERPD_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTCONTROLLERPD_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>



namespace sot {
  namespace dyninv {

  /* --------------------------------------------------------------------- */
  /* --- CLASS ----------------------------------------------------------- */
  /* --------------------------------------------------------------------- */


  class SOTCONTROLLERPD_EXPORT ControllerPD
    :public ::dynamicgraph::Entity
      ,public ::dynamicgraph::EntityHelper<ControllerPD>
      {

      public: /* --- CONSTRUCTOR ---- */

	ControllerPD( const std::string & name );

      protected:
	/* Parameters of the torque-control function:
	 * tau = kp * (qd-q) + kd* (dqd-dq) */
	int _dimension;

      public:
	void size(const int & dimension);
	int size(void) const;

      public: /* --- ENTITY INHERITANCE --- */

	static const std::string CLASS_NAME;
	virtual void display( std::ostream& os ) const;
	virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

	virtual void commandLine( const std::string& cmdLine,
				  std::istringstream& cmdArgs,
				  std::ostream& os );

      public:  /* --- SIGNALS --- */

	DECLARE_SIGNAL_IN(Kp,ml::Vector);
	DECLARE_SIGNAL_IN(Kd,ml::Vector);
	DECLARE_SIGNAL_IN(position,ml::Vector);
	DECLARE_SIGNAL_IN(positionRef,ml::Vector);
	DECLARE_SIGNAL_IN(velocity,ml::Vector);
	DECLARE_SIGNAL_IN(velocityRef,ml::Vector);

	DECLARE_SIGNAL_OUT(control,ml::Vector);

      }; // class ControllerPD

  } // namespace dyninv
} // namespace sot



#endif // #ifndef __sot_dyninv_ControllerPD_H__
