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

#ifndef __sot_dyninv_ContactSelecter_H__
#define __sot_dyninv_ContactSelecter_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (solver_op_space_EXPORTS)
#    define SOTCONTACTSELECTER_EXPORT __declspec(dllexport)
#  else
#    define SOTCONTACTSELECTER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTCONTACTSELECTER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot-dyninv/solver-dyn-reduced.h>
#include <map>

namespace dynamicgraph {
  namespace sot {
    namespace dyninv {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTCONTACTSELECTER_EXPORT ContactSelecter
	:public ::dynamicgraph::Entity
	,public ::dynamicgraph::EntityHelper<ContactSelecter>
	{

	public: /* --- CONSTRUCTOR ---- */

	  ContactSelecter( const std::string & name );

	public: /* --- ENTITY INHERITANCE --- */

	  static const std::string CLASS_NAME;
	  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }
	  virtual void display( std::ostream& os ) const;
	  void initCommands( void );

	public:  /* --- SIGNALS --- */

	  DECLARE_SIGNAL_OUT(trigger,int);

	public: /* --- COMMANDS --- */

	  void setSolverReference( const std::string& solverName );
	  void setContact( const std::string & contactName, const std::string & contactTaskName );
	  void setTask( const std::string & contactName, const std::string & taskName );
	  void setContactAndTask( const std::string & contactName,
				  const std::string & contactTaskName,
				  const std::string & taskName );
	  void setContactStatus( const std::string & contactName,
				 const bool & status );
	  bool getContactStatus( const std::string & contactName );

	public:

	  struct ContactInfo
	  {
	    std::string contactTaskName;
	    std::string complementaryTaskName;
	    std::string contactName;
	    bool status;
	    DECLARE_SIGNAL_IN(activation,bool);
	    DECLARE_SIGNAL_IN(support,ml::Matrix);
	    ContactInfo( const std::string & contactName,
			 const std::string & contactTaskName,
			 const std::string & complementaryTaskName,
			 ContactSelecter& mother);
	  };
	  typedef boost::shared_ptr<ContactInfo> ContactInfo_ptr;
	  typedef std::map<std::string, ContactInfo_ptr> ContactInfo_map;

	  typedef SolverDynReduced Solver;
	  typedef Solver* Solver_ptr;

	private:
	  ContactInfo_map contacts;
	  Solver_ptr solver;
	  bool verbose;

	}; // class ContactSelecter

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_dyninv_ContactSelecter_H__
