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

//#define VP_DEBUG
#define VP_DEBUG_MODE 50
#include <sot/core/debug.hh>
#ifdef VP_DEBUG
class contacter_op_space__INIT
{
public:contacter_op_space__INIT( void )
  {
    //    dynamicgraph::sot::DebugTrace::openFile("/tmp/dynred.txt");
  }
};
contacter_op_space__INIT contacter_op_space_initiator;
#endif //#ifdef VP_DEBUG

#include <iostream>
#include <sot-dyninv/commands-helper.h>
#include <sot-dyninv/contact-selecter.h>
#include <dynamic-graph/factory.h>
#include <boost/foreach.hpp>

namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {
#include <Eigen/Cholesky>

      namespace dg = ::dynamicgraph;
      using namespace dg;
      using dg::SignalBase;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ContactSelecter,"ContactSelecter");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      ContactSelecter::
      ContactSelecter( const std::string & name )
	: Entity(name)

	,CONSTRUCT_SIGNAL_OUT(trigger,int,sotNOSIGNAL)
	,contacts()
	,solver(NULL)

      {
	signalRegistration( triggerSOUT );
	initCommands();
      }

      void ContactSelecter::
      initCommands( void )
      {
	using namespace dynamicgraph::command;

	addCommand("setContact",
		   makeCommandVoid2(*this,&ContactSelecter::setContact,
				    docCommandVoid2("Create the contact, and give the name of the task that reprensent the contact function.",
						    "contact name (string)",
						    "name of the task definining the contact (string)")));

	addCommand("setTask",
		   makeCommandVoid2(*this,&ContactSelecter::setTask,
				    docCommandVoid2("Create the contact, and give the name of the task that is orthogonal to the contact. The task will be active when the input boolean is false.",
						    "contact name (string)",
						    "name of the task definining the orthogonal (string)")));

	addCommand("setContactAndTask",
		   makeCommandVoid3(*this,&ContactSelecter::setContactAndTask,
				    docCommandVoid3("Create the contact, and give the name of both the contact task and the orthogonal task.",
						    "contact name (string)",
						    "name of the task definining the contact (string)",
						    "name of the task definining the orthogonal (string)")));

	addCommand("setSolver",
		   makeCommandVoid1(*this,&ContactSelecter::setSolverReference,
				    docCommandVoid1("Give the name of the solver",
						    "solver name (string)")));

	addCommand("setContactStatus",
		   makeCommandVoid2(*this,&ContactSelecter::setContactStatus,
				    docCommandVoid2("Explicitely specify the status",
						    "contact name (string)",
						    "status (bool)")));

	addCommand("verbose",
		   makeDirectSetter(*this, &verbose,
				    docDirectSetter("verbose","bool")));
	

      }

      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */

      void ContactSelecter::
      setSolverReference( const std::string& solverName )
      {
	solver = dynamic_cast<Solver_ptr> ( &g_pool().getEntity(solverName) );
      }

      ContactSelecter::ContactInfo::
      ContactInfo( const std::string & contactName,
		   const std::string & contactTaskName,
		   const std::string & taskName,
		   ContactSelecter& mother)
	:contactTaskName(contactTaskName)
	,complementaryTaskName(taskName)
	,contactName( contactName )
	,status(false)
	,activationSIN( NULL,"ContactSelecter("+mother.getName()+")::input(bool)::"+contactName+"Activation" )
	,supportSIN( NULL,"ContactSelecter("+mother.getName()+")::input(matrix)::"+contactName+"Support" )
      {
	mother.signalRegistration( activationSIN << supportSIN );
	mother.triggerSOUT.addDependency( activationSIN );
      }

      void ContactSelecter::
      setContact( const std::string & contactName, const std::string & contactTaskName )
      {
	contacts[contactName] = ContactInfo_ptr( new ContactInfo(contactName,contactTaskName,"",*this) );
      }

      void ContactSelecter::
      setTask( const std::string & contactName, const std::string & taskName )
      {
	contacts[contactName] = ContactInfo_ptr( new ContactInfo(contactName,"",taskName,*this) );
      }
      void ContactSelecter::
      setContactAndTask( const std::string & contactName,
			 const std::string & contactTaskName,
			 const std::string & taskName )
      {
	contacts[contactName]
	  = ContactInfo_ptr( new ContactInfo(contactName,contactTaskName,taskName,*this) );
      }
      void ContactSelecter::
      setContactStatus(  const std::string & contactName,
			 const bool& status )
      {
	contacts[contactName]->status = status;
      }
      bool ContactSelecter::
      getContactStatus(  const std::string & contactName )
      {
	return contacts[contactName]->status;
      }


      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      int& ContactSelecter::
      triggerSOUT_function( int&,int t)
      {
	BOOST_FOREACH( ContactInfo_map::value_type iter, contacts)
	  {
	    ContactInfo_ptr contact = iter.second;
	    bool newStatus = contact->activationSIN(t);
	    if( newStatus!=contact->status )
	      {
		assert( solver!=NULL);
		if( newStatus )  // Activate the contact
		  {
		    if( verbose)
		      std::cout << "Iter " << t
				<< ":  Add the contact " << contact->contactName << std::endl;

		    /* Add the contact to the solver. */
		    if( contact->contactTaskName.size()>0 )
		      {
			solver->addContactFromTask(contact->contactTaskName,contact->contactName);
			solver->getSupportSIN(contact->contactName)->plug( &contact->supportSIN );
			//solver->getSupportSIN(contact->contactName)->setReady();
			contact->supportSIN.setReady();
		      }
		    /* Remove the complementary task from the solver. */
		    if( contact->complementaryTaskName.size()>0 )
		      {
			TaskDynPD & task = dynamic_cast<TaskDynPD&>
			  ( g_pool().getEntity(contact->complementaryTaskName) );
			solver->remove(task);
		      }
		  }
		else             // Unactive the contact
		  {
		    if( verbose)
		      std::cout << "Iter " << t
				<< ":  Rm the contact " << contact->contactName << std::endl;

		    /* Add the complementary task too the solver. */
		    if( contact->complementaryTaskName.size()>0 )
		      {
			TaskDynPD & task = dynamic_cast<TaskDynPD&>
			  ( g_pool().getEntity(contact->complementaryTaskName) );
			solver->push(task);
		      }

		    /* Remove the contact from the solver. */
		    if( contact->contactTaskName.size()>0 )
		      {
			solver->removeContact(contact->contactName);
		      }

		  }

		contact->status = newStatus;
	      }
	  }

      }
      
      void ContactSelecter::display( std::ostream& os ) const
      {
	using std::endl;
	os << "Contact&task selecter '" << getName() << "':" << endl
	   << " - attached solver: " << solver->getName() << endl
	   << " - contacts:" << endl;
	
	BOOST_FOREACH( ContactInfo_map::value_type iter, contacts)
	  {
	    ContactInfo_ptr contact = iter.second;
	    os << "   * " << contact->contactName << ": "
	       << "contact task '" << contact->contactTaskName <<"'"
	       << "   -- complementary task '" << contact->complementaryTaskName <<"'"
	       << endl;
	  }
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

