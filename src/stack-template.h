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

#ifndef __sot_dyninv_StackTemplate_H__
#define __sot_dyninv_StackTemplate_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (stack_template_EXPORTS)
#    define SOTSTACKTEMPLATE_EXPORT __declspec(dllexport)
#  else
#    define SOTSTACKTEMPLATE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSTACKTEMPLATE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot-dyninv/signal-helper.h>

namespace dynamicgraph {
  namespace sot {

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    namespace dg = dynamicgraph;

    template< typename TaskGeneric >
      class Stack
      {

      protected:

	typedef std::vector<TaskGeneric*> Stack_t;
	typedef typename Stack_t::iterator StackIterator_t;
	typedef typename Stack_t::const_iterator StackConstIterator_t;

	/*! \brief List of task (controllers) managed by the stack of tasks. */
	Stack_t stack;

	/*! \brief Number of joints to be used to compute the control law. */
	int nbDofs;

      public:

	/*! \brief Number of joints by default. */
	static const unsigned int NB_JOINTS_DEFAULT; // = 48;

      public: /* --- Constructors --- */

	Stack( void );
	~Stack( void ) { /* TODO!! */ }

      public: /* --- Modifiors --- */

	/*! \name Methods to handle the stack.
	  @{
	*/

	/*! \brief Push the task in the stack.
	  It has a lowest priority than the previous ones.
	  If this is the first task, then it has the highest
	  priority. */
	virtual void push( TaskGeneric& task );
	/*! \brief Pop the task from the stack.
	  This method removes the task with the smallest
	  priority in the task. The other are projected
	  in the null-space of their predecessors. */
	virtual TaskGeneric& pop( void );
	/*! \brief same as pop, but no return. */
	void pop0( void );

	/*! \brief This method allows to know if a task exists or not */
	virtual bool exist( const TaskGeneric& task );

	/*! \brief Remove a task regardless to its position in the stack.
	  It removes also the signals connected to the output signal of this stack.*/
	virtual void remove( const TaskGeneric& task );

	/*! \brief This method makes the task to swap with the task having the
	  immediate superior priority. */
	virtual void up( const TaskGeneric& task );

	/*! \brief This method makes the task to swap with the task having the
	  immediate inferior priority. */
	virtual void down( const TaskGeneric& task );

	/*! \brief Remove all the tasks from the stack. */
	virtual void clear( void );

	/*! Change the number of DOF, ie the field nbDofs. */
	virtual void defineNbDof( const int& nbDof );

	/*! @} */

	dg::SignalBase<int>* tatata;

	int titit;
	typedef std::list< const dg::SignalBase<int>* > TaskDependancyList_t;

	int totol;

	/*! \brief Return the signal to be added/removed from the dependancy
	 * list of the control signal. */
	virtual TaskDependancyList_t getTaskDependancyList( const TaskGeneric& task ) = 0;
	virtual void addDependancy( const TaskDependancyList_t& depList ) = 0;
	virtual void removeDependancy( const TaskDependancyList_t& depList ) = 0;
	virtual void resetReady( void ) = 0;

      public: /* --- DISPLAY --- */

	/*! \name Methods to display the stack of tasks.
	  @{
	*/

	/*! Display the stack of tasks in text mode as a tree. */
	virtual void display( std::ostream& os ) const;
	/*! \brief Write the priority between tasks in the outstream os. */
	virtual std::ostream &
	  writeGraph(const std::string & name, std::ostream & os) const;

	/*! @} */

      public: /* --- COMMANDS --- */

	/*! @{ */

	/* To use the macro, the typedef sot::Stack<TaskSpec> stack_t has to be set,
	 * as well as the EntityClassName. */
	//#define ADD_COMMANDS_FOR_THE_STACK /// Code in the .t.cpp


	/*! \brief This method deals with the command line.
	  The command given in argument is send to the stack of tasks by the shell.
	  The command understood by sot are:
	  <ul>
	  <li> Tasks
	  <ul>
	  <li> push <task> : Push a task in the stack (FILO).
	  <li> pop : Remove the task push in the stack.
	  <li> down <task> : Make the task have a higher priority, i.e.
	  swap with the task immediatly superior in priority.
	  <li> up <task> : Make the task have a lowest priority, i.e.
	  swap with the task immediatly inferior in priority.
	  <li> rm <task> : Remove the task from the stack.
	  </ul>
	*/
	virtual bool stackCommandLine( const std::string& cmdLine,
				       std::istringstream& cmdArgs,
				       std::ostream& os );

	void pushByTaskName( const std::string& taskName );
	void removeByTaskName( const std::string& taskName );
	void upByTaskName( const std::string& taskName );
	void downByTaskName( const std::string& taskName );

	/*! @} */

      }; // class StackTemplate


    template< typename TaskGeneric >
      std::ostream&
      operator<< ( std::ostream& os,const Stack<TaskGeneric>& sot )
      {
	sot.display(os);
	return os;
      }


  } // namespace sot
} // namespace dynamicgraph

#include <sot-dyninv/stack-template.t.cpp>

#endif // #ifndef __sot_dyninv_StackTemplate_H__
