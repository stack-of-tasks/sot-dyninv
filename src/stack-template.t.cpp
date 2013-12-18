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

#ifndef __sot_dyninv_StackTemplate_TCC__
#define __sot_dyninv_StackTemplate_TCC__

#include <dynamic-graph/pool.h>
#include <sot/core/debug.hh>
#include <sot/core/task-abstract.hh>

namespace dynamicgraph
{
  namespace sot
  {

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    template< typename TaskGeneric >
    const unsigned int Stack<TaskGeneric>::NB_JOINTS_DEFAULT = 46;

    /* --------------------------------------------------------------------- */
    /* --- CONSTRUCTION ---------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    template< typename TaskGeneric >
    Stack<TaskGeneric>::
    Stack( void )
      :stack()
      ,nbDofs( NB_JOINTS_DEFAULT )
    {
    }

    /* --------------------------------------------------------------------- */
    /* --- STACK MANIPULATION --- */
    /* --------------------------------------------------------------------- */

    template< typename TaskGeneric >
    void Stack<TaskGeneric>::
    push( TaskGeneric& task )
    {
      stack.push_back( &task );
      addDependancy( getTaskDependancyList( task ) );
      resetReady();
    }
    template< typename TaskGeneric >
    TaskGeneric& Stack<TaskGeneric>::
    pop( void )
    {
      TaskGeneric* res = stack.back();
      removeDependancy( getTaskDependancyList( *res ) );
      stack.pop_back();
      resetReady();
      return *res;
    }
    template< typename TaskGeneric >
    void Stack<TaskGeneric>::
    pop0( void )
    {
      pop();
    }
    template< typename TaskGeneric >
    bool Stack<TaskGeneric>::
    exist( const TaskGeneric& key )
    {
      StackIterator_t it;
      for ( it=stack.begin();stack.end()!=it;++it )
	{
	  if( *it == &key ) { return true; }
	}
      return false;
    }
    template< typename TaskGeneric >
    void Stack<TaskGeneric>::
    remove( const TaskGeneric& key )
    {
      bool find =false;
      StackIterator_t it;
      for ( it=stack.begin();stack.end()!=it;++it )
	{
	  if( *it == &key ) { find=true; break; }
	}
      if(! find ){ return; }

      stack.erase( it );
      removeDependancy( getTaskDependancyList( key ) );
      resetReady();
    }

    template< typename TaskGeneric >
    void Stack<TaskGeneric>::
    up( const TaskGeneric& key )
    {
      bool find =false;
      StackIterator_t it;
      for ( it=stack.begin();stack.end()!=it;++it )
	{
	  if( *it == &key ) { find=true; break; }
	}
      if( stack.begin()==it ) { return; }
      if(! find ){ return; }

      StackIterator_t pos=it; pos--;
      TaskGeneric * task = *it;
      stack.erase( it );
      stack.insert( pos,task );
      resetReady();
    }
    template< typename TaskGeneric >
    void Stack<TaskGeneric>::
    down( const TaskGeneric& key )
    {
      bool find =false;
      StackIterator_t it;
      for ( it=stack.begin();stack.end()!=it;++it )
	{
	  if( *it == &key ) { find=true; break; }
	}
      if( stack.end()==it ) { return; }
      if(! find ){ return; }

      StackIterator_t pos=it; pos++;

      // If the task is already at the end of the stack, do nothing
      if( stack.end()==pos ){ return; }

      TaskGeneric* task=*it;
      stack.erase( it );
      // the task was the second to last one
      if( stack.end()==pos ){ stack.push_back(task); }
      else
	{
	  stack.insert( pos,task );
	}
      resetReady();
    }

    template< typename TaskGeneric >
    void Stack<TaskGeneric>::
    clear( void )
    {
      for ( StackIterator_t it=stack.begin();stack.end()!=it;++it )
	{
	  removeDependancy( getTaskDependancyList( **it ) );
	}
      stack.clear();
      resetReady();
    }

    template< typename TaskGeneric >
    void Stack<TaskGeneric>::
    defineNbDof( const int& nbDof )
    {
      assert(nbDof >= 0);
      nbDofs = nbDof;
      resetReady();
    }

    /* --------------------------------------------------------------------- */
    /* --- DISPLAY --------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    template< typename TaskGeneric >
    void Stack<TaskGeneric>::
    display( std::ostream& os ) const
    {

      os << "+-----------------"<<std::endl<<"+   SOT     "
	 << std::endl<< "+-----------------"<<std::endl;
      for ( StackConstIterator_t it=this->stack.begin();
	    this->stack.end()!=it;++it )
	//BOOST_FOREACH( TaskGeneric* task, stack )
	{
	  os << "| " << (*it)->getName() <<std::endl;
	}
    }


    /* --------------------------------------------------------------------- */
    /* --- COMMAND --------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

#define ACT_BY_TASK_NAME(ACT)					\
    template< typename TaskGeneric >				\
    void  Stack<TaskGeneric>::					\
    ACT##ByTaskName( const std::string& tname )			\
    {								\
      dg::Entity & taska = dg::g_pool().getEntity( tname );	\
      TaskGeneric & task = dynamic_cast<TaskGeneric&>( taska );	\
      ACT(task);						\
    }

    ACT_BY_TASK_NAME(push)
    ACT_BY_TASK_NAME(remove)
    ACT_BY_TASK_NAME(up)
    ACT_BY_TASK_NAME(down)

    template< typename TaskGeneric >
    std::ostream& Stack<TaskGeneric>::
    writeGraph( const std::string & name, std::ostream& os ) const
    {
      StackConstIterator_t iter;
      for( iter = stack.begin(); iter!=stack.end();++iter )
	{
	  const TaskGeneric & task = **iter;
	  StackConstIterator_t nextiter =iter;
	  nextiter++;

	  if (nextiter!=stack.end())
	    {
	      TaskGeneric & nexttask = **nextiter;
	      os << "\t\t\t" << task.getName() << " -> " << nexttask.getName()
		 << " [color=red]" << std::endl;
	    }
	}

      os << "\t\tsubgraph cluster_Tasks {" <<std::endl
	 << "\t\t\tsubgraph cluster_" << name << " {" << std::endl
	 << "\t\t\t\tcolor=lightsteelblue1; label=\"" << name
	 << "\"; style=filled;" << std::endl;

      for(  iter = stack.begin(); iter!=stack.end();++iter )
	{
	  const TaskGeneric & task = **iter;
	  os << "\t\t\t\t" << task.getName()
	     << " [ label = \"" << task.getName() << "\" ," << std::endl
	     << "\t\t\t\t   fontcolor = black, color = black, "
	     << "fillcolor = magenta, style=filled, shape=box ]" << std::endl;

	}
      os << "\t\t\t}" << std::endl;
      os << "\t\t\t}" <<std::endl;
      return os;
    }




    /* To use the macro, the typedef sot::Stack<TaskSpec> stack_t has to be set,
     * as well as the EntityClassName. */
#define ADD_COMMANDS_FOR_THE_STACK					\
    addCommand("dispStack",						\
	       makeCommandVerbose(*this,&stack_t::display,		\
				  docCommandVerbose("Guess what?")));	\
    addCommand("up",							\
	       makeCommandVoid1(*this,					\
				(void (EntityClassName::*)(const std::string&))	\
				&stack_t::upByTaskName,			\
				docCommandVoid1("Up the named task.",	\
						"string (task name)"))); \
    addCommand("down",							\
	       makeCommandVoid1(*this,					\
				(void (EntityClassName::*)(const std::string&))	\
				&stack_t::downByTaskName,		\
				docCommandVoid1("Down the named task.",	\
						"string (task name)"))); \
    addCommand("push",							\
	       makeCommandVoid1(*this,					\
				(void (EntityClassName::*)(const std::string&))	\
				&stack_t::pushByTaskName,		\
				docCommandVoid1("Push the named task.",	\
						"string (task name)"))); \
    addCommand("remove",							\
	       makeCommandVoid1(*this,					\
				(void (EntityClassName::*)(const std::string&))	\
				&stack_t::removeByTaskName,		\
				docCommandVoid1("Remove the named task.", \
						"string (task name)"))); \
    addCommand("rm",							\
	       makeCommandVoid1(*this,					\
				(void (EntityClassName::*)(const std::string&))	\
				&stack_t::removeByTaskName,		\
				docCommandVoid1("Remove the named task.", \
						"string (task name)"))); \
    addCommand("clear",							\
	       makeCommandVoid0(*this,					\
				(void (EntityClassName::*)(void))	\
				&stack_t::clear,			\
				docCommandVoid0("Clear the stack from all task."))); \
    addCommand("pop",							\
	       makeCommandVoid0(*this,					\
				(void (EntityClassName::*)(void))	\
				&stack_t::pop0,				\
				docCommandVoid0("Remove the last (lowest) task of the stack."))); \
									\
    addCommand("setSize",						\
	       makeCommandVoid1(*this,					\
				(void (EntityClassName::*)(const int&))	\
				&stack_t::defineNbDof,			\
				docCommandVoid1("Set the size of the solver.",		\
						"int")));		\
    addCommand("getSize",						\
	       makeDirectGetter(*this,&nbDofs,				\
				docDirectGetter("size","int")))



  } // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __sot_dyninv_StackTemplate_TCC__
