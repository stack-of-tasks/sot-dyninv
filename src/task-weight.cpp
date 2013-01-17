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


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

//#define VP_DEBUG
//#define VP_DEBUG_MODE 15
#include <sot/core/debug.hh>

#include <dynamic-graph/factory.h>
#include <sot-dyninv/task-weight.h>
#include <dynamic-graph/all-commands.h>
#include <sot-dyninv/commands-helper.h>
#include <boost/foreach.hpp>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {

      namespace dg = ::dynamicgraph;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskWeight,"TaskWeight");

      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      TaskWeight::
      TaskWeight( const std::string & name )
	: Task(name)
      {
	taskSOUT.setFunction( boost::bind(&TaskWeight::computeTask,this,_1,_2) );
	jacobianSOUT.setFunction( boost::bind(&TaskWeight::computeJacobian,this,_1,_2) );

	commandMap.erase("add");
	addCommand("add",
		   makeCommandVoid2(*this,&TaskWeight::addTask,
				    docCommandVoid2("Add a sub task with weight","task name","weight")));
	addCommand("setWeight",
		   makeCommandVoid2(*this,&TaskWeight::setWeight,
				    docCommandVoid2("Modify the weight of a task","task name","weight")));
      }

      /* ---------------------------------------------------------------------- */
      /* --- COMPUTATION ------------------------------------------------------ */
      /* ---------------------------------------------------------------------- */

      void TaskWeight::
      addTask(const std::string& name,const double & w)
      {
	Task& task =
	  dynamic_cast<Task&> (PoolStorage::getInstance()->getTask(name));
	TaskContener tw;
	tw.task = &task;
	tw.weight = w;

	taskList.push_back(tw);
      }

      void TaskWeight::
      setWeight(const std::string& name,const double & w)
      {
	BOOST_FOREACH(TaskContener& tc,taskList)
	  {
	    if( tc.task->getName() == name )
	      { tc.weight = w; }
	  }
      }

      dg::sot::VectorMultiBound& TaskWeight::
      computeTask( dg::sot::VectorMultiBound& res,int time )
      {
	int cursorError = 0;
	int dimError = res .size();
	if( 0==dimError ){ dimError = 1; res.resize(dimError); }

	for(   std::list< TaskContener >::iterator iter = taskList.begin();
	       iter!=taskList.end(); ++iter )
	  {
	    TaskContener & tw = *iter;
	    Task & task = *tw.task;
	    double wi = tw.weight;

	    const dg::sot::VectorMultiBound & partialTask = task.taskSOUT(time);
	    const int dim = partialTask.size();

	    while( cursorError+dim>dimError )  // DEBUG It was >=
	      { dimError *= 2; res.resize(dimError,false); }

	    for( int k=0;k<dim;++k )
	      {
		res[cursorError] = partialTask[k];
		res[cursorError].boundSingle *= wi;
		res[cursorError].boundSup *= wi;
		res[cursorError].boundInf *= wi;
		cursorError++;
	      }
	  }

	/* If too much memory has been allocated, resize. */
	res .resize(cursorError,false);
	return res;
      }

      ml::Matrix& TaskWeight::
      computeJacobian( ml::Matrix& res,int time )
      {
	int cursorError = 0;
	int dimJ = res .nbRows(), colJ = res.nbCols();
	if( 0==dimJ ){ dimJ = 1; res.resize(dimJ,colJ); }

	for(   std::list< TaskContener >::iterator iter = taskList.begin();
	       iter!=taskList.end(); ++iter )
	  {
	    TaskContener & tw = *iter;
	    Task & task = *tw.task;
	    double wi = tw.weight;


	    const ml::Matrix & partialJ = task.jacobianSOUT(time);
	    const int dim = partialJ.nbRows();
	    if(colJ==0) colJ=partialJ.nbCols();

	    while( cursorError+dim>dimJ )  // DEBUG It was >=
	      { dimJ *= 2; res.resize(dimJ,colJ,false); }

	    for( int k=0;k<dim;++k )
	      {
		for( int l=0;l<colJ; ++l )
		  res(cursorError,l) = partialJ(k,l) * wi;
		cursorError++;
	      }
	  }

	/* If too much memory has been allocated, resize. */
	res .resize(cursorError,colJ,false);
	return res;
      }


      /* ------------------------------------------------------------------------ */
      /* --- DISPLAY ENTITY ----------------------------------------------------- */
      /* ------------------------------------------------------------------------ */

      void TaskWeight::
      display( std::ostream& os ) const
      {
	os << "TaskWeight " << name << ": " << std::endl;
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

