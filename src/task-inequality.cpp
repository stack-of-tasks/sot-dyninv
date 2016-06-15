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
#include <sot-dyninv/task-inequality.h>


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
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskInequality,"TaskInequality");

      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      TaskInequality::
      TaskInequality( const std::string & name )
	: Task(name)

	,CONSTRUCT_SIGNAL_IN(referenceInf,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(referenceSup,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(dt,double)
	,CONSTRUCT_SIGNAL_IN(selec,Flags)

	,CONSTRUCT_SIGNAL_OUT(normalizedPosition,dg::Vector,
			      errorSOUT<<referenceInfSIN<<referenceSupSIN)
	,CONSTRUCT_SIGNAL_OUT(size,int,
			      errorSOUT<<selecSIN)
      {
	taskSOUT.setFunction( boost::bind(&TaskInequality::computeTask,this,_1,_2) );
	taskSOUT.addDependency( selecSIN );
	taskSOUT.addDependency( dtSIN );
	taskSOUT.addDependency( referenceSupSIN );
	taskSOUT.addDependency( referenceInfSIN );
	taskSOUT.addDependency( sizeSOUT );

	jacobianSOUT.setFunction( boost::bind(&TaskInequality::computeJacobian,this,_1,_2) );
	jacobianSOUT.addDependency( selecSIN );
	jacobianSOUT.addDependency( sizeSOUT );

	controlGainSIN = 1.0;
	selecSIN = true;
	signalRegistration( referenceSupSIN << referenceInfSIN << dtSIN
			    << selecSIN << sizeSOUT << normalizedPositionSOUT );
      }

      /* ---------------------------------------------------------------------- */
      /* --- COMPUTATION ------------------------------------------------------ */
      /* ---------------------------------------------------------------------- */
      int& TaskInequality::
      sizeSOUT_function(int& res, int time)
      {
	const Flags & selec = selecSIN(time);
	const int size = errorSOUT(time).size();
	res=0;
	for( int i=0;i<size;++i )
	  {
	    if(selec(i)) res++;
	  }
	return res;
      }

      dg::sot::VectorMultiBound& TaskInequality::
      computeTask( dg::sot::VectorMultiBound& res,int time )
      {
	dg::Vector dummy;
	const bool withInf = referenceInfSIN, withSup = referenceSupSIN;
	MultiBound::SupInfType bound = withInf ? MultiBound::BOUND_INF : MultiBound::BOUND_SUP;

	const dg::Vector & error = errorSOUT(time);
	const dg::Vector & refInf = withInf ? referenceInfSIN(time) : dummy;
	const dg::Vector & refSup = withSup ? referenceSupSIN(time) : dummy;
	const Flags & selec = selecSIN(time);
	const int insize = error.size(), outsize=sizeSOUT(time);
	const double K = controlGainSIN(time)/dtSIN(time);
	assert( !withInf || insize==(int)refInf.size() );
	assert( !withSup || insize==(int)refSup.size() );

	res.resize(outsize); int idx=0;
	for( int i=0;i<insize;++i )
	  {
	    if( selec(i) )
	      {
		const double inf = withInf ? (refInf(i)-error(i))*K : 0.0;
		const double sup = withSup ? (refSup(i)-error(i))*K : 0.0;

		if( withInf && withSup )
		  res[idx++] = dg::sot::MultiBound( (refInf(i)-error(i))*K,
						    (refSup(i)-error(i))*K );
		else
		  res[idx++] = dg::sot::MultiBound( inf+sup,bound );
	      }
	  }

	sotDEBUG(15) << "taskU = "<< res << std::endl;
	return res;
      }

      dg::Matrix& TaskInequality::
      computeJacobian( dg::Matrix& res,int time )
      {
	const Flags & selec = selecSIN(time);
	dg::Matrix Jin;  Task::computeJacobian(Jin,time);
	const int insize = Jin.rows(), outsize=sizeSOUT(time), nbc=Jin.cols();
	//assert( );

	res.resize(outsize,nbc); int idx=0;
	for( int i=0;i<insize;++i )
	  {
	    if( selec(i) )
	      {
		for( int j=0;j<nbc;++j )
		  res(idx,j) = Jin(i,j);
		idx++;
	      }
	  }

	return res;
      }

      dg::Vector& TaskInequality::
      normalizedPositionSOUT_function( dg::Vector& res, int time )
      {
	const dg::Vector & error = errorSOUT(time);
	const dg::Vector & refInf = referenceInfSIN(time);
	const dg::Vector & refSup = referenceSupSIN(time);
	const Flags & selec = selecSIN(time);
	const int insize = error.size(), outsize=sizeSOUT(time);
	assert( insize==(int)refInf.size() && insize==(int)refSup.size() );

	res.resize( outsize ); int idx=0;
	for( int i=0;i<insize;++i )
	  { if(selec(i))
	      res(idx++) = ( error(i)-refInf(i) ) / ( refSup(i)-refInf(i) ); }
	return res;
      }

      /* ------------------------------------------------------------------------ */
      /* --- DISPLAY ENTITY ----------------------------------------------------- */
      /* ------------------------------------------------------------------------ */

      void TaskInequality::
      display( std::ostream& os ) const
      {
	os << "TaskInequality " << name << ": " << std::endl;
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

