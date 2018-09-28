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
#include <sot-dyninv/task-dyn-inequality.h>


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
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskDynInequality,"TaskDynInequality");

      /* ---------------------------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* ---------------------------------------------------------------------- */

      TaskDynInequality::
      TaskDynInequality( const std::string & name )
	: TaskDynPD(name)

	,CONSTRUCT_SIGNAL_IN(referenceInf,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(referenceSup,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(selec,Flags)

	  //,CONSTRUCT_SIGNAL_OUT(size,int,
	  //			      errorSOUT<<selecSIN)
	,sizeSOUT( boost::bind(&  TaskDynInequality::sizeSOUT_function,this,_1,_2),
		   errorSOUT<<selecSIN,getClassName()+"("+getName()+")::output(int)::size" )

      {
	taskSOUT.setFunction( boost::bind(&TaskDynInequality::computeTaskDyn,this,_1,_2) );
	taskSOUT.addDependency( selecSIN );
	taskSOUT.addDependency( referenceSupSIN );
	taskSOUT.addDependency( referenceInfSIN );
	taskSOUT.addDependency( sizeSOUT );

	jacobianSOUT.setFunction( boost::bind(&TaskDynInequality::computeJacobian,this,_1,_2) );
	jacobianSOUT.addDependency( selecSIN );
	jacobianSOUT.addDependency( sizeSOUT );

	controlGainSIN = 1.0;
	selecSIN = true;
	signalRegistration( referenceSupSIN << referenceInfSIN
			    << selecSIN << sizeSOUT );
      }

      /* ---------------------------------------------------------------------- */
      /* --- COMPUTATION ------------------------------------------------------ */
      /* ---------------------------------------------------------------------- */
      int& TaskDynInequality::
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

      dg::sot::VectorMultiBound& TaskDynInequality::
      computeTaskDyn( dg::sot::VectorMultiBound& res,int iter )
      {
	dg::Vector dummy;
	const bool withInf = referenceInfSIN, withSup = referenceSupSIN;
	MultiBound::SupInfType bound = withInf ? MultiBound::BOUND_INF : MultiBound::BOUND_SUP;

	const dg::Vector & error = errorSOUT(iter);
	const dg::Vector & errorDot = errorDotSOUT(iter);
	const dg::Vector & refInf = withInf ? referenceInfSIN(iter) : dummy;
	const dg::Vector & refSup = withSup ? referenceSupSIN(iter) : dummy;
	const Flags & selec = selecSIN(iter);
	const int insize = error.size(), outsize=sizeSOUT(iter);
	const double dt = dtSIN(iter)*controlGainSIN(iter), dt2 = 2/(dt*dt);
	assert( !withInf || insize==(int)refInf.size() );
	assert( !withSup || insize==(int)refSup.size() );

	// using std::cout; using std::endl;
	// int iterShow = 76;
	// if(iter==iterShow)
	//   {
	//     cout << "e = " << error << endl;
	//     cout << "edot = " << errorDot << endl;
	//     cout << "l = " << refInf << endl;
	//     cout << "u = " << refSup << endl;
	//   }
	res.resize(outsize); int idx=0;
	for( int i=0;i<insize;++i )
	  {
	    if( selec(i) )
	      {
		const double & e = error(i), & u = refSup(i),
		  & l = refInf(i), & ed = errorDot(idx);
		const double de = ed*dt;

		//const double EPS = 1e-4;
		//assert( e<=u+EPS && e>=l-EPS );

		double inf,sup;
		if(! withSup )                    { sup = 0; }
		else /*if( e+de/2<=u || e>u)*/	  { sup = (u-e-de)*dt2; }
		//else 		                  { sup = ed*ed/(2*(e-u)); }

		if(! withInf )	      	          { inf = 0; }
		else /*if(e<l || e+de/2>=l )*/	  { inf = (l-e-de)*dt2; }
		//else 		                  { inf = ed*ed/(2*(e-l)); }

		// if(i==0)
		//   {
		//     std::cout << inf << " " << sup;
		//     if( e+de/2>=l ) std::cout << " B";
		//     if( e<l ) std::cout << " C";
		//     std::cout << endl;
		//   }

		if( withInf && withSup )
		  res[idx++] = dg::sot::MultiBound( inf,sup );
		else
		  res[idx++] = dg::sot::MultiBound( inf+sup,bound );
	      }
	  }

	sotDEBUG(15) << "taskU = "<< res << std::endl;
	return res;
      }

      dg::Matrix& TaskDynInequality::
      computeJacobian( dg::Matrix& res,int iter )
      {
	const Flags & selec = selecSIN(iter);
	dg::Matrix Jin;  TaskDynPD::computeJacobian(Jin,iter);
	const int insize = Jin.rows(), outsize=sizeSOUT(iter), nbc=Jin.cols();
	assert( insize>=outsize );

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

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

