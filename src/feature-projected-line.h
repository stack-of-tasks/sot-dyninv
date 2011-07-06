/*
 * Copyright 2011,
 * Nicolas Mansard
 * LAAS-CNRS
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

#ifndef __sot_dyninv_FeatureProjectedLine_H__
#define __sot_dyninv_FeatureProjectedLine_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/exception-task.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/exception-feature.hh>

#include <sot-dyninv/signal-helper.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_projected_line_EXPORTS)
#    define SOTFEATUREPROJECTEDLINE_EXPORT __declspec(dllexport)
#  else
#    define SOTFEATUREPROJECTEDLINE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFEATUREPROJECTEDLINE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
  namespace sot {
    namespace dyninv {

      /*!
	\class FeatureProjectedLine
      */
      class SOTFEATUREPROJECTEDLINE_EXPORT FeatureProjectedLine
	: public FeatureAbstract
      {
      public:
	static const std::string CLASS_NAME;
	virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

	/* --- SIGNALS ------------------------------------------------------------ */
      public:

	DECLARE_SIGNAL_IN(xa,MatrixHomogeneous);
	DECLARE_SIGNAL_IN(xb,MatrixHomogeneous);
	DECLARE_SIGNAL_IN(Ja,ml::Matrix);
	DECLARE_SIGNAL_IN(Jb,ml::Matrix);
	DECLARE_SIGNAL_IN(xc,ml::Vector);

	DECLARE_NO_REFERENCE;

      public:
	FeatureProjectedLine( const std::string& name );
	virtual ~FeatureProjectedLine( void ) {}

	virtual unsigned int& getDimension( unsigned int & dim, int time );
	virtual ml::Vector& computeError( ml::Vector& res,int time );
	virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );

	virtual void display( std::ostream& os ) const;

      } ;
    } /* namespace dyninv */
  } /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_PROJECTEDLINE_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
