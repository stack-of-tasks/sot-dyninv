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

#include <sot-dyninv/controller-pd.h>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <iostream>
#include <sot-dyninv/commands-helper.h>

namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {


      namespace dg = ::dynamicgraph;
      using namespace dg;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ControllerPD,"ControllerPD");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      ControllerPD::
      ControllerPD( const std::string & name )
	: Entity(name)

	,CONSTRUCT_SIGNAL_IN(Kp,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(Kd,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(position,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(positionRef,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(velocity,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(velocityRef,dg::Vector)

	,CONSTRUCT_SIGNAL_OUT(control,dg::Vector,
			      KpSIN << KdSIN << positionSIN << positionRefSIN
			      << velocitySIN << velocityRefSIN )
      {
	Entity::signalRegistration( KpSIN << KdSIN << positionSIN << positionRefSIN
				    << velocitySIN << velocityRefSIN
				    << controlSOUT );

	/* Commands. */
	addCommand("getSize",
		   makeDirectGetter(*this,&_dimension,
				    docDirectGetter("dimension","int")));

	addCommand("setSize",
		   makeDirectSetter(*this, &_dimension,
				    docDirectSetter("dimension","int")));

	addCommand("velocityOnly",
		   makeCommandVoid0(*this,&ControllerPD::setGainVelocityOnly,
				    docCommandVoid0("Set a friction-style controller.")));

	addCommand("stdGain",
		   makeCommandVoid1(*this,&ControllerPD::setStandardGains,
				    docCommandVoid1("Set spefic gains.",
						    "string (in low|medium|high)")));

      }


      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      dg::Vector& ControllerPD::
      controlSOUT_function( dg::Vector &tau, int iter )
      {
	sotDEBUGIN(15);
	const dg::Vector& Kp = KpSIN(iter);
	const dg::Vector& Kd = KdSIN(iter);
	const dg::Vector& position = positionSIN(iter);
	const dg::Vector& desiredposition = positionRefSIN(iter);
	const unsigned int size = Kp.size();

	assert( _dimension == (int)size );
	assert( size==Kp.size() );        assert( size==Kd.size() );
	assert( size==position.size() );  assert( size==desiredposition.size() );

	bool useVelocity = velocitySIN;
	dg::Vector velocity;
	bool useVelocityDesired = false;
	dg::Vector desiredvelocity;
	if( useVelocity ) // TODO: there is a useless copy here. Use a pointer?
	  {
	    velocity = velocitySIN(iter);
	    assert( size==velocity.size() );
	    useVelocityDesired = velocityRefSIN;
	    if( useVelocityDesired )
	      {
		desiredvelocity = velocityRefSIN(iter);
		assert( size==desiredvelocity.size() );
	      }
	  }

	tau.resize(size); double dv; // TODO: use dv from start to avoid the copy.
	for(unsigned i = 0u; i < size; ++i)
	  {
	    tau(i) = Kp(i)*(desiredposition(i)-position(i));
	    if( useVelocity )
	      {
		dv= velocity(i);
		if( useVelocityDesired )
		  dv-=desiredvelocity(i);
		tau(i) -= Kd(i)*dv;
	      }
	  }

	sotDEBUG(15) << "p = " << position << std::endl;
	sotDEBUG(15) << "pd = " << desiredposition << std::endl;
	if( useVelocity ) {sotDEBUG(15) << "v= " << velocity << std::endl;}
	if( useVelocityDesired ) { sotDEBUG(15) << "vd= " << velocity << std::endl; }
	sotDEBUG(15) << "kp = " << Kp << std::endl;
	sotDEBUG(15) << "kd = " << Kd << std::endl;
	sotDEBUG(15) << "PD torque= " << tau << std::endl;

	sotDEBUGOUT(15);
	return tau;
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- MEMBERS ---------------------------------------------------------- */
      void ControllerPD::
      size(const int & dimension)
      {
	_dimension = dimension;
      }

      int ControllerPD::
      size(void) const
      {
	return _dimension;
      }

      void ControllerPD::
      setGainVelocityOnly( void )
      {
	dg::Vector zero(_dimension); zero.fill(0);
	positionSIN = zero;
	positionRefSIN = zero;
	KpSIN = zero;
      }

      /** Give some specific values for the Kp and Kd gains. Possible
       * configs are "low", "middle" and "high".
       * Warning: middle and high only works for dim 30.
       * TODO: properly throw errors when needed.
       */
      void ControllerPD::
      setStandardGains( const std::string & config )
      {

	if( config =="low" )
	  {
	    // Low gains
	    dg::Vector Kp(_dimension); Kp.fill(100);
	    dg::Vector Kd(_dimension); Kd.fill(20);
	    KpSIN = Kp;
	    KdSIN = Kd;
	  }
	else if( config =="medium" )
	  {
	    // Medium gains
	    if( _dimension != 30 )
	      { std::cerr << "Only working for dim=30!" << std::endl; return; }

	    dg::Vector Kp(_dimension),Kd(_dimension);
	    unsigned int i=0;
	    Kp(i++)=400;   Kp(i++)=1000; Kp(i++)=2000;  Kp(i++)=1800;  Kp(i++)=2000;
	    Kp(i++)=1000;  Kp(i++)=400;  Kp(i++)=1000;  Kp(i++)=2000;  Kp(i++)=1800;
	    Kp(i++)=2000;  Kp(i++)=1000; Kp(i++)=500;  Kp(i++)=250;  Kp(i++)=200;
	    Kp(i++)=200;   Kp(i++)=300;  Kp(i++)=300;  Kp(i++)=200;  Kp(i++)=400;
	    Kp(i++)=400;   Kp(i++)=200;  Kp(i++)=200;  Kp(i++)=300;  Kp(i++)= 300;
	    Kp(i++)=200;   Kp(i++)=400;  Kp(i++)=400;  Kp(i++)=200;  Kp(i++)=200;

	    i=0;
	    Kd(i++)=40;  Kd(i++)=100;  Kd(i++)=200;  Kd(i++)=180;  Kd(i++)=200;
	    Kd(i++)=100; Kd(i++)=40;   Kd(i++)=100;  Kd(i++)=200;  Kd(i++)=180;
	    Kd(i++)=200; Kd(i++)=100;  Kd(i++)=50;   Kd(i++)=25;   Kd(i++)=20;
	    Kd(i++)=20;  Kd(i++)=30;   Kd(i++)=30;   Kd(i++)=20;   Kd(i++)=40;
	    Kd(i++)=40;  Kd(i++)=20;   Kd(i++)=20;   Kd(i++)=30;   Kd(i++)= 30;
	    Kd(i++)=20;  Kd(i++)=40;   Kd(i++)=40;   Kd(i++)=20;   Kd(i++)=20;

	    KpSIN = Kp;
	    KdSIN = Kd;
	  }
	else // high
	  {
	    // High gains
	    if( _dimension != 30 )
	      { std::cerr << "Only working for dim=30!" << std::endl; return; }

	    dg::Vector Kp(_dimension),Kd(_dimension);
	    unsigned int i=0;
	    Kp(i++)=4000;  Kp(i++)=10000; Kp(i++)=20000; Kp(i++)=18000;  Kp(i++)=20000;
	    Kp(i++)=10000; Kp(i++)=4000;  Kp(i++)=10000; Kp(i++)=20000;  Kp(i++)=18000;
	    Kp(i++)=20000; Kp(i++)=10000; Kp(i++)=5000;  Kp(i++)=2500;   Kp(i++)=2000;
	    Kp(i++)=2000;  Kp(i++)=3000;  Kp(i++)=3000;  Kp(i++)=2000;   Kp(i++)=4000;
	    Kp(i++)=4000;  Kp(i++)=2000;  Kp(i++)=2000;  Kp(i++)=3000;   Kp(i++)= 3000;
	    Kp(i++)=2000;  Kp(i++)=4000;  Kp(i++)=4000;  Kp(i++)=2000;   Kp(i++)=2000;

	    i=0;
	    Kd(i++)=120;   Kd(i++)=300;   Kd(i++)=600;   Kd(i++)=500;    Kd(i++)=600;
	    Kd(i++)=300;   Kd(i++)=120;   Kd(i++)=300;   Kd(i++)=600;    Kd(i++)=500;
	    Kd(i++)=600;   Kd(i++)=300;   Kd(i++)=150;   Kd(i++)=75;     Kd(i++)=60;
	    Kd(i++)=60;    Kd(i++)=90;    Kd(i++)=90;    Kd(i++)=60;     Kd(i++)=120;
	    Kd(i++)=120;   Kd(i++)=60;    Kd(i++)=60;    Kd(i++)=90;     Kd(i++)= 90;
	    Kd(i++)=60;    Kd(i++)=120;   Kd(i++)=120;   Kd(i++)=60;     Kd(i++)=60;

	    KpSIN = Kp;
	    KdSIN = Kd;
	  }
      }
      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */

      void ControllerPD::
      display( std::ostream& os ) const
      {
	os << "ControllerPD "<<getName();
	try
	  {
	    os <<"control = "<<controlSOUT;
	  }
	catch (ExceptionSignal e) {}
      }
    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

