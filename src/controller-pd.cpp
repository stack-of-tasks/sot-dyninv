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
#include <sot-core/debug.h>
#include <dynamic-graph/factory.h>

#include <sot-dyninv/commands-helper.h>

namespace sot
{
  namespace dyninv
  {


    namespace dg = ::dynamicgraph;
    using namespace dg;
    using ::dynamicgraph::command::makeDirectGetter;
    using ::dynamicgraph::command::docDirectGetter;
    using ::dynamicgraph::command::makeDirectSetter;
    using ::dynamicgraph::command::docDirectSetter;

    /* --- DG FACTORY ------------------------------------------------------- */
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ControllerPD,"ControllerPD");

    /* --- CONSTRUCTION ----------------------------------------------------- */
    /* --- CONSTRUCTION ----------------------------------------------------- */
    /* --- CONSTRUCTION ----------------------------------------------------- */
    ControllerPD::
    ControllerPD( const std::string & name )
      : Entity(name)

      ,CONSTRUCT_SIGNAL_IN(Kp,ml::Vector)
      ,CONSTRUCT_SIGNAL_IN(Kd,ml::Vector)
      ,CONSTRUCT_SIGNAL_IN(position,ml::Vector)
      ,CONSTRUCT_SIGNAL_IN(positionRef,ml::Vector)
      ,CONSTRUCT_SIGNAL_IN(velocity,ml::Vector)
      ,CONSTRUCT_SIGNAL_IN(velocityRef,ml::Vector)

      ,CONSTRUCT_SIGNAL_OUT(control,ml::Vector,ControllerPD,
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

      // std::string docstring
      // 		 = "\nSet the vectors dimension.\n\nInput:\n- an int.\nVoid return.\n\n";
      // addCommand("setSize",
      // 		 new ::dynamicgraph::command::Setter<ControllerPD,int>
      // 		 (*this, &ControllerPD::size,docstring));

      addCommand("setSize",
		 makeDirectSetter(*this, &_dimension,
				  docDirectSetter("dimension","int")));
    }


    /* --- SIGNALS ---------------------------------------------------------- */
    /* --- SIGNALS ---------------------------------------------------------- */
    /* --- SIGNALS ---------------------------------------------------------- */

    ml::Vector& ControllerPD::
    controlSOUT_function( ml::Vector &tau, int iter )
    {
      sotDEBUGIN(15);
      const ml::Vector& Kp = KpSIN(iter);
      const ml::Vector& Kd = KdSIN(iter);
      const ml::Vector& position = positionSIN(iter);
      const ml::Vector& desiredposition = positionRefSIN(iter);
      const unsigned int size = Kp.size();

      assert( _dimension == (int)size );
      assert( size==Kp.size() );        assert( size==Kd.size() );
      assert( size==position.size() );  assert( size==desiredposition.size() );

      bool useVelocity = velocitySIN;
      ml::Vector velocity;
      bool useVelocityDesired = false;
      ml::Vector desiredvelocity;
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

    void ControllerPD::
    commandLine( const std::string& cmdLine,
		 std::istringstream& cmdArgs,
		 std::ostream& os )
    {
      if( cmdLine == "help" )
	{
	  os << "sotControlPD:\n"
	     << " - size <arg>\t\tset the size of the vector.\n"
	     << " - stdGain \t\tset the input vector gains according to the size for HRP2.\n"
	     << " - velocityonly <arg>\t\tset Kp = 0.\n"
	     << std::endl;
	}
      else if( cmdLine == "size" )
	{
	  cmdArgs >> std::ws;
	  if( cmdArgs.good() )
	    {
	      unsigned int i; cmdArgs >> i ;
	      size(i);
	    }
	  else
	    {
	      os << "size = " << size() << std::endl;
	    }
	}
      else if( cmdLine == "velocityonly" )
	{
	  ml::Vector zero(_dimension); zero.fill(0);
	  positionSIN = zero;
	  positionRefSIN = zero;
	  KpSIN = zero;
	}
      else if( cmdLine == "stdGain" )
	{
	  std::string config = "high";
	  cmdArgs >> std::ws; if( cmdArgs.good() ) cmdArgs >> config;

	  if( config =="low" )
	    {
	      // Low
	      ml::Vector Kp(_dimension); Kp.fill(100);
	      ml::Vector Kd(_dimension); Kp.fill(20);
	      KpSIN = Kp;
	      KdSIN = Kd;
	    }
	  else if( config =="medium" )
	    {
	      // Medium gains
	      // Kp = [30](400,1000,2000,1800,2000,1000,400,1000,2000,1800,2000,1000,500,250,200,200,300,300,200,400,400,200,200,300, 300,200,400,400,200,200);
	      // Kd = [30] (40,100,200,180,200,100,40,100,200,180,200,100,50,25,20,20,30,30,20,40,40,20,20,30, 30,20,40,40,20,20);
	      if( _dimension != 30 )
		{ os << "Only working for dim=30!" << std::endl; return; }

	      ml::Vector Kp(_dimension),Kd(_dimension);
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
	      // Kp = [30](4000,10000,20000,18000,20000,10000,4000,10000,20000,18000,20000,10000,5000,2500,2000,2000,3000,3000,2000,4000,4000,2000,2000,3000, 3000,2000,4000,4000,2000,2000);
	      //Kd = [30](120,300,600,500,600,300,120,300,600,500,600,300,150,75,60,60,90,90,60,120,120,60,60,90, 90,60,120,120,60,60);

	      if( _dimension != 30 )
		{ os << "Only working for dim=30!" << std::endl; return; }

	      ml::Vector Kp(_dimension),Kd(_dimension);
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
      else
	{
	  Entity::commandLine(cmdLine,cmdArgs,os);
	}
    }






  } // namespace dyninv
} // namespace sot

