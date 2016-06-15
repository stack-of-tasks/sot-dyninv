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

#include <sot/core/debug.hh>

#include <dynamic-graph/factory.h>
#include <sot-dyninv/robot-dyn-simu.h>
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
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RobotDynSimu,"RobotDynSimu");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      RobotDynSimu::
      RobotDynSimu( const std::string & name )
	: Device(name)

	,CONSTRUCT_SIGNAL_IN(acceleration,dg::Vector)
	,CONSTRUCT_SIGNAL_OUT(velocity,dg::Vector,sotNOSIGNAL)

      {
	Entity::signalRegistration( accelerationSIN << velocitySOUT );

	/* --- COMMANDS --- */
	addCommand("setVelocity",
		   makeDirectSetter(*this,&velocity_,
				    docDirectSetter("velocity","vector")));
	std::string docstring;
	/* Command increment. */
	docstring =
	  "\n"
	  "    Integrate dynamics for time step provided as input\n"
	  "\n"
	  "      take one floating point number as input\n"
	  "\n";
	addCommand("increment",
		   command::makeCommandVoid1((Device&)*this,
					     &Device::increment, docstring));

      }

      void RobotDynSimu::
      display( std::ostream& os ) const
      {
	os << "RobotDynSimu, nothing more to say yet." << std::endl;
      }

      dg::Vector& RobotDynSimu::
      velocitySOUT_function( dg::Vector& v, int )
      {
	if( velocity_.size()!=state_.size() )
	  {
	    velocity_.resize( state_.size() );
	    velocity_.fill(0.0);
	  }
	v = velocity_;
	return v;
      }

      /* ---------------------------------------------------------------------- */
      void RobotDynSimu::
      integrate( const double & dt )
      {
	const dg::Vector & acceleration = accelerationSIN( controlSIN.getTime() );

	if( velocity_.size()!=state_.size() )
	  {
	    velocity_.resize( state_.size() );
	    velocity_.fill(0.0);
	  }
	assert( velocity_.size() == state_.size()
		&& velocity_.size() == acceleration.size() );

	velocity_ += acceleration*dt;

	integrateRollPitchYaw(state_, velocity_, dt);
	for( unsigned int i=6;i<state_.size();++i )
	  { state_(i) += velocity_(i)*dt; }

	velocitySOUT.setReady();
      }

      void RobotDynSimu::
      setVelocity( const dg::Vector& v )
      {
	velocity_ = v;
	velocitySOUT.setReady();
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph
