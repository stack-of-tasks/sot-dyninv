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
#include <sot-dyninv/mal-to-eigen.h>


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

	,CONSTRUCT_SIGNAL_IN(acceleration,ml::Vector)
	,CONSTRUCT_SIGNAL(velocity,OUT,ml::Vector)

      {
	Entity::signalRegistration( accelerationSIN << velocitySOUT );

	/* --- COMMANDS --- */
	addCommand("setVelocity",
		   makeDirectSetter(*this,&velocity_,
				    docDirectSetter("velocity","vector")));

      }

      void RobotDynSimu::
      display( std::ostream& os ) const
      {
	os << "RobotDynSimu, nothing more to say yet." << std::endl;
      }

      /* ---------------------------------------------------------------------- */
      void RobotDynSimu::
      integrate( const double & dt )
      {
	const ml::Vector & acceleration = accelerationSIN( controlSIN.getTime() );

	assert( velocity_.size() == state_.size()
		&& velocity_.size() == acceleration.size() );

	velocity_ += acceleration*dt;

	integrateRollPitchYaw(state_, velocity_, dt);
	for( unsigned int i=6;i<state_.size();++i )
	  { state_(i) += velocity_(i)*dt; }

	velocitySOUT.setConstant( velocity_);
      }

      void RobotDynSimu::
      setVelocity( const ml::Vector& v )
      {
	velocity_ = v;
	velocitySOUT.setConstant( velocity_);
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph
