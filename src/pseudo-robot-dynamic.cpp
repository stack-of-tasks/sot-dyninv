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

#include <sot-dyninv/pseudo-robot-dynamic.h>
#include <sot/core/debug.hh>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/eigen-io.h>

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
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PseudoRobotDynamic,"PseudoRobotDynamic");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      PseudoRobotDynamic::
      PseudoRobotDynamic( const std::string & name )
	: DynamicIntegrator(name)

	,CONSTRUCT_SIGNAL_IN(control,dg::Vector)
	,CONSTRUCT_SIGNAL_OUT(qdot,dg::Vector,controlSIN)

	,CONSTRUCT_SIGNAL(rotation,OUT,dg::Vector)
	,CONSTRUCT_SIGNAL(translation,OUT,dg::Vector)
	,stateSOUT( &positionSOUT,getClassName()+"("+getName()+")::output(vector)::state" )

	,formerOpenHRP( NULL )
      {
	Entity::signalRegistration( controlSIN << qdotSOUT
				    << rotationSOUT << translationSOUT
				    << stateSOUT );

	/* --- COMMANDS --- */
	std::string doc
	  = docCommandVoid2("Replace in the pool a robot entity by this,"
			    "and modify the plugs (if 2de arg is true).",
			    "string (entity name)","bool (plug)");
	addCommand("replace",
		   makeCommandVoid2(*this,&PseudoRobotDynamic::replaceSimulatorEntity,
				    doc));
	doc = docCommandVoid1("Set the root position, and forward to the real simulator.",
			      "matrix homogeneous");
	addCommand("setRoot",
		   makeCommandVoid1(*this,&PseudoRobotDynamic::setRoot,
				    doc));

	doc = docCommandVoid1("Add to the current entity the command of the previous simu.",
			      "string (command name)");
	addCommand("addForward",
		   makeCommandVoid1(*this,&PseudoRobotDynamic::addForward,
				    doc));

      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      /* Force the recomputation of the DynInt::controlSIN, and one step
       * of the integrator. Then, export the resulting joint velocity,
       * along with the 6D position of the free floating through signals
       * rotationSOUT and translationSOUT.
       */
      dg::Vector& PseudoRobotDynamic::
      qdotSOUT_function( dg::Vector& mlqdot, int time )
      {
	sotDEBUGIN(5);

	controlSIN(time);
	integrateFromSignals(time);

	const Eigen::VectorXd v = velocity;
	mlqdot = v.tail(v.size()-6);

	rotationSOUT = position.segment<3>(3);
	translationSOUT = position.head<3>();

	sotDEBUGOUT(5);
	return mlqdot;
      }

      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */

      void PseudoRobotDynamic::
      replaceSimulatorEntity( const std::string& formerName, const bool & plug )
      {
	assert( formerOpenHRP == NULL );
	formerOpenHRP = &dg::g_pool().getEntity( formerName );
	dg::g_pool().deregisterEntity( formerName );
	// former.name = formerName+".old";
	dg::g_pool().registerEntity( formerName+"-old",formerOpenHRP );

	entityDeregistration(); name = formerName; entityRegistration();

	if( plug )
	  {
	    formerOpenHRP->getSignal("control").plug( & qdotSOUT );

	    try
	      {
		formerOpenHRP->getSignal("attitudeIN").plug( & rotationSOUT );
		formerOpenHRP->getSignal("translation").plug( & translationSOUT );
	      }
	    catch (...) {}

	    const dg::Vector& pos
	      = dynamic_cast< dg::Signal<dg::Vector,int>& >
	      ( formerOpenHRP->getSignal("state") ).accessCopy();
	    try
	      {
		const dg::Vector& vel
		  = dynamic_cast< dg::Signal<dg::Vector,int>& >
		  ( formerOpenHRP->getSignal("velocity") ).accessCopy();
		setState(pos,vel);
	      }
	    catch (... )
	      {
		dg::Vector velzero( pos.size() ); velzero.setZero();
		setState(pos,velzero);
	      }
	  }
      }
      void PseudoRobotDynamic::
      setRoot( const dg::Matrix & mlM )
      {
	sotDEBUG(15) << "Set root with " << mlM << std::endl;
	using namespace Eigen;

	const Eigen::MatrixXd M = mlM;
	Eigen::Vector3d r = (M.topLeftCorner<3,3>().eulerAngles(2,1,0)).reverse();
	Eigen::VectorXd q = position;
	if( q.size()<6 )
	  {
	    throw; // TODO
	  }
	q.head<3>() = M.col(3).head<3>();
	q.segment<3>(3) = r;

	if( formerOpenHRP )
	  try
	    {
	      forwardVoidCommandToSimu( "setRoot",mlM );
	    }
	  catch(...) {}
      }


      /* --- FORWARD ---------------------------------------------------------- */

      /* This function is a forward on the command on the former simu entity
       * <formerOpenHRP>: when calling it, the parameters are send to the named
       * command, that is executed.
       */
      template< typename T1 >
      void PseudoRobotDynamic::
      forwardVoidCommandToSimu( const std::string& cmdName,
				const T1& arg1 )
      {
	/* Forward to previous entity. */
	assert( formerOpenHRP );

	using dg::command::Command;
	using dg::command::Value;

	Command* command = formerOpenHRP->getNewStyleCommand(cmdName);
	std::vector<Value> valuesArg;
	valuesArg.push_back( Value( arg1 ) );
	command->setParameterValues(valuesArg);
	command->execute();
      }

      /* Add the named command of the former simu entity <formerOpenHRP> to the
       * commands of the entity.
       */
      void PseudoRobotDynamic::
      addForward( const std::string& cmdName )
      {
	assert( formerOpenHRP );
	addCommand(cmdName,formerOpenHRP->getNewStyleCommand(cmdName));
      }


      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */

      void PseudoRobotDynamic::
      display( std::ostream& os ) const
      {
	os << "PseudoRobotDynamic "<<getName() << ". " << std::endl;
      }
    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

