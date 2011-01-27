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
#include <sot-core/debug.h>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>

#include <sot-dyninv/commands-helper.h>
#include <sot-dyninv/mal-to-eigen.h>


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

      ,CONSTRUCT_SIGNAL_IN(control,ml::Vector)
      ,CONSTRUCT_SIGNAL_OUT(qdot,ml::Vector,controlSIN)

      ,CONSTRUCT_SIGNAL(rotation,OUT,ml::Vector)
      ,CONSTRUCT_SIGNAL(translation,OUT,ml::Vector)
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
    ml::Vector& PseudoRobotDynamic::
    qdotSOUT_function( ml::Vector& mlqdot, int time )
    {
      sotDEBUGIN(5);

      controlSIN(time);
      integrateFromSignals(time);

      EIGEN_VECTOR_FROM_SIGNAL(v,velocity );
      EIGEN_VECTOR_FROM_VECTOR( qdot,mlqdot,v.size()-6 );
      qdot = v.tail(v.size()-6);

      EIGEN_VECTOR_FROM_SIGNAL(p,position );
      {
	ml::Vector mlv3;
	EIGEN_VECTOR_FROM_VECTOR( r,mlv3,3 );
	r = p.segment(3,3);
	rotationSOUT = mlv3;
	r = p.head(3);
	translationSOUT = mlv3;
      }

      sotDEBUGOUT(5);
      return mlqdot;
    }

    /* --- TOOLS ------------------------------------------------------------- */
    /* --- TOOLS ------------------------------------------------------------- */
    /* --- TOOLS ------------------------------------------------------------- */

    namespace PseudoRobotDynamic_Static
    {
      using namespace Eigen;

      template< typename D1 >
      Vector3d computeEulerFromRotationMatrix ( const MatrixBase<D1> & rotation )
      {
	Vector3d euler;

	double rotationMatrix00 = rotation(0,0);
	double rotationMatrix10 = rotation(1,0);
	double rotationMatrix20 = rotation(2,0);
	double rotationMatrix01 = rotation(0,1);
	double rotationMatrix11 = rotation(1,1);
	double rotationMatrix21 = rotation(2,1);
	double rotationMatrix02 = rotation(0,2);
	double rotationMatrix12 = rotation(1,2);
	double rotationMatrix22 = rotation(2,2);

	double cosTheta = sqrt(0.5 * ( rotationMatrix00*rotationMatrix00
				       + rotationMatrix10*rotationMatrix10
				       + rotationMatrix21*rotationMatrix21
				       + rotationMatrix22*rotationMatrix22 ));
	double sinTheta = -rotationMatrix20;
	euler[1] = atan2 (sinTheta, cosTheta);

	double cosTheta_cosPhi = 0.5 * (rotationMatrix22 * rotationMatrix11
					- rotationMatrix21 * rotationMatrix12);
	double cosTheta_sinPhi = 0.5 * (rotationMatrix21 * rotationMatrix02
					- rotationMatrix22 * rotationMatrix01);
	double cosTheta_cosPsi = 0.5 * (rotationMatrix00 * rotationMatrix11
					- rotationMatrix01 * rotationMatrix10);
	double cosTheta_sinPsi = 0.5 * (rotationMatrix10 * rotationMatrix02
					- rotationMatrix00 * rotationMatrix12);

	//if cosTheta == 0
	if (fabs(cosTheta) < 1e-9 )
	  {
	    if (sinTheta > 0.5) // sinTheta ~= 1
	      {
		//phi_psi = phi - psi
		double phi_psi = atan2(- rotationMatrix10, rotationMatrix11);
		double psi = euler[2];

		double phi = phi_psi + psi;
		euler[0] = phi;
	      }
	    else  //sinTheta  ~= -1
	      {
		//phi_psi = phi + psi
		double phi_psi = atan2(- rotationMatrix10,  rotationMatrix11);

		double psi = euler[2];

		double phi = phi_psi;
		euler[0] = phi - psi;
	      }
	  }
	else
	  {
	    double cosPsi = cosTheta_cosPsi / cosTheta;
	    double sinPsi = cosTheta_sinPsi / cosTheta;
	    euler[0] = atan2 (sinPsi, cosPsi);

	    double cosPhi = cosTheta_cosPhi / cosTheta;
	    double sinPhi = cosTheta_sinPhi / cosTheta;
	    euler[2] = atan2 (sinPhi, cosPhi);
	  }

	return euler;
      }

    } // namespace PseudoRobotDynamic_Static

    /* --- COMMANDS ---------------------------------------------------------- */
    /* --- COMMANDS ---------------------------------------------------------- */
    /* --- COMMANDS ---------------------------------------------------------- */


    void PseudoRobotDynamic::
    replaceSimulatorEntity( const std::string& formerName, const bool & plug )
    {
      assert( formerOpenHRP == NULL );
      formerOpenHRP = &dg::g_pool.getEntity( formerName );
      dg::g_pool.deregisterEntity( formerName );
      // former.name = formerName+".old";
      dg::g_pool.registerEntity( formerName+"-old",formerOpenHRP );

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

	  const ml::Vector& pos
	    = dynamic_cast< dg::Signal<ml::Vector,int>& >
	    ( formerOpenHRP->getSignal("state") ).accessCopy();
	  try
	    {
	      const ml::Vector& vel
		= dynamic_cast< dg::Signal<ml::Vector,int>& >
		( formerOpenHRP->getSignal("velocity") ).accessCopy();
	      setState(pos,vel);
	    }
	  catch (... )
	    {
	      ml::Vector velzero( pos.size() ); velzero.setZero();
	      setState(pos,velzero);
	    }
	}
    }
    void PseudoRobotDynamic::
    setRoot( const ml::Matrix & mlM )
    {
      sotDEBUG(15) << "Set root with " << mlM << std::endl;
      using namespace Eigen;
      using PseudoRobotDynamic_Static::computeEulerFromRotationMatrix;

      EIGEN_MATRIX_FROM_SIGNAL(M,mlM);
      Vector3d r = computeEulerFromRotationMatrix( M.topLeftCorner(3,3) );
      EIGEN_VECTOR_FROM_SIGNAL( q,position );
      if( q.size()<6 )
	{
	  throw; // TODO
	}
      q.head(3) = M.col(3).head(3);
      q.segment(3,3) = r;

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

    void PseudoRobotDynamic::
    commandLine( const std::string& cmdLine,
		 std::istringstream& cmdArgs,
		 std::ostream& os )
    {
      sotDEBUGIN(15);

      if( cmdLine == "help" )
	{
	  os << "PseudoRobotDynamic:" << std::endl
	     << " - replace <OpenHRP> [plug]" << std::endl;
	}
      else if( cmdLine == "replace" )
	{
	  if( cmdArgs >> std::ws, cmdArgs.good() )
	    {
	      std::string repName; cmdArgs >> repName >> std::ws;
	      bool plug = cmdArgs.good();
	      replaceSimulatorEntity( repName,plug );
	    }
	}
      else if( cmdLine=="sbs" || cmdLine=="play" || cmdLine =="withForces"
	       || cmdLine == "periodicCall" || cmdLine == "periodicCallBefore"
	       || cmdLine == "periodicCallAfter"  )
	{
	  formerOpenHRP ->commandLine( cmdLine,cmdArgs,os );
	}
      else if( cmdLine=="root" )
	{
	  if( cmdArgs >> std::ws, cmdArgs.good() )
	    {
	      ml::Matrix M; cmdArgs >> M; setRoot(M);
	      std::ostringstream osback;
	      osback << M.accessToMotherLib(); cmdArgs.str(osback.str());
	      formerOpenHRP ->commandLine( cmdLine,cmdArgs,os );
	    }
	  else
	    {
	      os << "TODO" << std::endl;
	    }
	}
      else
	{
	  Entity::commandLine( cmdLine,cmdArgs,os );
	}
      sotDEBUGOUT(15);
    }

  } // namespace dyninv
} // namespace sot

