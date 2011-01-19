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
#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>

#include <jrl/mal/matrixabstractlayer.hh>

namespace sot
{
  namespace dyninv
  {

    using namespace dg;

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ControllerPD,"ControllerPD");

    ControllerPD::
    ControllerPD( const std::string & name )
      :Entity(name)
      ,sensorWorldRotationSIN(NULL,"sotControllerPD("+name
			      +")::input(MatrixRotation)::sensorWorldRotation")
      ,sensorEmbeddedPositionSIN(NULL,"sotControllerPD("+name
				 +")::input(MatrixHomo)::sensorEmbeddedPosition")
      ,contactWorldPositionSIN(NULL,"sotControllerPD("+name
			       +")::input(MatrixHomo)::contactWorldPosition")
      ,contactEmbeddedPositionSIN(NULL,"sotControllerPD("+name
				  +")::input(MatrixHomo)::contactEmbeddedPosition")

      ,anglesSOUT( boost::bind(&ControllerPD::computeAngles,this,_1,_2),
		   sensorWorldRotationSIN<<sensorEmbeddedPositionSIN
		   <<contactWorldPositionSIN<<contactEmbeddedPositionSIN,
		   "sotControllerPD("+name+")::output(Vector)::angles" )
      ,flexibilitySOUT( boost::bind(&ControllerPD::computeFlexibilityFromAngles
				    ,this,_1,_2),
			anglesSOUT,
			"sotControllerPD("+name+")::output(matrixRotation)::flexibility" )
      ,driftSOUT( boost::bind(&ControllerPD::computeDriftFromAngles,this,_1,_2),
		  anglesSOUT,
		  "sotControllerPD("+name+")::output(matrixRotation)::drift" )
      ,sensorWorldRotationSOUT( boost::bind(&ControllerPD::computeSensorWorldRotation
					    ,this,_1,_2),
				anglesSOUT<<sensorWorldRotationSIN,
				"sotControllerPD("+name
				+")::output(matrixRotation)::sensorCorrectedRotation" )
      ,waistWorldRotationSOUT( boost::bind(&ControllerPD::computeWaistWorldRotation
					   ,this,_1,_2),
			       sensorWorldRotationSOUT<<sensorEmbeddedPositionSIN,
			       "sotControllerPD("+name
			       +")::output(matrixRotation)::waistWorldRotation" )
      ,waistWorldPositionSOUT( boost::bind(&ControllerPD::computeWaistWorldPosition
					   ,this,_1,_2),
			       flexibilitySOUT << contactEmbeddedPositionSIN,
			       "sotControllerPD("+name
			       +")::output(MatrixHomogeneous)::waistWorldPosition" )
      ,waistWorldPoseRPYSOUT( boost::bind(&ControllerPD::computeWaistWorldPoseRPY
					  ,this,_1,_2),
			      waistWorldPositionSOUT,
			      "sotControllerPD("+name
			      +")::output(vectorRollPitchYaw)::waistWorldPoseRPY" )

      ,fromSensor_(true)
    {
      sotDEBUGIN(5);

      signalRegistration( sensorWorldRotationSIN     << sensorEmbeddedPositionSIN
			  << contactWorldPositionSIN << contactEmbeddedPositionSIN
			  << anglesSOUT              << flexibilitySOUT
			  << driftSOUT               << sensorWorldRotationSOUT
			  << waistWorldRotationSOUT
			  << waistWorldPositionSOUT  << waistWorldPoseRPYSOUT  );

      // Commands
      std::string docstring;
      docstring = "    \n"
	"    Set flag specifying whether angle is measured from sensors or simulated.\n"
	"    \n"
	"      Input:\n"
	"        - a boolean value.\n"
	"    \n";
      addCommand("setFromSensor",
		 new ::dynamicgraph::command::Setter<ControllerPD, bool>
		 (*this, &ControllerPD::fromSensor, docstring));

      docstring = "    \n"
	"    Get flag specifying whether angle is measured from sensors or simulated.\n"
	"    \n"
	"      No input,\n"
	"      return a boolean value.\n"
	"    \n";
      addCommand("getFromSensor",
		 new ::dynamicgraph::command::Getter<ControllerPD, bool>
		 (*this, &ControllerPD::fromSensor, docstring));

      sotDEBUGOUT(5);
    }


    ControllerPD::
    ~ControllerPD( void )
    {
      sotDEBUGIN(5);

      sotDEBUGOUT(5);
      return;
    }

    /* --- SIGNALS -------------------------------------------------------------- */
    /* --- SIGNALS -------------------------------------------------------------- */
    /* --- SIGNALS -------------------------------------------------------------- */
    ml::Vector& ControllerPD::
    computeAngles( ml::Vector& res,
		   const int& time )
    {
      sotDEBUGIN(15);

      res.resize(3);

      const MatrixRotation &worldestRchest = sensorWorldRotationSIN( time );
      sotDEBUG(35) << "worldestRchest = " << std::endl << worldestRchest;
      const MatrixHomogeneous &waistMchest = sensorEmbeddedPositionSIN( time );
      MatrixRotation waistRchest; waistMchest.extract( waistRchest );

      const MatrixHomogeneous & waistMleg = contactEmbeddedPositionSIN( time );
      MatrixRotation waistRleg; waistMleg.extract( waistRleg );
      MatrixRotation chestRleg; waistRchest.transpose().multiply( waistRleg,chestRleg );
      MatrixRotation worldestRleg; worldestRchest.multiply( chestRleg,worldestRleg );

      sotDEBUG(35) << "worldestRleg = " << std::endl << worldestRleg;

      /* Euler angles with following code: (-z)xy, -z being the yaw drift, x the
       * first flexibility and y the second flexibility. */
      const double TOLERANCE_TH = 1e-6;

      const MatrixRotation &R = worldestRleg;
      if( (fabs(R(0,1))<TOLERANCE_TH)&&(fabs(R(1,1))<TOLERANCE_TH) ) 
	{
	  /* This means that cos(X) is very low, ie flex1 is close to 90deg.
	   * I take the case into account, but it is bloody never going 
	   * to happens. */
	  if( R(2,1)>0 ) res(0)=M_PI/2; else res(0) = -M_PI/2;
	  res(2) = atan2( -R(0,2),R(0,0) );
	  res(1) = 0;

	  /* To sum up: if X=PI/2, then Y and Z are in singularity.
	   * we cannot decide both of then. I fixed Y=0, which
	   * means that all the measurement coming from the sensor
	   * is assumed to be drift of the gyro. */
	}
      else
	{
	  double &X = res(0);
	  double &Y = res(1);
	  double &Z = res(2);

	  Y = atan2( R(2,0),R(2,2) );
	  Z = atan2( R(0,1),R(1,1) );
	  if( fabs(R(2,0))>fabs(R(2,2)) )
	    { X=atan2( R(2,1)*sin(Y),R(2,0) ); }
	  else 
	    { X=atan2( R(2,1)*cos(Y),R(2,2) ); }
	}
  
      sotDEBUG(35) << "angles = " << res;


      sotDEBUGOUT(15);
      return res;
    }

    /* compute the transformation matrix of the flexibility, ie
     * feetRleg. 
     */
    MatrixRotation& ControllerPD::
    computeFlexibilityFromAngles( MatrixRotation& res,
				  const int& time )
    {
      sotDEBUGIN(15);

      const ml::Vector & angles = anglesSOUT( time );
      double cx= cos( angles(0) );
      double sx= sin( angles(0) );
      double cy= cos( angles(1) );
      double sy= sin( angles(1) );
  
      res(0,0) = cy;
      res(0,1) = 0;
      res(0,2) = -sy;

      res(1,0) = -sx*sy;
      res(1,1) = cx;
      res(1,2) = -sx*cy;

      res(2,0) = cx*sy;
      res(2,1) = sx;
      res(2,2) = cx*cy;

      sotDEBUGOUT(15);
      return res;
    }

    /* Compute the rotation matrix of the drift, ie the
     * transfo from the world frame to the estimated (drifted) world
     * frame: worldRworldest.
     */
    MatrixRotation& ControllerPD::
    computeDriftFromAngles( MatrixRotation& res,
			    const int& time )
    {
      sotDEBUGIN(15);
  
      const ml::Vector & angles = anglesSOUT( time );
      double cz = cos( angles(2) );
      double sz = sin( angles(2) );
  
      res(0,0) = cz;
      res(0,1) = -sz;
      res(0,2) = 0;

      // z is the positive angle (R_{-z} has been computed
      // in the <angles> function). Thus, the /-/sin(z) is in 0,1
      res(1,0) = sz;  
      res(1,1) = cz;
      res(1,2) = 0;

      res(2,0) = 0;
      res(2,1) = 0;
      res(2,2) = 1;

      sotDEBUGOUT(15);
      return res;
    }

    MatrixRotation& ControllerPD::
    computeSensorWorldRotation( MatrixRotation& res,
				const int& time )
    {
      sotDEBUGIN(15);
  
      const MatrixRotation & worldRworldest = driftSOUT( time );
      const MatrixRotation & worldestRsensor = sensorWorldRotationSIN( time );
  
      worldRworldest.multiply( worldestRsensor,res );

      sotDEBUGOUT(15);
      return res;
    }
 
    MatrixRotation& ControllerPD::
    computeWaistWorldRotation( MatrixRotation& res,
			       const int& time )
    {
      sotDEBUGIN(15);
  
      // chest = sensor
      const MatrixRotation & worldRsensor = sensorWorldRotationSOUT( time );
      const MatrixHomogeneous & waistMchest = sensorEmbeddedPositionSIN( time );
      MatrixRotation waistRchest; waistMchest.extract( waistRchest );
  
      worldRsensor .multiply( waistRchest.transpose(),res );

      sotDEBUGOUT(15);
      return res;
    }



    MatrixHomogeneous& ControllerPD::
    computeWaistWorldPosition( MatrixHomogeneous& res,
			       const int& time )
    {
      sotDEBUGIN(15);
  
      const MatrixHomogeneous & waistMleg = contactEmbeddedPositionSIN( time );
      const MatrixHomogeneous& contactPos = contactWorldPositionSIN( time );
      MatrixHomogeneous legMwaist; waistMleg.inverse( legMwaist );
      MatrixHomogeneous tmpRes;
      if( fromSensor_ )
	{ 
	  const MatrixRotation & Rflex = flexibilitySOUT( time ); // footRleg
	  ml::Vector zero(3); zero.fill(0.);
	  MatrixHomogeneous footMleg; footMleg.buildFrom( Rflex,zero );
				    
	  footMleg.multiply( legMwaist,tmpRes );
	}
      else { tmpRes = legMwaist; }

      contactPos.multiply( tmpRes, res );
      sotDEBUGOUT(15);
      return res;
    }

    ml::Vector& ControllerPD::
    computeWaistWorldPoseRPY( ml::Vector& res,
			      const int& time )
    {
      const MatrixHomogeneous & M = waistWorldPositionSOUT( time );

      MatrixRotation R; M.extract(R);
      VectorRollPitchYaw r; r.fromMatrix(R);
      ml::Vector t(3); M.extract(t);

      res.resize(6);
      for( int i=0;i<3;++i ) { res(i)=t(i); res(i+3)=r(i); }

      return res;
    }

    /* --- PARAMS --------------------------------------------------------------- */
    /* --- PARAMS --------------------------------------------------------------- */
    /* --- PARAMS --------------------------------------------------------------- */
    void ControllerPD::
    commandLine( const std::string& cmdLine,
		 std::istringstream& cmdArgs,
		 std::ostream& os )
    {
      sotDEBUG(25) << "Cmd " << cmdLine <<std::endl;

      if( cmdLine == "help" )
	{
	  Entity::commandLine(cmdLine,cmdArgs,os);
	}
      else if( cmdLine == "fromSensor" )
	{
	  std::string val; cmdArgs>>val;
	  if( ("true"==val)||("false"==val) )
	    {
	      fromSensor_ = ( val=="true" );
	    } else {
	    os << "fromSensor = " << (fromSensor_?"true":"false") << std::endl;
	  }
	}
      else { Entity::commandLine( cmdLine,cmdArgs,os); }
    }

  } // namespace dyninv
} // namespace sot

