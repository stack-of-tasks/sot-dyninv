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

#include <sot-dyninv/dynamic-integrator.h>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot-dyninv/commands-helper.h>

#include <sot-dyninv/mal-to-eigen.h>
#include <soth/Algebra.hpp>

/** This class proposes to integrate the acceleration given in input
 * to produce both velocity and acceleration. Initial conditions have to
 * be provided using the setters of position and velocity. The integration
 * has to be explicitely triggered by calling the command 'inc' (increments).
 */

namespace dynamicgraph
{
  namespace sot
  {
    namespace dyninv
    {

      namespace dg = ::dynamicgraph;
      using namespace dg;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DynamicIntegrator,"DynamicIntegrator");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      DynamicIntegrator::
      DynamicIntegrator( const std::string & name )
	: Entity(name)

	,CONSTRUCT_SIGNAL_IN(acceleration,ml::Vector)
	,CONSTRUCT_SIGNAL_IN(dt,double)

	,CONSTRUCT_SIGNAL_OUT(velocity,ml::Vector,sotNOSIGNAL)
	,CONSTRUCT_SIGNAL_OUT(position,ml::Vector,sotNOSIGNAL)
      {
	Entity::signalRegistration( accelerationSIN << dtSIN
				    << velocitySOUT << positionSOUT );

	addCommand("inc",
		   makeCommandVoid0(*this,&DynamicIntegrator::integrateFromSignals,
				    docCommandVoid0("Integrate acc, update v and p.")));

	addCommand("setPosition",
		   makeDirectSetter(*this,&position,
				    docDirectSetter("position","vector")));
	addCommand("setVelocity",
		   makeDirectSetter(*this,&velocity,
				    docDirectSetter("velocity","vector")));

      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      ml::Vector& DynamicIntegrator::
      velocitySOUT_function( ml::Vector& mlv,int )
      {
	mlv = velocity;
	return mlv;
      }

      ml::Vector& DynamicIntegrator::
      positionSOUT_function( ml::Vector& mlp,int )
      {
	mlp = position;
	return mlp;
      }

      /* --- PROXYS ----------------------------------------------------------- */
      void DynamicIntegrator::
      integrateFromSignals( const int & time )
      {
	const ml::Vector & acc = accelerationSIN(time);
	const double & dt = dtSIN(time);

	integrate( acc,dt, velocity,position );
	velocitySOUT.setReady();
	positionSOUT.setReady();
      }

      void DynamicIntegrator::
      integrateFromSignals( void )
      {
	integrateFromSignals( accelerationSIN.getTime() + 1 );
      }

      void DynamicIntegrator::
      setPosition( const ml::Vector& p )
      {
	position = p;
	positionSOUT.setReady();
      }

      void DynamicIntegrator::
      setVelocity( const ml::Vector& v )
      {
	velocity = v;
	velocitySOUT.setReady();
      }

      void DynamicIntegrator::
      setState( const ml::Vector& p,const ml::Vector& v )
      {
	sotDEBUG(5) << "State: " << p << v << std::endl;
	position = p;
	velocity = v;
	velocitySOUT.setReady();
	positionSOUT.setReady();
      }

      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */

      using namespace Eigen;

      namespace DynamicIntegratorStatic
      {
	template< typename D1 >
	static Matrix3d
	computeRotationMatrixFromEuler(const MatrixBase<D1> & euler)
	{
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<D1>);

	  double Rpsi	= euler[0];
	  double Rtheta = euler[1];
	  double Rphy	= euler[2];

	  double cosPhy = cos(Rphy);
	  double sinPhy = sin(Rphy);

	  double cosTheta = cos(Rtheta);
	  double sinTheta = sin(Rtheta);

	  double cosPsi = cos(Rpsi);
	  double sinPsi = sin(Rpsi);

	  Matrix3d rotation;

	  rotation(0, 0) =  cosPhy * cosTheta;
	  rotation(1, 0) =  sinPhy * cosTheta;
	  rotation(2, 0) = -sinTheta;

	  double   sinTheta__sinPsi = sinTheta * sinPsi;
	  double   sinTheta__cosPsi = sinTheta * cosPsi;

	  rotation(0, 1) = cosPhy * sinTheta__sinPsi - sinPhy * cosPsi;
	  rotation(1, 1) = sinPhy * sinTheta__sinPsi + cosPhy * cosPsi;
	  rotation(2, 1) = cosTheta * sinPsi;

	  rotation(0, 2) = cosPhy * sinTheta__cosPsi + sinPhy * sinPsi;
	  rotation(1, 2) = sinPhy * sinTheta__cosPsi - cosPhy * sinPsi;
	  rotation(2, 2) = cosTheta * cosPsi;

	  return rotation;
	}

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

	template< typename D1 >
	Matrix3d skew( const MatrixBase<D1> & v )
	{
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY( MatrixBase<D1> );
	  Matrix3d mat;
	  mat(0,0) =  0   ;	mat(0,1)= -v[2];	mat(0,2)=  v[1];
	  mat(1,0) =  v[2];	mat(1,1)=  0   ;	mat(1,2)= -v[0];
	  mat(2,0) = -v[1];	mat(2,1)=  v[0];	mat(2,2)=  0   ;
	  return mat;
	}

	/*  Convert data expressed at the origin of the joint - typicaly acc and vel
	 * in Djj - to data expressed at the center of the world - typicaly acc
	 * and vel in Amelif - For the waist dq/ddq(1:6) is equivalent to [ v/a
	 * angular | v/a linear ] expressed at the origin of the joint.
	 */
	template< typename D1,typename D2, typename D3 >
	void
	djj2amelif ( Vector3d & angAmelif, Vector3d & linAmelif,
		     const MatrixBase<D1> & angDjj, const MatrixBase<D2> & linDjj,
		     const MatrixBase<D3> & pos, const Matrix3d & /*orient*/ )
	{
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<D1>);
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<D2>);
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<D3>);
	  assert( angDjj.size() == 3 && linDjj.size() == 3 && pos.size() == 3 );

	  angAmelif = angDjj;
	  linAmelif = linDjj + skew(pos)* angDjj;
	  sotDEBUG(20) << "cross = " << skew(pos)* angDjj << std::endl;
	  sotDEBUG(20) << "cross = " << skew(pos) << std::endl;
	  sotDEBUG(20) << "cross = " <<  angDjj << std::endl;
	}

	/*  Convert data expressed at the center of the world - typicaly acc and vel
	 * in Amelif - to data expressed at the origin of the joint - typicaly acc
	 * and vel in Djj - For the waist dq/ddq(1:6) is equivalent to [ v/a
	 * angular | v/a linear ] expressed at the origin of the joint
	 */
	template< typename D1,typename D2, typename D3 >
	void
	amelif2djj ( MatrixBase<D1> & angDjj, MatrixBase<D2> & linDjj,
		     const Vector3d & angAmelif, const Vector3d & linAmelif,
		     const MatrixBase<D3> & pos, const Matrix3d & /*orient*/ )
	{
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<D1>);
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<D2>);
	  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<D3>);
	  assert( angDjj.size() == 3 && linDjj.size() == 3 && pos.size() == 3 );

	  angDjj = angAmelif;
	  linDjj = linAmelif - skew(pos)* angAmelif;
	}

      }

      /* -------------------------------------------------------------------------- */
      void DynamicIntegrator::
      integrate( const ml::Vector& mlacceleration,
		 const double & dt,
		 ml::Vector & mlvelocity,
		 ml::Vector & mlposition )
      {
	using namespace DynamicIntegratorStatic;
	using soth::MATLAB;
	sotDEBUGIN(15);

	/* --- Convert acceleration, velocity and position to amelif style  ------- */
	EIGEN_CONST_VECTOR_FROM_SIGNAL( acceleration,mlacceleration );
	EIGEN_VECTOR_FROM_SIGNAL( velocity,mlvelocity );
	EIGEN_VECTOR_FROM_SIGNAL( position,mlposition );


	sotDEBUG(1) << "acceleration = " << (MATLAB)acceleration << std::endl;
	sotDEBUG(1) << "velocity = " << (MATLAB)velocity << std::endl;
	sotDEBUG(1) << "position = " << (MATLAB)position << std::endl;

	VectorBlock<SigVectorXd> fftrans = position.head(3);
	VectorBlock<SigVectorXd> ffeuler = position.segment(3,3);
	Matrix3d ffrot = computeRotationMatrixFromEuler(ffeuler);
	sotDEBUG(15) << "Rff_start = " << (MATLAB)ffrot << std::endl;
	sotDEBUG(15) << "tff_start = " << (MATLAB)fftrans << std::endl;

	VectorBlock<SigVectorXd> ffvtrans = velocity.head(3);
	VectorBlock<SigVectorXd> ffvrot = velocity.segment(3,3);
	Vector3d v_lin,v_ang;
	djj2amelif( v_ang,v_lin,ffvrot,ffvtrans,fftrans,ffrot );
	sotDEBUG(15) << "vff_start = " << (MATLAB)v_lin << std::endl;
	sotDEBUG(15) << "wff_start = " << (MATLAB)v_ang << std::endl;

	const VectorBlock<const_SigVectorXd> ffatrans = acceleration.head(3);
	const VectorBlock<const_SigVectorXd> ffarot = acceleration.segment(3,3);
	Vector3d a_lin,a_ang;
	djj2amelif( a_ang,a_lin,ffarot,ffatrans,fftrans,ffrot );
	sotDEBUG(15) << "alff_start = " << (MATLAB)a_lin << std::endl;
	sotDEBUG(15) << "aaff_start = " << (MATLAB)a_ang << std::endl;

	/* --- Integrate velocity ------------------------------------------------- */
	{
	  /* Acceleration, velocity and position of the FF. */
	  Matrix3d finalBodyOrientation;
	  Vector3d finalBodyPosition;

	  double norm_v_ang = v_ang.norm();

	  /* If there's no angular velocity : no rotation. */
	  if (norm_v_ang < 1e-8 )
	    {
	      finalBodyPosition = v_lin*dt + fftrans;
	      finalBodyOrientation = ffrot;
	    }
	  else
	    {
	      const double th  = norm_v_ang * dt;
	      double sth  =  sin(th), cth  =  1.0 - cos(th);
	      Vector3d wn  = v_ang / norm_v_ang;
	      Vector3d vol = v_lin / norm_v_ang;

	      /* drot = wnX * sin(th) + wnX * wnX * (1 - cos (th)). */
	      const Matrix3d w_wedge = skew(wn);

	      Matrix3d drot = w_wedge * cth;
	      drot += Matrix3d::Identity()*sth;
	      drot = w_wedge * drot;

	      //rot = drot + id
	      Matrix3d rot(drot);
	      rot += Matrix3d::Identity();
	      sotDEBUG(1) << "Rv = " << (MATLAB)rot << std::endl;

	      /* Update the body rotation for the body. */
	      finalBodyOrientation = rot * ffrot;

	      /* Update the body position for the body
	       * pos = rot * pos - drot * (wn ^ vol) + th* wn *T(wn) * vol */
	      finalBodyPosition = rot * fftrans;

	      // Calculation of "- drot * crossProduct(wn, vol)"
	      VectorXd tmp1 = (w_wedge*vol);
	      VectorXd tmp2 = w_wedge*tmp1;
	      VectorXd tmp3 = w_wedge*tmp2;
	      tmp2 *= sth;
	      tmp3 *= cth;
	      finalBodyPosition -= tmp2;
	      finalBodyPosition -= tmp3;

	      // Calculation of "th * wn * T(wn) * vol"
	      double w0v0 = wn[0u] * vol[0u];
	      double w1v1 = wn[1u] * vol[1u];
	      double w2v2 = wn[2u] * vol[2u];
	      w0v0 += w1v1;
	      w0v0 += w2v2;
	      w0v0 *= th;

	      // pos += wn * wovo
	      finalBodyPosition[0u] += wn[0u] * w0v0;
	      finalBodyPosition[1u] += wn[1u] * w0v0;
	      finalBodyPosition[2u] += wn[2u] * w0v0;
	    }

	  sotDEBUG(1) << "Rff_end = " << (MATLAB)finalBodyOrientation << std::endl;

	  /* --- Convert position --------------------------------------------------- */
	  Vector3d ffeuleur_fin = computeEulerFromRotationMatrix( finalBodyOrientation );

	  position.head(3) = finalBodyPosition;
	  position.segment(3,3) = ffeuleur_fin;
	  position.tail( position.size()-6 ) += velocity.tail( position.size()-6 ) * dt;
	}

	/* --- Integrate acceleration --------------------------------------------- */

	v_lin += a_lin*dt;
	v_ang += a_ang*dt;

	Vector3d vdjj_ang,vdjj_lin;
	amelif2djj( vdjj_ang,vdjj_lin,v_ang,v_lin,fftrans,ffrot);
	velocity.head(3) = vdjj_lin; velocity.segment(3,3) = vdjj_ang;
	//amelif2djj( ffvrot,ffvtrans,v_ang,v_lin,fftrans,ffrot);
	velocity.tail( velocity.size()-6 ) += acceleration.tail( acceleration.size()-6 )*dt;

	sotDEBUG(15) << "vff_end = " << (MATLAB)v_lin << std::endl;
	sotDEBUG(15) << "wff_end = " << (MATLAB)v_ang << std::endl;


	sotDEBUG(1) << "velocity = " << (MATLAB)velocity << std::endl;
	sotDEBUG(1) << "position = " << (MATLAB)position << std::endl;
	sotDEBUGOUT(15);
      }

      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */

      void DynamicIntegrator::
      display( std::ostream& os ) const
      {
	os << "DynamicIntegrator "<<getName() << ". " << std::endl;
      }

      void DynamicIntegrator::
      commandLine( const std::string& cmdLine,
		   std::istringstream& cmdArgs,
		   std::ostream& os )
      {
	if( cmdLine == "help" )
	  {
	    os << "DynamicIntegrator:" << std::endl
	       << " - inc [dt]" << std::endl;
	  }
	else if( cmdLine == "inc" )
	  {
	    if( cmdArgs >> std::ws, cmdArgs.good() )
	      {
		double dt; cmdArgs >> dt; dtSIN = dt;
	      }
	    integrateFromSignals();
	  }
	else
	  {
	    Entity::commandLine( cmdLine,cmdArgs,os );
	  }
      }

    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

