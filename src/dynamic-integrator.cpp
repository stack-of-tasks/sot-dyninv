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

	,CONSTRUCT_SIGNAL_IN(acceleration,dg::Vector)
	,CONSTRUCT_SIGNAL_IN(dt,double)

	,CONSTRUCT_SIGNAL_OUT(velocity,dg::Vector,sotNOSIGNAL)
	,CONSTRUCT_SIGNAL_OUT(position,dg::Vector,sotNOSIGNAL)
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

      dg::Vector& DynamicIntegrator::
      velocitySOUT_function( dg::Vector& mlv,int )
      {
	mlv = velocity;
	return mlv;
      }

      dg::Vector& DynamicIntegrator::
      positionSOUT_function( dg::Vector& mlp,int )
      {
	mlp = position;
	return mlp;
      }

      /* --- PROXYS ----------------------------------------------------------- */
      void DynamicIntegrator::
      integrateFromSignals( const int & time )
      {
	const dg::Vector & acc = accelerationSIN(time);
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
      setPosition( const dg::Vector& p )
      {
	position = p;
	positionSOUT.setReady();
      }

      void DynamicIntegrator::
      setVelocity( const dg::Vector& v )
      {
	velocity = v;
	velocitySOUT.setReady();
      }

      void DynamicIntegrator::
      setState( const dg::Vector& p,const dg::Vector& v )
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
	void
	computeRotationMatrixFromEuler(const Eigen::Vector3d& euler, Eigen::Matrix3d& res) {
	  res = (Eigen::AngleAxisd(euler(2),Eigen::Vector3d::UnitZ())*
		 Eigen::AngleAxisd(euler(1),Eigen::Vector3d::UnitY())*
		 Eigen::AngleAxisd(euler(0),Eigen::Vector3d::UnitX())).toRotationMatrix();
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
      integrate( const dg::Vector& mlacceleration,
		 const double & dt,
		 dg::Vector & mlvelocity,
		 dg::Vector & mlposition )
      {
	using namespace DynamicIntegratorStatic;
	using soth::MATLAB;
	sotDEBUGIN(15);

	/* --- Convert acceleration, velocity and position to amelif style  ------- */
	const Eigen::VectorXd acceleration(mlacceleration);
	Eigen::VectorXd velocity(mlvelocity);
	Eigen::VectorXd position(mlposition);

	sotDEBUG(1) << "acceleration = " << (MATLAB)acceleration << std::endl;
	sotDEBUG(1) << "velocity = " << (MATLAB)velocity << std::endl;
	sotDEBUG(1) << "position = " << (MATLAB)position << std::endl;

	const Eigen::Vector3d fftrans = position.head<3>();
	const Eigen::Vector3d ffeuler = position.segment<3>(3);
	Eigen::Matrix3d ffrot; computeRotationMatrixFromEuler(ffeuler, ffrot);
	sotDEBUG(15) << "Rff_start = " << (MATLAB)ffrot << std::endl;
	sotDEBUG(15) << "tff_start = " << (MATLAB)fftrans << std::endl;

	Eigen::Vector3d ffvtrans = velocity.head<3>();
	Eigen::Vector3d ffvrot = velocity.segment<3>(3);
	Eigen::Vector3d v_lin,v_ang;
	djj2amelif( v_ang,v_lin,ffvrot,ffvtrans,fftrans,ffrot );
	sotDEBUG(15) << "vff_start = " << (MATLAB)v_lin << std::endl;
	sotDEBUG(15) << "wff_start = " << (MATLAB)v_ang << std::endl;

	const Eigen::Vector3d ffatrans = acceleration.head<3>();
	const Eigen::Vector3d ffarot = acceleration.segment<3>(3);
	Eigen::Vector3d a_lin,a_ang;
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
	      Eigen::Vector3d wn  = v_ang / norm_v_ang;
	      Eigen::Vector3d vol = v_lin / norm_v_ang;

	      /* drot = wnX * sin(th) + wnX * wnX * (1 - cos (th)). */
	      const Eigen::Matrix3d w_wedge = skew(wn);

	      Eigen::Matrix3d drot = w_wedge * cth;
	      drot += Eigen::Matrix3d::Identity()*sth;
	      drot = w_wedge * drot;

	      //rot = drot + id
	      Eigen::Matrix3d rot(drot);
	      rot += Eigen::Matrix3d::Identity();
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
	  position.head<3>() = finalBodyPosition;
	  position.segment<3>(3) = (finalBodyOrientation.eulerAngles(2,1,0)).reverse();
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
    } // namespace dyninv
  } // namespace sot
} // namespace dynamicgraph

