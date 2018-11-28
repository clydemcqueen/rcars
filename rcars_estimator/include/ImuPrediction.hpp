/*
* Copyright (c) 2014, Michael Neunert & Michael Blösch
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef RCARS_IMUPREDICTION_HPP_
#define RCARS_IMUPREDICTION_HPP_

#include "FilterStates.hpp"
#include "lightweight_filtering/Prediction.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rcars {

template<typename FILTERSTATE>
class ImuPrediction: public LWF::Prediction<FILTERSTATE>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::Prediction<FILTERSTATE> Base;
  using Base::eval;
  using Base::prenoiP_;
  using Base::doubleRegister_;
  using Base::boolRegister_;
  using Base::disablePreAndPostProcessingWarning_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  /*!
   * Gravity vector expressed in the inertial frame
   */
  const V3D g_;
  /*!
   * Verbose flag
   */
  bool verbose_;
  ImuPrediction():g_(0,0,-9.81){
    int ind;
    for(int i=0;i<mtState::nDynamicTags_;i++){
      ind = mtState::template getId<mtState::_dyp>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind+0,ind+0));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+1,ind+1));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+2,ind+2));
      doubleRegister_.registerScalar("PredictionNoise.dyp_0",prenoiP_(ind+0,ind+0));
      doubleRegister_.registerScalar("PredictionNoise.dyp_1",prenoiP_(ind+1,ind+1));
      doubleRegister_.registerScalar("PredictionNoise.dyp_2",prenoiP_(ind+2,ind+2));
      ind = mtState::template getId<mtState::_dya>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind+0,ind+0));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+1,ind+1));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+2,ind+2));
      doubleRegister_.registerScalar("PredictionNoise.dya_0",prenoiP_(ind+0,ind+0));
      doubleRegister_.registerScalar("PredictionNoise.dya_1",prenoiP_(ind+1,ind+1));
      doubleRegister_.registerScalar("PredictionNoise.dya_2",prenoiP_(ind+2,ind+2));

    }
    for(int i=0;i<mtState::nHybridTags_;i++){
      ind = mtState::template getId<mtState::_hya>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind+0,ind+0));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+1,ind+1));
      doubleRegister_.registerScalar("PredictionNoise.hya_0",prenoiP_(ind+0,ind+0));
      doubleRegister_.registerScalar("PredictionNoise.hya_1",prenoiP_(ind+1,ind+1));
    }
    disablePreAndPostProcessingWarning_ = true;
    verbose_ = false;
  };
  ~ImuPrediction(){};
  /*!
   * Prediction model of filter. Based on IMU measurements.
   */
  void eval(mtState& output, const mtState& state, const mtMeas& meas, const mtNoise noise, double dt) const{
	double sqrt_dt = sqrt(dt);
    output.template get<mtState::_aux>().MwIMmeas_ = meas.template get<mtMeas::_gyr>();
    output.template get<mtState::_aux>().MwIMest_ = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    const V3D imuRor = output.template get<mtState::_aux>().MwIMest_+noise.template get<mtNoise::_att>()/sqrt_dt;
    const V3D dImuOmega = dt*imuRor;
    QPD dQ = dQ.exponentialMap(-dImuOmega);
    output.template get<mtState::_pos>() = state.template get<mtState::_pos>()-dt*(state.template get<mtState::_att>().rotate(state.template get<mtState::_vel>())
        -noise.template get<mtNoise::_pos>()/sqrt_dt);
    output.template get<mtState::_vel>() = (M3D::Identity()-gSM(dImuOmega))*state.template get<mtState::_vel>()
        -dt*(meas.template get<mtMeas::_acc>()-state.template get<mtState::_acb>()+state.template get<mtState::_att>().inverseRotate(g_)-noise.template get<mtNoise::_vel>()/sqrt_dt);
    output.template get<mtState::_acb>() = state.template get<mtState::_acb>()+noise.template get<mtNoise::_acb>()*sqrt_dt;
    output.template get<mtState::_gyb>() = state.template get<mtState::_gyb>()+noise.template get<mtNoise::_gyb>()*sqrt_dt;
    output.template get<mtState::_att>() = state.template get<mtState::_att>()*dQ;
    output.template get<mtState::_vep>() = state.template get<mtState::_vep>()+noise.template get<mtNoise::_vep>()*sqrt_dt;
    dQ = dQ.exponentialMap(noise.template get<mtNoise::_vea>()*sqrt_dt);
    output.template get<mtState::_vea>() = dQ*state.template get<mtState::_vea>();

    // Predict new robocentric tag position
    const V3D camRor(state.get_qVM().rotate(imuRor));
    const V3D camVel(state.get_qVM().rotate(V3D(imuRor.cross(state.get_MrMV())-state.template get<mtState::_vel>())));
    const V3D dtCamRor(dt*camRor);
    const V3D dtCamVel(dt*camVel);

    for(unsigned int i=0;i<mtState::nDynamicTags_;i++){
      output.template get<mtState::_dyp>(i) = (M3D::Identity()-gSM(dtCamRor))*state.template get<mtState::_dyp>(i)-dtCamVel+noise.template get<mtNoise::_dyp>(i)*sqrt_dt;
      dQ = dQ.exponentialMap(-dtCamRor + noise.template get<mtNoise::_dya>(i)*sqrt_dt);
      output.template get<mtState::_dya>(i) = state.template get<mtState::_dya>(i)*dQ;
    }
    for(unsigned int i=0;i<mtState::nHybridTags_;i++){
      state.template get<mtState::_hya>(i).boxPlus((noise.template get<mtNoise::_hya>(i)*sqrt_dt).eval(),output.template get<mtState::_hya>(i));
    }

    output.template get<mtState::_aux>().wMeasCov_ = prenoiP_.template block<3,3>(mtNoise::template getId<mtNoise::_att>(),mtNoise::template getId<mtNoise::_att>())/dt;
    output.template get<mtState::_aux>().timeSinceLastValidUpdate_ = state.template get<mtState::_aux>().timeSinceLastValidUpdate_+dt;
    output.fix();
  }
  void noMeasCase(mtFilterState& filterState, mtMeas& meas, double dt){
    meas.template get<mtMeas::_gyr>() = filterState.state_.template get<mtState::_gyb>();
    meas.template get<mtMeas::_acc>() = filterState.state_.template get<mtState::_acb>()-filterState.state_.template get<mtState::_att>().inverseRotate(g_);
  }
  /*!
   * Jacobian of prediction model with respect to the filter state
   */
  void jacInput(mtJacInput& F, const mtState& state, const mtMeas& meas, double dt) const{
    const V3D imuRor(meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>());
    const V3D dImuOmega(dt*imuRor);
    const M3D attMatrix(MPD(state.template get<mtState::_att>()).matrix());

    F.setZero();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_pos>()).setIdentity();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_vel>()) = -dt*attMatrix;
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_att>()) = -dt*gSM(state.template get<mtState::_att>().rotate(state.template get<mtState::_vel>()));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_vel>()) = (M3D::Identity()-gSM(dImuOmega));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_acb>()) = dt*M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_gyb>()) = -dt*gSM(state.template get<mtState::_vel>());
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_att>()) = dt*attMatrix.transpose()*gSM(g_);
    F.template block<3,3>(mtState::template getId<mtState::_acb>(),mtState::template getId<mtState::_acb>()).setIdentity();
    F.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtState::template getId<mtState::_gyb>()).setIdentity();
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_gyb>()) = dt*attMatrix*Lmat(-dImuOmega);
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_att>()).setIdentity();
    F.template block<3,3>(mtState::template getId<mtState::_vep>(),mtState::template getId<mtState::_vep>()).setIdentity();
    F.template block<3,3>(mtState::template getId<mtState::_vea>(),mtState::template getId<mtState::_vea>()).setIdentity();

    const V3D camRor(state.get_qVM().rotate(imuRor));
    const V3D camVel(state.get_qVM().rotate(V3D(imuRor.cross(state.get_MrMV())-state.template get<mtState::_vel>())));

    const M3D veaMatrix(MPD(state.get_qVM()).matrix());
    const M3D veaMatrixDt(veaMatrix*dt);
    const M3D dtveaMatrix(dt*veaMatrix);
    const M3D lmatCamRor(Lmat(-dt*camRor));
    const M3D gSMvep(gSM(state.get_MrMV()));
    const M3D gSMveaImuRorVepVelDt(gSM(state.get_qVM().rotate(V3D(imuRor.cross(state.get_MrMV())-state.template get<mtState::_vel>())))*dt);
    const M3D gSMcamRor(gSM(camRor));
    const M3D gSMimuRor(gSM(imuRor));

    for(unsigned int i=0;i<mtState::nDynamicTags_;i++){
      const M3D dyaMatrixDt(MPD(state.template get<mtState::_dya>(i)).matrix()*dt);
      const M3D dyaMatrixDtLmatCamRor(dyaMatrixDt*lmatCamRor);
      F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_dyp>(i)) = (M3D::Identity()-gSM(V3D(dt*camRor)));
      F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_vel>()) = dtveaMatrix;
      F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_gyb>()) =
          -veaMatrixDt*gSMvep
          -dt*gSM(state.template get<mtState::_dyp>(i))*veaMatrix;
      if(state.template get<mtState::_aux>().enableExtrinsicCalibration_){
        F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_vep>()) =
            -veaMatrixDt*gSM(imuRor);
        F.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtState::template getId<mtState::_vea>()) =
            -gSMveaImuRorVepVelDt
            +dt*gSM(state.template get<mtState::_dyp>(i))*gSMcamRor;
        F.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtState::template getId<mtState::_vea>()) =
            -dyaMatrixDtLmatCamRor*gSMcamRor;
      }
      F.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtState::template getId<mtState::_dya>(i)).setIdentity();
      F.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtState::template getId<mtState::_gyb>()) =
    	   dyaMatrixDtLmatCamRor*veaMatrix;
    }
    for(unsigned int i=0;i<mtState::nHybridTags_;i++){
      F.template block<2,2>(mtState::template getId<mtState::_hya>(i),mtState::template getId<mtState::_hya>(i)).setIdentity();
    }
  }
  /*!
   * Jacobian of prediction model with respect to the process noise
   */
  void jacNoise(mtJacNoise& G, const mtState& state, const mtMeas& meas, double dt) const{
	const double sqrt_dt = sqrt(dt);
    const V3D imuRor(meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>());
    const V3D dImuOmega(dt*imuRor);
    const M3D sqrtDtIdentity(M3D::Identity()*sqrt_dt);
    G.setZero();
    G.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = sqrtDtIdentity;
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_vel>()) = sqrtDtIdentity;
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_att>()) =
        gSM(state.template get<mtState::_vel>())*sqrt_dt;
    G.template block<3,3>(mtState::template getId<mtState::_acb>(),mtNoise::template getId<mtNoise::_acb>()) = sqrtDtIdentity;
    G.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtNoise::template getId<mtNoise::_gyb>()) = sqrtDtIdentity;
    G.template block<3,3>(mtState::template getId<mtState::_att>(),mtNoise::template getId<mtNoise::_att>()) =
        -MPD(state.template get<mtState::_att>()).matrix()*Lmat(-dImuOmega)*sqrt_dt;
    G.template block<3,3>(mtState::template getId<mtState::_vep>(),mtNoise::template getId<mtNoise::_vep>()) = sqrtDtIdentity;
    G.template block<3,3>(mtState::template getId<mtState::_vea>(),mtNoise::template getId<mtNoise::_vea>()) = sqrtDtIdentity;

    const V3D camRor(state.get_qVM().rotate(imuRor));
    const M3D veaMatrix(MPD(state.get_qVM()).matrix());
    const M3D veaMatrixSqrtDt(MPD(state.get_qVM()).matrix()*sqrt_dt);
    const M3D lmatCamRor(Lmat(-dt*camRor));
    const M3D gSMvep(gSM(state.get_MrMV()));
    const M3D veaMatrixgSMvepSqrtDt(veaMatrixSqrtDt*gSMvep);

    for(unsigned int i=0;i<mtState::nDynamicTags_;i++){
      const M3D dyaMatrix(MPD(state.template get<mtState::_dya>(i)).matrix());
      const M3D dyaMatrixLmatCamRorSqrtDt(dyaMatrix*lmatCamRor*sqrt_dt);
      G.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtNoise::template getId<mtNoise::_dyp>(i)) = sqrtDtIdentity;
      G.template block<3,3>(mtState::template getId<mtState::_dyp>(i),mtNoise::template getId<mtNoise::_att>()) =
    	   veaMatrixgSMvepSqrtDt
          +gSM(state.template get<mtState::_dyp>(i))*veaMatrixSqrtDt;
      G.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtNoise::template getId<mtNoise::_att>()) =
          -dyaMatrixLmatCamRorSqrtDt*veaMatrix;
      G.template block<3,3>(mtState::template getId<mtState::_dya>(i),mtNoise::template getId<mtNoise::_dya>(i)) =
    		  dyaMatrixLmatCamRorSqrtDt;
    }

    for(unsigned int i=0;i<mtState::nHybridTags_;i++){
      G.template block<2,2>(mtState::template getId<mtState::_hya>(i),mtNoise::template getId<mtNoise::_hya>(i)) = Eigen::Matrix2d::Identity()*sqrt_dt;
    }
  }
};

}


#endif /* RCARS_IMUPREDICTION_HPP_ */
