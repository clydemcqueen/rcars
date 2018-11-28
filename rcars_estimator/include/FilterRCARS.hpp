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

/*!
 * Coordinate frame overview:
 * - I: Inertial frame
 * - M: Imu-fixed coordinate frame
 * - V: Camera-fixed coordinate frame
 * - T: Tag-fixed coordinate frame
 */



#ifndef FILTERRCARS_HPP_
#define FILTERRCARS_HPP_

#include "FilterStates.hpp"
#include "ImuPrediction.hpp"
#include "TagUpdate.hpp"
#include "lightweight_filtering/FilterBase.hpp"

/*!
 * Filter Namespace
 */
namespace rcars {

/*!
 * Filter class.
 */
template<int nDynamicTags, int nHybridTags>
class FilterRCARS:public LWF::FilterBase<ImuPrediction<FilterState<nDynamicTags,nHybridTags>>,TagUpdate<FilterState<nDynamicTags,nHybridTags>>>{
 public:
  /*!
   * Typedefs and using-declarations
   */
  typedef LWF::FilterBase<ImuPrediction<FilterState<nDynamicTags,nHybridTags>>,TagUpdate<FilterState<nDynamicTags,nHybridTags>>> Base;
  using Base::init_;
  using Base::updateTimelineTuple_;
  using Base::reset;
  using Base::safe_;
  using Base::front_;
  using Base::doubleRegister_;
  using Base::boolRegister_;
  using Base::mUpdates_;
  using Base::mPrediction_;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtPrediction mtPrediction;
  typedef typename Base::mtState mtState;
  /*!
   * Constructor
   */
  FilterRCARS(){
    std::get<0>(mUpdates_).outlierDetection_.setEnabledAll(true);
    reset(0.0);
    int ind;
    for(int i=0;i<mtState::nDynamicTags_;i++){
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_dyp>(i)(0));
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_dyp>(i)(1));
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_dyp>(i)(2));
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_dya>(i).toImplementation().w());
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_dya>(i).toImplementation().x());
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_dya>(i).toImplementation().y());
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_dya>(i).toImplementation().z());
      ind = mtState::template getId<mtState::_dyp>(i);
      doubleRegister_.removeScalarByVar(init_.cov_(ind+0,ind+0));
      doubleRegister_.removeScalarByVar(init_.cov_(ind+1,ind+1));
      doubleRegister_.removeScalarByVar(init_.cov_(ind+2,ind+2));
      ind = mtState::template getId<mtState::_dya>(i);
      doubleRegister_.removeScalarByVar(init_.cov_(ind+0,ind+0));
      doubleRegister_.removeScalarByVar(init_.cov_(ind+1,ind+1));
      doubleRegister_.removeScalarByVar(init_.cov_(ind+2,ind+2));
    }
    for(int i=0;i<mtState::nHybridTags_;i++){
      ind = mtState::template getId<mtState::_hya>(i);
      doubleRegister_.removeScalarByVar(init_.cov_(ind+0,ind+0));
      doubleRegister_.removeScalarByVar(init_.cov_(ind+1,ind+1));
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_hya>(i).q_.toImplementation().w());
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_hya>(i).q_.toImplementation().x());
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_hya>(i).q_.toImplementation().y());
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_hya>(i).q_.toImplementation().z());
    }
    mPrediction_.doubleRegister_.removeScalarByStr("alpha");
    mPrediction_.doubleRegister_.removeScalarByStr("beta");
    mPrediction_.doubleRegister_.removeScalarByStr("kappa");
    for(int i=0;i<3;i++){
      std::get<0>(mUpdates_).doubleRegister_.registerScalar("initTagPosCov",init_.dynamicTagInitCov_(i,i));
      std::get<0>(mUpdates_).doubleRegister_.registerScalar("initTagAttCov",init_.dynamicTagInitCov_(i+3,i+3));
    }
    boolRegister_.registerScalar("Common.verbose",verbose_);
    boolRegister_.registerScalar("Common.verbose",mPrediction_.verbose_);
    boolRegister_.registerScalar("Common.verbose",std::get<0>(mUpdates_).verbose_);
    std::get<0>(mUpdates_).doubleRegister_.registerScalar("maxWaitTime",std::get<0>(updateTimelineTuple_).maxWaitTime_);
    boolRegister_.registerScalar("Common.enableExtrinsicCalibration",init_.state_.template get<mtState::_aux>().enableExtrinsicCalibration_);
    verbose_ = false;
  }
  /*!
   * Verbose flag
   */
  bool verbose_;
  /*!
   * Gets called after the info file is read
   */
  void refreshProperties(){
    init_.state_.template get<mtState::_aux>().MrMV_ = init_.state_.template get<mtState::_vep>();
    init_.state_.template get<mtState::_aux>().qVM_ = init_.state_.template get<mtState::_vea>();
  };
  /*!
   * Destructor
   */
  ~FilterRCARS(){};

  /*!
   * Returns the position of the IMU in the inertial frame
   * IrIM
   */
  V3D get_IrIM(const mtFilterState& filterState){
    return filterState.state_.template get<mtState::_pos>();
  }
  V3D get_IrIM_front(){
    return get_IrIM(front_);
  }
  V3D get_IrIM_safe(){
    return get_IrIM(safe_);
  }

  /*!
   * Returns the robocentric rotational rate w.r.t. the IMU frame
   * MwIM
   */
  V3D get_MwIM(const mtFilterState& filterState){
    return filterState.state_.template get<mtState::_aux>().MwIMest_;
  }

  /*!
   * Returns the velocity w.r.t. the inertial frame
   * IvM = qIM*(MvM)
   */
  V3D get_IvM(const mtFilterState& filterState){
    return filterState.state_.template get<mtState::_att>().rotate(V3D(-filterState.state_.template get<mtState::_vel>()));
  }

  /*!
   * Returns the attitude of the IMU w.r.t. the inertial frame
   * qMI = (qIM)^T
   */
  QPD get_qMI(const mtFilterState& filterState){
    return filterState.state_.template get<mtState::_att>().inverted();
  }
  QPD get_qMI_front(){
    return get_qMI(front_);
  }
  QPD get_qMI_safe(){
    return get_qMI(safe_);
  }

  /*!
   * Returns the position of the dynamic tag i w.r.t. the camera frame
   */
  V3D get_VrVT_dyn(const mtFilterState& filterState, int i){
    return filterState.state_.template get<mtState::_dyp>(i);
  }
  V3D get_VrVT_dyn_front(int i){
    return get_VrVT_dyn(front_,i);
  }
  V3D get_VrVT_dyn_safe(int i){
    return get_VrVT_dyn(safe_,i);
  }

  /*!
   * Returns the attitude of the dynamic tag i w.r.t. the camera frame
   */
  QPD get_qTV_dyn(const mtFilterState& filterState, int i){
    return filterState.state_.template get<mtState::_dya>(i);
  }
  QPD get_qTV_dyn_front(int i){
    return get_qTV_dyn(front_,i);
  }
  QPD get_qTV_dyn_safe(int i){
    return get_qTV_dyn(safe_,i);
  }
  /*!
   * Returns the position of the dynamic tag i w.r.t. the inertial frame
   * IrIT = IrIM + qIM*(MrMV + qVM^T*(VrVT))
   */
  V3D get_IrIT_dyn(const mtFilterState& filterState, int i){
    return filterState.state_.template get<mtState::_pos>() + filterState.state_.template get<mtState::_att>().rotate(V3D(filterState.state_.get_MrMV()
        + filterState.state_.get_qVM().inverseRotate(filterState.state_.template get<mtState::_dyp>(i))));
  }
  V3D get_IrIT_dyn_front(int i){
    return get_IrIT_dyn(front_,i);
  }
  V3D get_IrIT_dyn_safe(int i){
    return get_IrIT_dyn(safe_,i);
  }

  /*!
   * Returns the attitude of the dynamic tag i w.r.t. the inertial frame
   * qTI = qTV*qVM*qIM^T
   */
  QPD get_qTI_dyn(const mtFilterState& filterState, int i){
    return filterState.state_.template get<mtState::_dya>(i)*filterState.state_.get_qVM()*filterState.state_.template get<mtState::_att>().inverted();
  }
  QPD get_qTI_dyn_front(int i){
    return get_qTI_dyn(front_,i);
  }
  QPD get_qTI_dyn_safe(int i){
    return get_qTI_dyn(safe_,i);
  }

  /*!
   * Resets the filter filterState.state_ with the help of an accelerometer measurement assuming no velocity
   */
  void resetWithAccelerometer(const V3D& fMeasInit, double t = 0.0){
    init_.initWithAccelerometer(fMeasInit);
    reset(t);
  }

  /*!
   * Compute commonly used output filterState.state_
   */
  void getOutput(const mtFilterState& filterState,V3D& IrIM,QPD& qIM,V3D& MvM,V3D& MwM){
    /*!
     * Position of body expressed in inertial frame
     */
    IrIM = get_IrIM(filterState); // Position of body expressed in inertial frame
    /*!
     * Compute attitude between body and inertial frame
     */
    qIM = filterState.state_.template get<mtState::_att>();
    /*!
     * Combute robocentric rotational rate
     */
    MwM = get_MwIM(filterState);
    /*!
     * Compute robocentric velocity of body frame
     */
    MvM = -filterState.state_.template get<mtState::_vel>();
  }

  /*!
   * Compute covariance of output computed in getOutput(...)
   */
  Eigen::Matrix<double,12,12> getOutputCovariance(const mtFilterState& filterState){
    /*!
     * Reduced covariance matrix.
     * Order of filterState.state_: MrIM, qIM, MvM, gyb
     */
    Eigen::Matrix<double,12,12> C;

    // Extract the reduced covariance matrix based on the following list of indices
    unsigned int index[4] = {mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_gyb>()};
    for(unsigned int i=0;i<4;i++){
      for(unsigned int j=0;j<4;j++){
        C.block<3,3>(i*3,j*3) = filterState.cov_.template block<3,3>(index[i],index[j]);
      }
    }

    // Add covariance of gyroscope noise
    C.block<3,3>(9,9) = C.block<3,3>(9,9) + filterState.state_.template get<mtState::_aux>().wMeasCov_;

    return C;
  }
};

}


#endif /* FILTERRCARS_HPP_ */
