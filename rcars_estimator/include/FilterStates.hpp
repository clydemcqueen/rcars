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

#ifndef RCARS_FILTERSTATES_HPP_
#define RCARS_FILTERSTATES_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rcars {

enum TagType{
  DYNAMIC_TAG = 0,
  HYBRID_TAG = 1,
  STATIC_TAG = 2,
  TAG_UNSPECIFIED = -1
};

template<unsigned int nDynamicTags, unsigned int nHybridTags>
class StateAuxiliary: public LWF::AuxiliaryBase<StateAuxiliary<nDynamicTags,nHybridTags>>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*!
   * Constructor
   */
  StateAuxiliary(){
    MwIMest_.setZero();
    MwIMmeas_.setZero();
    wMeasCov_.setIdentity();
    for(unsigned int i=0;i<nDynamicTags;i++){
      dynamicIds_[i] = -1;
      corners_[i].setZero();
      nOutliers_[i] = 0;
    }
    for(unsigned int i=0;i<nHybridTags;i++){
      hybridIds_[i] = -1;
    }
    measIndIterator_ = 0;
    timeSinceLastValidUpdate_ = 0;
    enableExtrinsicCalibration_ = true;
  };
  ~StateAuxiliary(){};
  /*!
   * Estimated rotational rate (bias corrected)
   */
  V3D MwIMest_;
  /*!
   * Measured rotational rate
   */
  V3D MwIMmeas_;
  /*!
   * Actual gyroscope covariance matrix (discretized form)
   */
  M3D wMeasCov_;
  /*!
   * Array containing ID of dynamic tags
   * Contains -1 if not yet assigned
   */
  int dynamicIds_[nDynamicTags];
  /*!
   * Array the estimated corner locations in the image
   */
  mutable Eigen::Matrix<double,8,1> corners_[nDynamicTags];
  /*!
   * Array containing ID of hybrid tags
   * Contains -1 if not yet assigned
   */
  int hybridIds_[nHybridTags];
  /*!
   * Internal iterator for measurement indices
   */
  int measIndIterator_;
  /*!
   * Time since last valid update
   */
  double timeSinceLastValidUpdate_;
  /*!
   * Number of subsequent outliers
   */
  int nOutliers_[nDynamicTags];
  /*!
   * Whether the extrinsics calibration is enabled
   */
  bool enableExtrinsicCalibration_;
  /*!
   * Position between IMU and Camera frame expressed in the camera frame. Used when extrinsic calibration disabled.
   */
  V3D MrMV_;
  /*!
   * Orientation between IMU and Camera frame. Used when extrinsic calibration disabled.
   */
  QPD qVM_;
  /*!
   * Searches the dynamic tag ID vector for a specific tag ID and returns the vector index
   * Returns -1 if not found.
   */
  int getDynamicIndFromTagId(int tagId) const{
    for(unsigned int i=0;i<nDynamicTags;i++){
      if(dynamicIds_[i]==tagId){
        return i;
      }
    }
    return -1;
  }
  /*!
   * Attempts to find an empty dynamic tag state.
   * Returns -1 if all tag states are occupied.
   */
  int getFreeDynamicInd() const{
    for(unsigned int i=0;i<nDynamicTags;i++){
      if(dynamicIds_[i]==-1){
        return i;
      }
    }
    return -1;
  }
  /*!
   * Searches the hybrid tag ID vector for a specific tag ID and returns the vector index
   * Returns -1 if not found.
   */
  int getHybridIndFromTagId(int tagId) const{
    for(unsigned int i=0;i<nHybridTags;i++){
      if(hybridIds_[i]==tagId){
        return i;
      }
    }
    return -1;
  }
  /*!
   * Attempts to find an empty hybrid tag state.
   * Returns -1 if all tag states are occupied.
   */
  int getFreeHybridInd() const{
    for(unsigned int i=0;i<nHybridTags;i++){
      if(hybridIds_[i]==-1){
        return i;
      }
    }
    return -1;
  }
};

/*!
 * State class, contains references to substates:
 * - pos: robocentric position
 * - vel: robocentric velocity
 * - att: robocentric attitude
 * - acb: accelerometer bias
 * - gyb: gyroscope bias
 * - vep: Vison extrinsics position
 * - vea: Vison extrinsics attitude
 * - dyp: dynamic tag positions
 * - dya: dynamic tag attitudes
 * - ĥya: hybrid tag normal
 * Also includes tracking of further quantities.
 * nDynamicTags is the maximum number of dynamic tags that can be kept in the filter state.
 * nHybridTags is the maximum number of hybrid tags that can be kept in the filter state.
 */
template<int nDynamicTags, int nHybridTags>
class State: public LWF::State<
    LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
    LWF::TH_multiple_elements<LWF::QuaternionElement,2>,
    LWF::ArrayElement<LWF::VectorElement<3>,nDynamicTags>,
    LWF::ArrayElement<LWF::QuaternionElement,nDynamicTags>,
    LWF::ArrayElement<LWF::NormalVectorElement,nHybridTags>,
    StateAuxiliary<nDynamicTags,nHybridTags>>{
 public:
  typedef LWF::State<
      LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
      LWF::TH_multiple_elements<LWF::QuaternionElement,2>,
      LWF::ArrayElement<LWF::VectorElement<3>,nDynamicTags>,
      LWF::ArrayElement<LWF::QuaternionElement,nDynamicTags>,
      LWF::ArrayElement<LWF::NormalVectorElement,nHybridTags>,
      StateAuxiliary<nDynamicTags,nHybridTags>> Base;
  using Base::D_;
  using Base::E_;
  static constexpr int nDynamicTags_ = nDynamicTags;
  static constexpr int nHybridTags_ = nHybridTags;
  static constexpr int nTags_ = nDynamicTags + nHybridTags;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _acb = _vel+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _vep = _gyb+1;
  static constexpr unsigned int _att = _vep+1;
  static constexpr unsigned int _vea = _att+1;
  static constexpr unsigned int _dyp = _vea+(int)(nDynamicTags>0);
  static constexpr unsigned int _dya = _dyp+(int)(nDynamicTags>0);
  static constexpr unsigned int _hya = _dya+(int)(nHybridTags>0);
  static constexpr unsigned int _aux = _hya+1;
  /*!
   * Constructor
   */
  State(){
    static_assert(_aux+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_vep>() = "vep";
    this->template getName<_att>() = "att";
    this->template getName<_vea>() = "vea";
    if((int)(nDynamicTags>0)) this->template getName<_dyp>() = "dyp";
    if((int)(nDynamicTags>0)) this->template getName<_dya>() = "dya";
    if((int)(nHybridTags>0)) this->template getName<_hya>() = "hya";
    this->template getName<_aux>() = "aux";
  }
  ~State(){};
  /*!
   * Sets the corner data (mutable)
   */
  void setCorners(int i, int j, double& x, double& y) const{
    this->template get<_aux>().corners_[i](j*2+0) = x;
    this->template get<_aux>().corners_[i](j*2+1) = y;
  }
  V3D& get_MrMV(){
    if(this->template get <_aux>().enableExtrinsicCalibration_){
      return this->template get<_vep>();
    } else {
      return this->template get<_aux>().MrMV_;
    }
  }
  const V3D& get_MrMV() const{
    if(this->template get <_aux>().enableExtrinsicCalibration_){
      return this->template get<_vep>();
    } else {
      return this->template get<_aux>().MrMV_;
    }
  }
  QPD& get_qVM(){
    if(this->template get <_aux>().enableExtrinsicCalibration_){
      return this->template get<_vea>();
    } else {
      return this->template get<_aux>().qVM_;
    }
  }
  const QPD& get_qVM() const{
    if(this->template get <_aux>().enableExtrinsicCalibration_){
      return this->template get<_vea>();
    } else {
      return this->template get<_aux>().qVM_;
    }
  }
};
/*!
 * Prediction measurement class, contains references to subentries:
 * - acc: accelerometer measurement
 * - gyr: gyroscope measurement
 */
class PredictionMeas: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
  static constexpr unsigned int _acc = 0;
  static constexpr unsigned int _gyr = _acc+1;
  /*!
   * Constructor
   */
  PredictionMeas(){
    static_assert(_gyr+1==E_,"Error with indices");
    this->template getName<_acc>() = "acc";
    this->template getName<_gyr>() = "gyr";
  }
  ~PredictionMeas(){};
};

template<typename STATE>
class PredictionNoise: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,7>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nDynamicTags_>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nDynamicTags_>,
      LWF::ArrayElement<LWF::VectorElement<2>,STATE::nHybridTags_>>{
 public:
  using LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,7>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nDynamicTags_>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nDynamicTags_>,
      LWF::ArrayElement<LWF::VectorElement<2>,STATE::nHybridTags_>>::E_;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _acb = _vel+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _vep = _gyb+1;
  static constexpr unsigned int _att = _vep+1;
  static constexpr unsigned int _vea = _att+1;
  static constexpr unsigned int _dyp = _vea+(int)(STATE::nDynamicTags_>0);
  static constexpr unsigned int _dya = _dyp+(int)(STATE::nDynamicTags_>0);
  static constexpr unsigned int _hya = _dya+(int)(STATE::nHybridTags_>0);
  /*!
   * Constructor
   */
  PredictionNoise(){
    static_assert(_hya+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_vep>() = "vep";
    this->template getName<_att>() = "att";
    this->template getName<_vea>() = "vea";
    if((int)(STATE::nDynamicTags_>0)) this->template getName<_dyp>() = "dyp";
    if((int)(STATE::nDynamicTags_>0)) this->template getName<_dya>() = "dya";
    if((int)(STATE::nHybridTags_>0)) this->template getName<_hya>() = "hya";
  }
  ~PredictionNoise(){};
};

template<int nDynamicTags, int nHybridTags>
class FilterState: public LWF::FilterState<State<nDynamicTags,nHybridTags>,PredictionMeas,PredictionNoise<State<nDynamicTags,nHybridTags>>,0,true>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::FilterState<State<nDynamicTags,nHybridTags>,PredictionMeas,PredictionNoise<State<nDynamicTags,nHybridTags>>,0,true> Base;
  typedef typename Base::mtState mtState;
  using Base::state_;
  using Base::cov_;
  using Base::usePredictionMerge_;
  Eigen::Matrix<double,6,6> dynamicTagInitCov_;
  bool hasSeenStaticTag_;
  /*!
   * Constructor
   */
  FilterState(){
    dynamicTagInitCov_.setIdentity();
    hasSeenStaticTag_ = false;
  }

  /*!
   * Adds a new dynamic tag to the filter at index newInd and with tag ID tagId.
   * VrVT and qTV are used for initializing the tag position.
   */
  void makeNewDynamicTag(unsigned int newInd,int tagId,const Eigen::Vector3d& VrVT,const rot::RotationQuaternionPD& qTV){
    state_.template get<mtState::_aux>().dynamicIds_[newInd] = tagId;
    resetTagPoseAndCovariance(newInd,VrVT,qTV);
  }

  /*!
   * Reset the tag pose and pose covariance for a specific tag.
   */
  void resetTagPoseAndCovariance(int tagIndex, const Eigen::Vector3d& VrVT, const rot::RotationQuaternionPD& qTV){
    state_.template get<mtState::_dyp>(tagIndex) = VrVT;
    state_.template get<mtState::_dya>(tagIndex) = qTV;
    // Reset the covariance terms
    cov_.template block<mtState::D_,3>(0,mtState::template getId<mtState::_dyp>(tagIndex)).setZero();
    cov_.template block<3,mtState::D_>(mtState::template getId<mtState::_dyp>(tagIndex),0).setZero();
    cov_.template block<mtState::D_,3>(0,mtState::template getId<mtState::_dya>(tagIndex)).setZero();
    cov_.template block<3,mtState::D_>(mtState::template getId<mtState::_dya>(tagIndex),0).setZero();
    cov_.template block<3,3>(mtState::template getId<mtState::_dyp>(tagIndex),mtState::template getId<mtState::_dyp>(tagIndex)) = dynamicTagInitCov_.template block<3,3>(0,0);
    cov_.template block<3,3>(mtState::template getId<mtState::_dyp>(tagIndex),mtState::template getId<mtState::_dya>(tagIndex)) = dynamicTagInitCov_.template block<3,3>(0,3);
    cov_.template block<3,3>(mtState::template getId<mtState::_dya>(tagIndex),mtState::template getId<mtState::_dyp>(tagIndex)) = dynamicTagInitCov_.template block<3,3>(3,0);
    cov_.template block<3,3>(mtState::template getId<mtState::_dya>(tagIndex),mtState::template getId<mtState::_dya>(tagIndex)) = dynamicTagInitCov_.template block<3,3>(3,3);
  }

  /*!
   * Reset the tag orientation and orientation covariance for a specific tag. This is used when a tag has been wrongly initialized.
   */
  void resetTagOrientationAndCovariance(int tagIndex, const rot::RotationQuaternionPD& qTV){
	state_.template get<mtState::_dya>(tagIndex) = qTV;
    cov_.template block<mtState::D_,3>(0,mtState::template getId<mtState::_dya>(tagIndex)).setZero();
    cov_.template block<3,mtState::D_>(mtState::template getId<mtState::_dya>(tagIndex),0).setZero();
    cov_.template block<3,3>(mtState::template getId<mtState::_dya>(tagIndex),mtState::template getId<mtState::_dya>(tagIndex)) = dynamicTagInitCov_.template block<3,3>(3,3);
  }

  /*!
   * Adds a new dynamic tag to the filter at index newInd and with tag ID tagId.
   * VrVT and qTV are used for initializing the tag position.
   */
  void removeDynamicTag(int tagId){
    const unsigned ind = state_.template get<mtState::_aux>().getDynamicIndFromTagId(tagId);
    if(ind != -1){
      state_.template get<mtState::_aux>().dynamicIds_[ind] = -1;
      cov_.template block<mtState::D_,3>(0,mtState::template getId<mtState::_dyp>(ind)).setZero();
      cov_.template block<3,mtState::D_>(mtState::template getId<mtState::_dyp>(ind),0).setZero();
      cov_.template block<mtState::D_,3>(0,mtState::template getId<mtState::_dya>(ind)).setZero();
      cov_.template block<3,mtState::D_>(mtState::template getId<mtState::_dya>(ind),0).setZero();
      cov_.template block<3,3>(mtState::template getId<mtState::_dyp>(ind),mtState::template getId<mtState::_dyp>(ind)).setIdentity();
      cov_.template block<3,3>(mtState::template getId<mtState::_dya>(ind),mtState::template getId<mtState::_dya>(ind)).setIdentity();
    }
  }
  void initWithAccelerometer(const V3D& fMeasInit){
    V3D unitZ(0,0,1);
    if(fMeasInit.norm()>1e-6){
      state_.template get<mtState::_att>().setFromVectors(unitZ,fMeasInit);
    } else {
      state_.template get<mtState::_att>().setIdentity();
    }
    hasSeenStaticTag_ = false;
  }
  void alignToStaticTag(const V3D& IrIT, const QPD& qTI, const V3D& VrVT, const QPD& qTV){
    // qIM_2 = qTI^T*qTV*qVM
    // (qIM_2*q)^T*g = q^T*qIM_2^T*g = qIM_1^T*g
    // qIM_2^T*g = q*(qIM_1^T*g)
    // qIM_3 = qIM_2*q
    // IrIM = IrIT - qIM*(qVM^T*VrVT + MrMV)
    QPD qIM_2 = qTI.inverted()*qTV*state_.get_qVM();
    V3D unitZ(0,0,1);
    QPD q;
    q.setFromVectors(qIM_2.inverseRotate(unitZ),state_.template get<mtState::_att>().inverseRotate(unitZ));
    state_.template get<mtState::_att>() = qIM_2*q;
    state_.template get<mtState::_pos>() = IrIT - state_.template get<mtState::_att>().rotate(V3D(state_.get_qVM().inverseRotate(VrVT) + state_.get_MrMV()));
  }
};

}


#endif /* RCARS_FILTERSTATES_HPP_ */
