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

#include "FilterRCARS.hpp"

int main(int argc, char** argv){
  unsigned int s;

  typedef rcars::FilterRCARS<3,4> mtFilter;
  typedef mtFilter::mtFilterState mtFilterState;
  typedef mtFilterState::mtState mtState;
  mtState state;
  state.setRandom(s);
  state.template get<mtState::_aux>().dynamicIds_[0] = 0;

  rcars::ImuPrediction<mtFilterState> mPrediction;
  mPrediction.testJacs(1e-8,1e-5,0.1);

  rcars::TagUpdate<mtFilterState> mTagUpdate;
  typedef rcars::TagUpdate<mtFilterState>::mtMeas mtTagUpdateMeas;
  mtTagUpdateMeas tagUpdateMeas;
  tagUpdateMeas.get<mtTagUpdateMeas::_aux>().resize(1);
  tagUpdateMeas.get<mtTagUpdateMeas::_aux>().tagIds_[0] = 0;
  tagUpdateMeas.get<mtTagUpdateMeas::_aux>().tagTypes_[0] = rcars::DYNAMIC_TAG;
  tagUpdateMeas.get<mtTagUpdateMeas::_aux>().VrVC_[0] << 204.292, 152.97, 251.154, 152.893, 204.117, 192.845, 252.616, 191.779;
  tagUpdateMeas.get<mtTagUpdateMeas::_aux>().tagPos_[0] = V3D(-0.543614, -0.213167, 1.55529);
  tagUpdateMeas.get<mtTagUpdateMeas::_aux>().tagAtt_[0] = QPD(-0.192343, 0.972483, -0.0485181, 0.122176);
  tagUpdateMeas.get<mtTagUpdateMeas::_aux>().IrIT_[0] = V3D(-0.543614, -0.213167, 1.55529);
  tagUpdateMeas.get<mtTagUpdateMeas::_aux>().qTI_[0] = QPD(-0.192343, 0.972483, -0.0485181, 0.122176);
  tagUpdateMeas.print();
  mTagUpdate.testJacs(state,tagUpdateMeas,1e-8,1e-5,0.1);
  tagUpdateMeas.get<mtTagUpdateMeas::_aux>().tagTypes_[0] = rcars::STATIC_TAG;
  mTagUpdate.testJacs(state,tagUpdateMeas,1e-8,1e-5,0.1);

  return 0;
}
