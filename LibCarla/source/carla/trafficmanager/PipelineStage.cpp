// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/trafficmanager/PipelineStage.h"

namespace carla {
namespace traffic_manager {

PipelineStage::PipelineStage(
    const std::string &stage_name)
  : stage_name(stage_name),
    performance_diagnostics(PerformanceDiagnostics(stage_name)) {
}

PipelineStage::~PipelineStage() {}

void PipelineStage::Update() {
  // Receive data from previous stage.
  DataReceiver();

  performance_diagnostics.RegisterUpdate(true);
  // Stage operation logic.
  Action();
  performance_diagnostics.RegisterUpdate(false);

  // Send data to next stage.
  DataSender();
}

} // namespace traffic_manager
} // namespace carla
