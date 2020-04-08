
/// This file contains definitions of data structures used in traffic manager.

#pragma once

#include <deque>
#include <string>
#include <vector>

#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Vector3D.h"
#include "carla/rpc/ActorId.h"
#include "carla/rpc/Command.h"
#include "carla/rpc/TrafficLightState.h"

#include "carla/trafficmanager/SimpleWaypoint.h"

namespace carla
{
namespace traffic_manager
{

namespace cc = carla::client;
namespace cg = carla::geom;

using ActorId = carla::ActorId;
using ActorPtr = carla::SharedPtr<cc::Actor>;

using SimpleWaypointPtr = std::shared_ptr<SimpleWaypoint>;
using Buffer = std::deque<SimpleWaypointPtr>;
using BufferMap = std::unordered_map<carla::ActorId, Buffer>;
using BufferMapPtr = std::shared_ptr<BufferMap>;

using TLS = carla::rpc::TrafficLightState;

struct KinematicState
{
  bool physics_enabled;
  cg::Location location;
  cg::Rotation rotation;
  cg::Vector3D velocity;
};
using KinematicStateMap = std::unordered_map<ActorId, KinematicState>;

enum ActorType
{
  Vehicle,
  Pedestrian,
  Any
};

struct StaticAttributes
{
  ActorType actor_type;
  float half_length;
  float half_width;
  float half_height;
  float speed_limit;
};
using StaticAttributeMap = std::unordered_map<ActorId, StaticAttributes>;

struct TrafficLightState
{
  TLS tl_state;
  bool at_traffic_light;
};
using TrafficLightStateMap = std::unordered_map<ActorId, TrafficLightState>;

struct CollisionLock
{
  ActorId lead_vehicle_id;
  double distance_to_lead_vehicle;
  double initial_lock_distance;
};
using CollisionLockMap = std::unordered_map<ActorId, CollisionLock>;

struct CollisionHazardData
{
  bool hazard;
  float available_distance_margin;
  ActorId hazard_actor_id;
};
using CollisionFrame = std::vector<CollisionHazardData>;
using CollisionFramePtr = std::shared_ptr<CollisionFrame>;

using ControlFrame = std::vector<carla::rpc::Command>;
using ControlFramePtr = std::shared_ptr<ControlFrame>;

using TLFrame = std::vector<bool>;
using TLFramePtr = std::shared_ptr<TLFrame>;

} // namespace traffic_manager
} // namespace carla
