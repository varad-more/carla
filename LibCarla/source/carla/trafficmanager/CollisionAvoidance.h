
/// This file has functionality to detect potential collision with a nearby actor.

#include <memory>

#include "carla/Memory.h"

#include "boost/geometry.hpp"
#include "boost/geometry/geometries/geometries.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "boost/geometry/geometries/polygon.hpp"

#include "carla/trafficmanager/Constants.h"
#include "carla/trafficmanager/DataStructures.h"
#include "carla/trafficmanager/LocalizationUtils.h"
#include "carla/trafficmanager/SimpleWaypoint.h"

namespace carla
{
namespace traffic_manager
{

struct CollisionLock
{
  ActorId lead_vehicle_id;
  double distance_to_lead_vehicle;
  double initial_lock_distance;
};

struct GeometryComparison
{
  double reference_vehicle_to_other_geodesic;
  double other_vehicle_to_reference_geodesic;
  double inter_geodesic_distance;
  double inter_bbox_distance;
};

namespace bg = boost::geometry;

using Buffer = std::deque<std::shared_ptr<SimpleWaypoint>>;
using BufferMap = std::unordered_map<carla::ActorId, Buffer>;
using BufferMapPtr = std::shared_ptr<BufferMap>;
using LocationList = std::vector<cg::Location>;
using GeodesicBoundaryMap = std::unordered_map<ActorId, LocationList>;
using GeometryComparisonMap = std::unordered_map<ActorId, GeometryComparison>;
using KinematicStateMap = std::unordered_map<ActorId, KinematicState>;
using Point2D = bg::model::point<double, 2, bg::cs::cartesian>;
using Polygon = bg::model::polygon<bg::model::d2::point_xy<double>>;
using SimpleWaypointPtr = std::shared_ptr<SimpleWaypoint>;
using StaticAttributeMap = std::unordered_map<ActorId, StaticAttributes>;
using TrafficLightStateMap = std::unordered_map<ActorId, TrafficLightState>;
using TLS = carla::rpc::TrafficLightState;

using constants::WaypointSelection::JUNCTION_LOOK_AHEAD;

/// Returns the bounding box corners of the vehicle passed to the method.
LocationList GetBoundary(const KinematicState &kinematic_state, const StaticAttributes &attributes);

/// Method to calculate the speed dependent bounding box extention for a vehicle.
float GetBoundingBoxExtention(const KinematicState &kinematic_state);

/// Returns the extrapolated bounding box of the vehicle along its
/// trajectory.
LocationList GetGeodesicBoundary(const GeodesicBoundaryMap &geodesic_boundary_map,
                                 const KinematicState &kinematic_state,
                                 const StaticAttributes &attributes,
                                 const BufferMap &buffer);

/// Method to construct a boost polygon object.
Polygon GetPolygon(const LocationList &boundary);

/// The method returns true if ego_vehicle should stop and wait for
/// other_vehicle to pass along with available distance margin.
std::pair<bool, float> NegotiateCollision(const KinematicState &reference_vehicle_state,
                                          const KinematicState &other_vehicle_state,
                                          const StaticAttributes &reference_vehicle_attributes,
                                          const StaticAttributes &other_vehcile_attributes,
                                          const Buffer &reference_vehicle_buffer,
                                          const Buffer &other_vehicle_buffer);

/// Method to compute Geometry result between two vehicles
GeometryComparison GetGeometryBetweenActors(GeometryComparisonMap &geometry_cache,
                                            const KinematicState &reference_vehicle_state,
                                            const KinematicState &other_vehicle_state,
                                            const StaticAttributes &reference_vehicle_attributes,
                                            const StaticAttributes &other_vehcile_attributes,
                                            const Buffer &reference_vehicle_buffer,
                                            const Buffer &other_vehicle_buffer);

void CollisionAvoidance(const unsigned long index,
                        const std::vector<ActorId> &vehicle_id_list,
                        const KinematicStateMap &state_map,
                        const StaticAttributeMap &attribute_map,
                        const BufferMapPtr &buffer_map,
                        const TrackTraffic& track_traffic)
{
  GeodesicBoundaryMap geodesic_boundary_map;
  GeometryComparisonMap geometry_cache;

  const ActorId ego_actor_id = vehicle_id_list.at(index);
  const KinematicState &ego_kinematic_state = state_map.at(ego_actor_id);
  const cg::Location ego_location = ego_kinematic_state.location;
  const cg::Vector3D ego_velocity = ego_kinematic_state.velocity;
  const Buffer &ego_buffer = buffer_map->at(ego_actor_id);
  const SimpleWaypointPtr& closest_point = ego_buffer.front();
  const SimpleWaypointPtr junction_look_ahead = GetTargetWaypoint(ego_buffer, JUNCTION_LOOK_AHEAD).first;
  const ActorIdSet overlapping_actors = track_traffic.GetOverlappingVehicles(ego_actor_id);

  bool collision_hazard = false;
  float available_distance_margin = std::numeric_limits<float>::infinity();
  cg::Vector3D obstacle_velocity;
  const SimpleWaypointPtr safe_point_junction = data.safe_point_after_junction;

  try {
    // Check every actor in the vicinity if it poses a collision hazard.
    for (auto actor_info = collision_candidates.begin();
        actor_info != collision_candidates.end() && !collision_hazard;
        ++actor_info) {

      ActorId other_actor_id;
      Actor other_actor;
      cg::Vector3D other_velocity;
      std::tie(other_actor_id, other_actor, other_velocity) = *actor_info;

      if (!other_actor->IsAlive()) continue;
      const auto other_actor_type = other_actor->GetTypeId();
      const cg::Location other_location = other_actor->GetLocation();

      // Temporary fix to (0,0,0) bug
      if (!(other_location.x == 0 && other_location.y == 0 && other_location.z == 0)) {

        if (other_actor_id != ego_actor_id &&
            (cg::Math::DistanceSquared(ego_location, other_location)
            < std::pow(MAX_COLLISION_RADIUS, 2)) &&
            (std::abs(ego_location.z - other_location.z) < VERTICAL_OVERLAP_THRESHOLD)) {

          if (parameters.GetCollisionDetection(ego_actor, other_actor)) {

            std::pair<bool, float> negotiation_result = NegotiateCollision(ego_actor, other_actor, ego_location,
                                                                          other_location, closest_point, junction_look_ahead,
                                                                          ego_velocity, other_velocity);
            if ((safe_point_junction != nullptr
                && !IsLocationAfterJunctionSafe(ego_actor, other_actor, safe_point_junction,
                                                other_location, other_velocity))
                || negotiation_result.first)
            {

              if ((other_actor_type[0] == 'v' && parameters.GetPercentageIgnoreVehicles(ego_actor) <= (rand() % 101)) ||
                  (other_actor_type[0] == 'w' && parameters.GetPercentageIgnoreWalkers(ego_actor) <= (rand() % 101)))
              {
                collision_hazard = true;
                available_distance_margin = negotiation_result.second;
                obstacle_velocity = other_velocity;
              }
            }
          }
        }
      }
    }

  } catch (const std::exception &e) {
    carla::log_info("Actor might not be alive \n");
  }

  CollisionToPlannerData &message = current_planner_frame->at(i);
  message.hazard = collision_hazard;
  message.distance_to_other_vehicle = available_distance_margin;
  message.other_vehicle_velocity = obstacle_velocity;
}

} // namespace traffic_manager
} // namespace carla
