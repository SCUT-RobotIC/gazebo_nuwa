// SPDX-License-Identifier: MIT
/*
 * Mid-360 LiDAR simulation for Gazebo Sim (gz-sim 8).
 *
 * Replays the real Livox Mid-360 non-repetitive scan pattern and raycasts each
 * point through the physics engine at that point's own time, then publishes a
 * gz PointCloudPacked carrying per-point relative timestamps.
 */

#ifndef MID360_SIM_MID360LIDAR_HH_
#define MID360_SIM_MID360LIDAR_HH_

#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace mid360_sim
{
/// \brief One ray of the recorded Mid-360 pattern, given as a unit direction.
struct PatternDirection
{
  float x;
  float y;
  float z;
};

/// \brief One output point. The memory layout must match the PointCloudPacked
///        fields declared in Configure().
#pragma pack(push, 1)
struct OutputPoint
{
  float x;
  float y;
  float z;
  float intensity;
  float t;      ///< seconds relative to the frame header stamp
  uint8_t tag;
  uint8_t line;
  uint8_t ring;
};
#pragma pack(pop)

static_assert(sizeof(OutputPoint) == 23,
              "OutputPoint layout must match the PointCloudPacked point_step");

/// \brief Gazebo Sim system plugin that emulates a Livox Mid-360.
///
/// Unlike gpu_lidar (a regular azimuth/elevation grid rendered in one instant),
/// this plugin:
///   * replays the measured Mid-360 ray sequence from a CSV pattern file;
///   * splits every 10 Hz frame into small chunks and raycasts each chunk at
///     the simulator pose of that instant, so a moving robot produces the same
///     intra-frame motion as the real sensor;
///   * stamps every point with its own time offset (5 us at 200 k points/s).
class Mid360Lidar : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate,
                    public gz::sim::ISystemPostUpdate
{
  public: Mid360Lidar();
  public: ~Mid360Lidar() override;

  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override;

  public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                          const gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Read the pattern CSV (Time/Azimuth/Zenith) into unit directions.
  ///        Returns false and fills \p _error on failure.
  private: bool LoadPattern(const std::string &_path, std::string &_error);

  /// \brief Try to find the pattern file when a relative path was configured.
  private: static std::string ResolvePatternPath(const std::string &_path);

  /// \brief Apply the per-cycle azimuth rotation used to avoid exact repeats
  ///        of the recorded 40-frame sequence.
  private: PatternDirection CycleDirection(
      const PatternDirection &_dir) const;

  /// \brief Publish the accumulated frame.
  private: void PublishFrame(int64_t _startNs,
                             const std::vector<OutputPoint> &_points);

  private: bool configured{false};

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity linkEntity{gz::sim::kNullEntity};

  private: std::unique_ptr<gz::transport::Node> node;
  private: gz::transport::Node::Publisher publisher;
  private: gz::msgs::PointCloudPacked cloudTemplate;

  private: std::vector<PatternDirection> pattern;
  private: std::string frameId;
  private: std::string topic;

  // Sensor mounting: pose in the link frame.
  private: gz::math::Pose3d sensorPoseLink{gz::math::Pose3d::Zero};
  private: gz::math::Vector3d sensorPositionLink{gz::math::Vector3d::Zero};
  private: gz::math::Quaterniond sensorRotationLink{
      gz::math::Quaterniond::Identity};

  // Scan settings.
  // pointsPerScan is the number of pattern rows in one 10 Hz frame (20000 at
  // the real 200 k points/s). pointStride > 1 emits every Nth row, keeping
  // full-FOV coverage and the original per-point timestamps while reducing
  // the raycasting cost.
  private: size_t pointsPerScan{20000};
  private: size_t pointStride{1};
  private: int64_t scanPeriodNs{100000000};
  private: int64_t pointPeriodNs{5000};
  private: double rangeMin{0.1};
  private: double rangeMax{40.0};
  private: double rangeNoiseStddev{0.01};
  private: double intensity{100.0};
  private: double dropoutProbability{0.0};
  private: uint8_t lineCount{4};
  private: uint64_t noiseSeed{42};
  private: double azimuthCycleOffsetDeg{137.507764};
  private: uint64_t logInterval{100};

  // Pattern playback state.
  private: uint64_t patternCursor{0};   ///< first row of the active scan
  private: uint64_t cycleCount{0};
  private: double cycleCos{1.0};
  private: double cycleSin{0.0};

  // Per-scan state.
  private: bool scanActive{false};
  private: bool scanReady{false};
  private: int64_t scanStartNs{0};
  private: int64_t nextScanStartNs{0};
  private: size_t scanNextIndex{0};
  private: std::vector<OutputPoint> framePoints;

  // Chunk emitted during the current simulation step.
  private: std::vector<uint64_t> pendingRows;
  private: std::vector<float> pendingTimes;
  private: size_t pendingCount{0};

  // Bookkeeping / diagnostics.
  private: std::mt19937 rng;
  private: std::normal_distribution<double> rangeNoise{0.0, 1.0};
  private: std::uniform_real_distribution<double> uniform{0.0, 1.0};
  private: uint64_t frameCounter{0};
  private: uint64_t rayCounter{0};
  private: uint64_t hitCounter{0};
  private: uint64_t missedChunkCounter{0};
  private: bool warnedMissingResults{false};
};
}  // namespace mid360_sim

#endif  // MID360_SIM_MID360LIDAR_HH_
