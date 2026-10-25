// SPDX-License-Identifier: MIT
#include "mid360_sim/Mid360Lidar.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/RaycastData.hh>

namespace mid360_sim
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

template <typename T>
T SdfValue(const std::shared_ptr<const sdf::Element> &_sdf,
           const std::string &_name, const T &_default)
{
  if (!_sdf || !_sdf->HasElement(_name))
    return _default;
  return _sdf->Get<T>(_name);
}
}  // namespace

Mid360Lidar::Mid360Lidar() = default;

Mid360Lidar::~Mid360Lidar() = default;

bool Mid360Lidar::LoadPattern(const std::string &_path, std::string &_error)
{
  std::ifstream input(_path);
  if (!input.is_open())
  {
    _error = "failed to open pattern file: " + _path;
    return false;
  }

  std::string line;
  // The Livox CSV starts with "Time/s,Azimuth/deg,Zenith/deg".
  std::getline(input, line);

  this->pattern.clear();
  this->pattern.reserve(200000);

  size_t lineNumber = 1;
  while (std::getline(input, line))
  {
    ++lineNumber;
    if (line.empty())
      continue;

    double time = 0.0;
    double azimuthDeg = 0.0;
    double zenithDeg = 0.0;
    int parsed = std::sscanf(line.c_str(), "%lf,%lf,%lf", &time, &azimuthDeg,
                             &zenithDeg);
    if (parsed != 3)
    {
      // Also accept a two-column file: Azimuth/deg,Zenith/deg.
      parsed = std::sscanf(line.c_str(), "%lf,%lf", &azimuthDeg, &zenithDeg);
      if (parsed != 2)
      {
        gzwarn << "[mid360] skipping malformed pattern line " << lineNumber
               << std::endl;
        continue;
      }
    }

    if (!std::isfinite(azimuthDeg) || !std::isfinite(zenithDeg))
      continue;

    // Zenith is measured from +Z, so elevation above the horizon is 90 - zen.
    // (The Gazebo Classic Livox plugin used zenith - 90, which mirrors the
    // point cloud vertically; that is not what the real Mid-360 reports.)
    const double elevation = (90.0 - zenithDeg) * kDegToRad;
    const double azimuth = azimuthDeg * kDegToRad;
    const double cosElevation = std::cos(elevation);

    PatternDirection direction;
    direction.x = static_cast<float>(cosElevation * std::cos(azimuth));
    direction.y = static_cast<float>(cosElevation * std::sin(azimuth));
    direction.z = static_cast<float>(std::sin(elevation));
    this->pattern.push_back(direction);
  }

  if (this->pattern.empty())
  {
    _error = "pattern file contains no usable rows: " + _path;
    return false;
  }

  // Validate the FOV, so a wrong file (e.g. an Avia scan table) is loud.
  double minZ = std::numeric_limits<double>::max();
  double maxZ = std::numeric_limits<double>::lowest();
  for (const auto &d : this->pattern)
  {
    minZ = std::min(minZ, static_cast<double>(d.z));
    maxZ = std::max(maxZ, static_cast<double>(d.z));
  }
  std::cerr << "[mid360] loaded " << this->pattern.size()
        << " pattern directions from " << _path << " (elevation "
        << std::asin(minZ) / kDegToRad << " .. "
        << std::asin(maxZ) / kDegToRad << " deg)" << std::endl;
  return true;
}

std::string Mid360Lidar::ResolvePatternPath(const std::string &_path)
{
  namespace fs = std::filesystem;

  std::vector<fs::path> candidates;
  const fs::path requested(_path);
  if (requested.is_absolute())
    candidates.push_back(requested);
  else
    candidates.push_back(requested);

  if (const char *env = std::getenv("MID360_PATTERN_FILE"))
    candidates.emplace_back(env);

  Dl_info info;
  if (dladdr(reinterpret_cast<void *>(&Mid360Lidar::ResolvePatternPath),
             &info) != 0 &&
      info.dli_fname)
  {
    const fs::path libDir = fs::path(info.dli_fname).parent_path();
    const std::string name = requested.filename().string();
    candidates.push_back(libDir / "patterns" / name);
    candidates.push_back(libDir / ".." / "patterns" / name);
    candidates.push_back(libDir / ".." / "share" / "mid360_sim" /
                        "patterns" / name);
    candidates.push_back(libDir / ".." / ".." / "share" / "mid360_sim" /
                        "patterns" / name);
  }

  for (const auto &candidate : candidates)
  {
    std::error_code ec;
    if (candidate.empty() || !fs::exists(candidate, ec))
      continue;
    return fs::weakly_canonical(candidate, ec).string();
  }
  return {};
}

PatternDirection Mid360Lidar::CycleDirection(
    const PatternDirection &_dir) const
{
  if (this->azimuthCycleOffsetDeg == 0.0 || this->cycleCount == 0)
    return _dir;

  PatternDirection rotated;
  rotated.x = static_cast<float>(this->cycleCos * _dir.x -
                                 this->cycleSin * _dir.y);
  rotated.y = static_cast<float>(this->cycleSin * _dir.x +
                                 this->cycleCos * _dir.y);
  rotated.z = _dir.z;
  return rotated;
}

void Mid360Lidar::Configure(const gz::sim::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;

  this->topic = SdfValue<std::string>(_sdf, "topic", "/mid360/points");
  this->frameId =
      SdfValue<std::string>(_sdf, "frame_id", "mini_car/base_link/mid360");
  const std::string linkName =
      SdfValue<std::string>(_sdf, "link_name", "base_link");
  const std::string patternFile =
      SdfValue<std::string>(_sdf, "pattern_file", "mid360.csv");

  this->pointsPerScan = static_cast<size_t>(
      std::max(1, SdfValue<int>(_sdf, "points_per_scan", 20000)));
  this->pointStride = static_cast<size_t>(
      std::max(1, SdfValue<int>(_sdf, "point_stride", 1)));
  const double scanRate =
      std::max(0.1, SdfValue<double>(_sdf, "scan_rate_hz", 10.0));
  this->scanPeriodNs =
      static_cast<int64_t>(std::llround(1.0e9 / scanRate));
  this->pointPeriodNs = std::max<int64_t>(
      1, this->scanPeriodNs / static_cast<int64_t>(this->pointsPerScan));

  this->rangeMin = SdfValue<double>(_sdf, "range_min", 0.1);
  this->rangeMax = SdfValue<double>(_sdf, "range_max", 40.0);
  if (this->rangeMax <= this->rangeMin)
  {
    gzerr << "[mid360] range_max must be greater than range_min; using 40/0.1"
          << std::endl;
    this->rangeMin = 0.1;
    this->rangeMax = 40.0;
  }

  this->rangeNoiseStddev =
      std::max(0.0, SdfValue<double>(_sdf, "range_noise_stddev", 0.01));
  this->intensity = SdfValue<double>(_sdf, "intensity", 45.0);
  this->dropoutProbability = gz::math::clamp(
      SdfValue<double>(_sdf, "dropout_probability", 0.0), 0.0, 1.0);
  this->lineCount = static_cast<uint8_t>(
      std::max(1, std::min(255, SdfValue<int>(_sdf, "line_count", 4))));
  this->noiseSeed =
      static_cast<uint64_t>(std::max(0, SdfValue<int>(_sdf, "seed", 42)));
  this->azimuthCycleOffsetDeg =
      SdfValue<double>(_sdf, "azimuth_cycle_offset_deg", 137.507764);
  this->logInterval =
      static_cast<uint64_t>(std::max(0, SdfValue<int>(_sdf, "log_interval", 100)));

  if (_sdf && _sdf->HasElement("sensor_pose"))
  {
    std::istringstream poseStream(_sdf->Get<std::string>("sensor_pose"));
    double x = 0.0, y = 0.0, z = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    if (poseStream >> x >> y >> z >> roll >> pitch >> yaw)
    {
      gz::math::Quaterniond rotation;
      rotation.SetFromEuler(roll, pitch, yaw);
      this->sensorPoseLink.Set(gz::math::Vector3d(x, y, z), rotation);
    }
    else
    {
      gzwarn << "[mid360] could not parse <sensor_pose>; using 0 0 0.40 0 0 0"
             << std::endl;
      this->sensorPoseLink.Set(gz::math::Vector3d(0.0, 0.0, 0.40),
                               gz::math::Quaterniond::Identity);
    }
  }
  else
  {
    this->sensorPoseLink.Set(gz::math::Vector3d(0.0, 0.0, 0.40),
                             gz::math::Quaterniond::Identity);
  }
  this->sensorPositionLink = this->sensorPoseLink.Pos();
  this->sensorRotationLink = this->sensorPoseLink.Rot();

  this->rng.seed(static_cast<std::mt19937::result_type>(this->noiseSeed));
  this->rangeNoise = std::normal_distribution<double>(0.0, 1.0);
  this->uniform = std::uniform_real_distribution<double>(0.0, 1.0);

  // -----------------------------------------------------------------------
  // Locate the link that carries the rays.
  // -----------------------------------------------------------------------
  this->linkEntity = _ecm.EntityByComponents(
      gz::sim::components::Link(),
      gz::sim::components::ParentEntity(this->modelEntity),
      gz::sim::components::Name(linkName));

  if (this->linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[mid360] link [" << linkName
          << "] not found in model entity " << this->modelEntity
          << "; plugin disabled" << std::endl;
    return;
  }

  if (nullptr == _ecm.Component<gz::sim::components::Pose>(this->linkEntity))
  {
    gzerr << "[mid360] link [" << linkName
          << "] has no Pose component; plugin disabled" << std::endl;
    return;
  }

  // The physics system updates results for every entity that has a
  // RaycastData component. Creating it in Configure() makes the physics
  // system start raycasting before the first simulation step.
  if (nullptr ==
      _ecm.Component<gz::sim::components::RaycastData>(this->linkEntity))
  {
    if (nullptr == _ecm.CreateComponent(
                       this->linkEntity,
                       gz::sim::components::RaycastData()))
    {
      gzerr << "[mid360] failed to create RaycastData component on link ["
            << linkName << "]" << std::endl;
      return;
    }
  }

  // -----------------------------------------------------------------------
  // Pattern file.
  // -----------------------------------------------------------------------
  const std::string resolvedPath = ResolvePatternPath(patternFile);
  if (resolvedPath.empty())
  {
    gzerr << "[mid360] pattern file [" << patternFile
          << "] not found. Set <pattern_file> to an absolute path or export "
             "MID360_PATTERN_FILE" << std::endl;
    return;
  }

  std::string error;
  if (!LoadPattern(resolvedPath, error))
  {
    gzerr << "[mid360] " << error << std::endl;
    return;
  }

  if (this->pattern.size() < this->pointsPerScan)
  {
    gzerr << "[mid360] pattern has " << this->pattern.size()
          << " points, fewer than points_per_scan=" << this->pointsPerScan
          << std::endl;
    return;
  }

  if (this->pattern.size() % this->pointsPerScan != 0)
  {
    gzwarn << "[mid360] pattern size " << this->pattern.size()
           << " is not a multiple of points_per_scan="
           << this->pointsPerScan
           << "; frames will wrap in the middle of the pattern" << std::endl;
  }

  // -----------------------------------------------------------------------
  // gz-transport publisher.
  // -----------------------------------------------------------------------
  this->node = std::make_unique<gz::transport::Node>();
  this->publisher =
      this->node->Advertise<gz::msgs::PointCloudPacked>(this->topic);

  // Field order is chosen so every float is naturally aligned and the
  // resulting point_step is 23 bytes. GLIM recognises "t" as per-point time.
  this->cloudTemplate.Clear();
  gz::msgs::InitPointCloudPacked(
      this->cloudTemplate, this->frameId, false,
      {{"x", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"y", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"z", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"intensity", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"t", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"tag", gz::msgs::PointCloudPacked::Field::UINT8},
       {"line", gz::msgs::PointCloudPacked::Field::UINT8},
       {"ring", gz::msgs::PointCloudPacked::Field::UINT8}});

  if (this->cloudTemplate.point_step() != sizeof(OutputPoint))
  {
    gzerr << "[mid360] internal layout mismatch: point_step="
          << this->cloudTemplate.point_step()
          << " sizeof(OutputPoint)=" << sizeof(OutputPoint) << std::endl;
    return;
  }

  this->pendingRows.reserve(this->pointsPerScan);
  this->pendingTimes.reserve(this->pointsPerScan);
  this->framePoints.reserve(this->pointsPerScan);

  this->configured = true;

  std::cerr << "[mid360] emulating Mid-360 on link [" << linkName
        << "], sensor pose " << this->sensorPoseLink
        << ", frame_id [" << this->frameId << "], topic [" << this->topic
        << "], " << this->pointsPerScan << " pattern rows @ " << scanRate
        << " Hz, point_stride " << this->pointStride << " ("
        << (this->pointsPerScan + this->pointStride - 1) /
               this->pointStride
        << " rays/frame), per-point dt " << this->pointPeriodNs << " ns"
        << std::endl;
}

void Mid360Lidar::PreUpdate(const gz::sim::UpdateInfo &_info,
                            gz::sim::EntityComponentManager &_ecm)
{
  if (!this->configured)
    return;

  auto *raycast =
      _ecm.Component<gz::sim::components::RaycastData>(this->linkEntity);
  if (nullptr == raycast)
    return;

  auto &rays = raycast->Data().rays;

  if (_info.paused)
  {
    rays.clear();
    this->pendingCount = 0;
    return;
  }

  rays.clear();

  const int64_t nowNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             _info.simTime)
                             .count();
  const int64_t dtNs =
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.dt).count();

  if (!this->scanActive)
  {
    this->scanActive = true;
    this->scanReady = false;
    this->scanNextIndex = 0;
    this->scanStartNs = nowNs;

    // Keep frames phase-locked to the configured rate. If the simulator
    // stepped past the scheduled start, fall back to "now" instead of
    // catching up with a burst of points.
    if (this->nextScanStartNs > 0 &&
        this->nextScanStartNs <= nowNs + dtNs &&
        nowNs - this->nextScanStartNs < this->scanPeriodNs)
    {
      this->scanStartNs = this->nextScanStartNs;
    }

    this->framePoints.clear();
    this->pendingCount = 0;
  }

  // Raycast at the pose the link will have at the END of this step, because
  // Physics::Update performs the intersection after stepping.
  const int64_t elapsedNs =
      std::max<int64_t>(0, nowNs + dtNs - this->scanStartNs);
  // Number of pattern rows whose timestamp is <= the end of this step.
  // Integer math avoids drift when pointPeriodNs does not divide evenly.
  size_t due = static_cast<size_t>(
                   (elapsedNs * static_cast<int64_t>(this->pointsPerScan)) /
                   this->scanPeriodNs) +
               1;
  due = std::min(due, this->pointsPerScan);

  if (due > this->scanNextIndex)
  {
    // Emit every pointStride-th row. The row index still determines the
    // timestamp, so even a strided scan has the real Mid-360 timing
    // (e.g. stride 2 -> one point every 10 us instead of 5 us).
    size_t count = 0;
    for (size_t index = this->scanNextIndex; index < due; ++index)
    {
      if (index % this->pointStride == 0)
        ++count;
    }

    this->pendingRows.resize(count);
    this->pendingTimes.resize(count);
    rays.reserve(count);

    const double scanPeriodSec =
        static_cast<double>(this->scanPeriodNs) * 1e-9;

    size_t k = 0;
    for (size_t index = this->scanNextIndex; index < due; ++index)
    {
      if (index % this->pointStride != 0)
        continue;

      const uint64_t row =
          (this->patternCursor + index) % this->pattern.size();
      const PatternDirection direction = this->CycleDirection(this->pattern[row]);
      const gz::math::Vector3d directionLink =
          this->sensorRotationLink.RotateVector(
              gz::math::Vector3d(direction.x, direction.y, direction.z));

      gz::sim::components::RayInfo ray;
      ray.start = this->sensorPositionLink;
      ray.end = this->sensorPositionLink + directionLink * this->rangeMax;
      rays.push_back(ray);

      this->pendingRows[k] = row;
      this->pendingTimes[k] = static_cast<float>(
          scanPeriodSec * static_cast<double>(index) /
          static_cast<double>(this->pointsPerScan));
      ++k;
    }

    this->pendingCount = count;
    this->scanNextIndex = due;
  }
  else
  {
    this->pendingCount = 0;
  }

  if (this->scanNextIndex >= this->pointsPerScan)
    this->scanReady = true;
}

void Mid360Lidar::PostUpdate(const gz::sim::UpdateInfo &_info,
                             const gz::sim::EntityComponentManager &_ecm)
{
  if (!this->configured || _info.paused || !this->scanActive)
    return;

  const auto *raycast =
      _ecm.Component<gz::sim::components::RaycastData>(this->linkEntity);
  if (nullptr == raycast)
    return;

  const auto &results = raycast->Data().results;

  if (this->pendingCount > 0)
  {
    if (results.size() != this->pendingCount)
    {
      if (!this->warnedMissingResults)
      {
        gzwarn << "[mid360] physics engine returned " << results.size()
               << " intersections for " << this->pendingCount
               << " rays. Raycasting may be unsupported (the default dartsim "
                  "engine supports it)." << std::endl;
        this->warnedMissingResults = true;
      }
      ++this->missedChunkCounter;
    }
    else
    {
      for (size_t k = 0; k < this->pendingCount; ++k)
      {
        const double fraction = results[k].fraction;
        ++this->rayCounter;

        // No hit: the dartsim backend reports fraction 0 or (at) 1.0
        // depending on the engine version. Both mean "no return".
        if (!std::isfinite(fraction) || fraction <= 0.0 ||
            fraction >= 1.0 - 1e-9)
        {
          continue;
        }

        double distance = fraction * this->rangeMax;
        if (this->rangeNoiseStddev > 0.0)
          distance += this->rangeNoise(this->rng) * this->rangeNoiseStddev;

        if (!std::isfinite(distance) || distance < this->rangeMin ||
            distance > this->rangeMax)
        {
          continue;
        }

        if (this->dropoutProbability > 0.0 &&
            this->uniform(this->rng) < this->dropoutProbability)
        {
          continue;
        }

        // Results are in the link frame, but the published cloud must be in
        // the sensor frame.  Since the distance is measured along the ray,
        // dir_sensor * distance is exact and avoids a frame round-trip.
        const PatternDirection direction =
            this->CycleDirection(this->pattern[this->pendingRows[k]]);

        OutputPoint point;
        point.x = static_cast<float>(direction.x * distance);
        point.y = static_cast<float>(direction.y * distance);
        point.z = static_cast<float>(direction.z * distance);
        point.intensity = static_cast<float>(this->intensity);
        point.t = this->pendingTimes[k];
        point.tag = 0;
        point.line = static_cast<uint8_t>(this->pendingRows[k] %
                                          this->lineCount);
        point.ring = point.line;

        this->framePoints.push_back(point);
        ++this->hitCounter;
      }
    }
    this->pendingCount = 0;
  }

  if (!this->scanReady)
    return;

  this->PublishFrame(this->scanStartNs, this->framePoints);
  ++this->frameCounter;

  if (this->frameCounter == 1)
  {
    float minT = 0.0f;
    float maxT = 0.0f;
    if (!this->framePoints.empty())
    {
      minT = this->framePoints.front().t;
      maxT = this->framePoints.back().t;
    }
    std::cerr << "[mid360] first frame published: "
              << this->framePoints.size() << " points, t "
              << minT << " .. " << maxT << " s" << std::endl;
  }

  // Advance the replay cursor by one complete frame.
  this->patternCursor =
      (this->patternCursor + this->pointsPerScan) % this->pattern.size();
  if (this->patternCursor == 0)
  {
    // The recorded file is four seconds long. Rotating the pattern by the
    // golden angle each time the file wraps keeps the scan non-repetitive
    // for arbitrarily long runs while preserving the measured structure.
    ++this->cycleCount;
    const double angle =
        static_cast<double>(this->cycleCount) *
        this->azimuthCycleOffsetDeg * kDegToRad;
    this->cycleCos = std::cos(angle);
    this->cycleSin = std::sin(angle);
  }

  this->nextScanStartNs = this->scanStartNs + this->scanPeriodNs;
  this->scanActive = false;
  this->scanReady = false;
  this->scanNextIndex = 0;

  if (this->logInterval > 0 &&
      this->frameCounter % this->logInterval == 0)
  {
    const double hitRatio =
        this->rayCounter > 0
            ? static_cast<double>(this->hitCounter) /
                  static_cast<double>(this->rayCounter)
            : 0.0;
    std::cerr << "[mid360] frame " << this->frameCounter << ": "
          << this->framePoints.size() << " points, " << this->pendingCount
          << " pending, hit ratio " << hitRatio << ", missed chunks "
          << this->missedChunkCounter << ", sim time "
          << std::chrono::duration<double>(_info.simTime).count() << " s"
          << std::endl;
  }
}

void Mid360Lidar::PublishFrame(int64_t _startNs,
                               const std::vector<OutputPoint> &_points)
{
  gz::msgs::PointCloudPacked msg = this->cloudTemplate;

  msg.set_height(1);
  msg.set_width(static_cast<uint32_t>(_points.size()));
  msg.set_row_step(msg.point_step() *
                   static_cast<uint32_t>(_points.size()));
  msg.set_is_dense(true);

  auto *stamp = msg.mutable_header()->mutable_stamp();
  stamp->set_sec(_startNs / 1000000000LL);
  stamp->set_nsec(static_cast<int32_t>(_startNs % 1000000000LL));

  auto *data = msg.mutable_data();
  data->resize(_points.size() * msg.point_step());
  if (!_points.empty())
  {
    std::memcpy(&(*data)[0], _points.data(), data->size());
  }

  this->publisher.Publish(msg);
}
}  // namespace mid360_sim

GZ_ADD_PLUGIN(mid360_sim::Mid360Lidar, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(mid360_sim::Mid360Lidar, "mid360_lidar")
GZ_ADD_PLUGIN_ALIAS(mid360_sim::Mid360Lidar, "mid360_sim::Mid360Lidar")
