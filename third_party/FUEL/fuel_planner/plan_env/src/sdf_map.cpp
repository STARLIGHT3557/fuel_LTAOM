#include "plan_env/sdf_map.h"
#include "plan_env/map_ros.h"
#include <plan_env/raycast.h>

#include <cerrno>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace fast_planner {

namespace {

// Persisted grid format. The magic/version guard against loading a foreign or
// truncated file, and the recorded geometry must match the live map exactly:
// a grid from a different size/resolution/origin would be reused at wrong
// coordinates, which looks like "unknown obstacles" during flight.
const char kGridMagic[8] = {'L', 'T', 'A', 'O', 'M', 'G', 'R', 'D'};
const int kGridVersion = 1;

// mkdir -p without pulling in boost::filesystem or std::filesystem (the
// package is built as C++14).
void ensureDirectory(const std::string& path) {
  std::string current;
  current.reserve(path.size());
  for (size_t i = 0; i <= path.size(); ++i) {
    if (i == path.size() || path[i] == '/') {
      if (!current.empty() && current != ".") {
        if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
          ROS_WARN("SDFMap: cannot create directory %s (%s)", current.c_str(),
                   strerror(errno));
          return;
        }
      }
      if (i == path.size()) break;
      current.push_back('/');
    } else {
      current.push_back(path[i]);
    }
  }
}

std::string parentDirectory(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  if (slash == std::string::npos) return std::string();
  return path.substr(0, slash);
}

}  // namespace
SDFMap::SDFMap() {
}

SDFMap::~SDFMap() {
}

void SDFMap::initMap(ros::NodeHandle& nh) {
  mp_.reset(new MapParam);
  md_.reset(new MapData);
  mr_.reset(new MapROS);

  // Params of map properties
  double x_size, y_size, z_size;
  nh.param("sdf_map/resolution", mp_->resolution_, -1.0);
  nh.param("sdf_map/map_size_x", x_size, -1.0);
  nh.param("sdf_map/map_size_y", y_size, -1.0);
  nh.param("sdf_map/map_size_z", z_size, -1.0);
  nh.param("sdf_map/obstacles_inflation", mp_->obstacles_inflation_, -1.0);
  nh.param("sdf_map/local_bound_inflate", mp_->local_bound_inflate_, 1.0);
  nh.param("sdf_map/local_map_margin", mp_->local_map_margin_, 1);
  nh.param("sdf_map/ground_height", mp_->ground_height_, 1.0);
  nh.param("sdf_map/default_dist", mp_->default_dist_, 5.0);
  nh.param("sdf_map/optimistic", mp_->optimistic_, true);
  nh.param("sdf_map/signed_dist", mp_->signed_dist_, false);

  mp_->local_bound_inflate_ = max(mp_->resolution_, mp_->local_bound_inflate_);
  mp_->resolution_inv_ = 1 / mp_->resolution_;
  mp_->map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_->ground_height_);
  mp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  for (int i = 0; i < 3; ++i)
    mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
  mp_->map_min_boundary_ = mp_->map_origin_;
  mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;

  // Params of raycasting-based fusion
  nh.param("sdf_map/p_hit", mp_->p_hit_, 0.70);
  nh.param("sdf_map/p_miss", mp_->p_miss_, 0.35);
  nh.param("sdf_map/p_min", mp_->p_min_, 0.12);
  nh.param("sdf_map/p_max", mp_->p_max_, 0.97);
  nh.param("sdf_map/p_occ", mp_->p_occ_, 0.80);
  nh.param("sdf_map/max_ray_length", mp_->max_ray_length_, -0.1);
  nh.param("sdf_map/virtual_ceil_height", mp_->virtual_ceil_height_, -0.1);

  auto logit = [](const double& x) { return log(x / (1 - x)); };
  mp_->prob_hit_log_ = logit(mp_->p_hit_);
  mp_->prob_miss_log_ = logit(mp_->p_miss_);
  mp_->clamp_min_log_ = logit(mp_->p_min_);
  mp_->clamp_max_log_ = logit(mp_->p_max_);
  mp_->min_occupancy_log_ = logit(mp_->p_occ_);
  mp_->unknown_flag_ = 0.01;
  cout << "hit: " << mp_->prob_hit_log_ << ", miss: " << mp_->prob_miss_log_
       << ", min: " << mp_->clamp_min_log_ << ", max: " << mp_->clamp_max_log_
       << ", thresh: " << mp_->min_occupancy_log_ << endl;

  // Initialize data buffer of map
  int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
  md_->occupancy_buffer_ = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
  md_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_->distance_buffer_neg_ = vector<double>(buffer_size, mp_->default_dist_);
  md_->distance_buffer_ = vector<double>(buffer_size, mp_->default_dist_);
  md_->count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_->count_hit_ = vector<short>(buffer_size, 0);
  md_->count_miss_ = vector<short>(buffer_size, 0);
  md_->flag_rayend_ = vector<char>(buffer_size, -1);
  md_->flag_visited_ = vector<char>(buffer_size, -1);
  md_->tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_->tmp_buffer2_ = vector<double>(buffer_size, 0);
  md_->raycast_num_ = 0;
  md_->reset_updated_box_ = true;
  md_->update_min_ = md_->update_max_ = Eigen::Vector3d(0, 0, 0);

  // Try retriving bounding box of map, set box to map size if not specified
  vector<string> axis = { "x", "y", "z" };
  for (int i = 0; i < 3; ++i) {
    nh.param("sdf_map/box_min_" + axis[i], mp_->box_mind_[i], mp_->map_min_boundary_[i]);
    nh.param("sdf_map/box_max_" + axis[i], mp_->box_maxd_[i], mp_->map_max_boundary_[i]);
  }
  posToIndex(mp_->box_mind_, mp_->box_min_);
  posToIndex(mp_->box_maxd_, mp_->box_max_);

  // Initialize ROS wrapper
  mr_->setMap(this);
  mr_->node_ = nh;
  mr_->init();

  caster_.reset(new RayCaster);
  caster_->setParams(mp_->resolution_, mp_->map_origin_);
}

void SDFMap::resetBuffer() {
  resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
  md_->local_bound_min_ = Eigen::Vector3i::Zero();
  md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos) {
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_->distance_buffer_[toAddress(x, y, z)] = mp_->default_dist_;
      }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_->map_voxel_num_(dim)];
  double z[mp_->map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = md_->local_bound_min_;
  Eigen::Vector3i max_esdf = md_->local_bound_max_;

  if (mp_->optimistic_) {
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
  } else {
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              int adr = toAddress(x, y, z);
              return (md_->occupancy_buffer_inflate_[adr] == 1 ||
                      md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3) ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF(
          [&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
          [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
          max_esdf[1], 1);
    }
  for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF(
          [&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
          [&](int x, double val) {
            md_->distance_buffer_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
          },
          min_esdf[0], max_esdf[0], 0);
    }

  if (mp_->signed_dist_) {
    // Compute negative distance
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return md_->occupancy_buffer_inflate_
                          [x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) +
                           y * mp_->map_voxel_num_(2) + z] == 0 ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF(
            [&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
            [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
            max_esdf[1], 1);
      }
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF(
            [&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
            [&](int x, double val) {
              md_->distance_buffer_neg_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
            },
            min_esdf[0], max_esdf[0], 0);
      }
    // Merge negative distance with positive
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
      for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
          int idx = toAddress(x, y, z);
          if (md_->distance_buffer_neg_[idx] > 0.0)
            md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
        }
  }
}

void SDFMap::setCacheOccupancy(const int& adr, const int& occ) {
  // Add to update list if first visited
  if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0) md_->cache_voxel_.push(adr);

  if (occ == 0)
    md_->count_miss_[adr] = 1;
  else if (occ == 1)
    md_->count_hit_[adr] += 1;

  // md_->count_hit_and_miss_[adr] += 1;
  // if (occ == 1)
  //   md_->count_hit_[adr] += 1;
  // if (md_->count_hit_and_miss_[adr] == 1)
  //   md_->cache_voxel_.push(adr);
}

void SDFMap::inputPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
    const Eigen::Vector3d& camera_pos) {
  if (point_num == 0) return;
  md_->raycast_num_ += 1;

  Eigen::Vector3d update_min = camera_pos;
  Eigen::Vector3d update_max = camera_pos;
  if (md_->reset_updated_box_) {
    md_->update_min_ = camera_pos;
    md_->update_max_ = camera_pos;
    md_->reset_updated_box_ = false;
  }

  Eigen::Vector3d pt_w, tmp;
  Eigen::Vector3i idx;
  int vox_adr;
  double length;
  for (int i = 0; i < point_num; ++i) {
    auto& pt = points.points[i];
    pt_w << pt.x, pt.y, pt.z;
    int tmp_flag;
    // Set flag for projected point
    if (!isInMap(pt_w)) {
      // Find closest point in map and set free
      pt_w = closetPointInMap(pt_w, camera_pos);
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_)
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
      if (pt_w[2] < 0.2) continue;
      tmp_flag = 0;
    } else {
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_) {
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
        if (pt_w[2] < 0.2) continue;
        tmp_flag = 0;
      } else
        tmp_flag = 1;
    }
    posToIndex(pt_w, idx);
    vox_adr = toAddress(idx);
    setCacheOccupancy(vox_adr, tmp_flag);

    for (int k = 0; k < 3; ++k) {
      update_min[k] = min(update_min[k], pt_w[k]);
      update_max[k] = max(update_max[k], pt_w[k]);
    }
    // Raycasting between camera center and point
    if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
      continue;
    else
      md_->flag_rayend_[vox_adr] = md_->raycast_num_;

    caster_->input(pt_w, camera_pos);
    caster_->nextId(idx);
    while (caster_->nextId(idx))
      setCacheOccupancy(toAddress(idx), 0);
  }

  Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
  posToIndex(update_max + bound_inf, md_->local_bound_max_);
  posToIndex(update_min - bound_inf, md_->local_bound_min_);
  boundIndex(md_->local_bound_min_);
  boundIndex(md_->local_bound_max_);
  mr_->local_updated_ = true;

  // Bounding box for subsequent updating
  for (int k = 0; k < 3; ++k) {
    md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
    md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);
  }

  while (!md_->cache_voxel_.empty()) {
    int adr = md_->cache_voxel_.front();
    md_->cache_voxel_.pop();
    double log_odds_update =
        md_->count_hit_[adr] >= md_->count_miss_[adr] ? mp_->prob_hit_log_ : mp_->prob_miss_log_;
    md_->count_hit_[adr] = md_->count_miss_[adr] = 0;
    if (md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
      md_->occupancy_buffer_[adr] = mp_->min_occupancy_log_;

    md_->occupancy_buffer_[adr] = std::min(
        std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_),
        mp_->clamp_max_log_);
  }
}

Eigen::Vector3d
SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
  double min_t = 1000000;
  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }
  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap(bool apply_ceiling) {
  // /*clear outside local*/
  // const int vec_margin = 5;

  // Eigen::Vector3i min_cut = md_->local_bound_min_ -
  //     Eigen::Vector3i(mp_->local_map_margin_, mp_->local_map_margin_,
  //     mp_->local_map_margin_);
  // Eigen::Vector3i max_cut = md_->local_bound_max_ +
  //     Eigen::Vector3i(mp_->local_map_margin_, mp_->local_map_margin_,
  //     mp_->local_map_margin_);
  // boundIndex(min_cut);
  // boundIndex(max_cut);

  // Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin,
  // vec_margin); Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); boundIndex(min_cut_m); boundIndex(max_cut_m);

  // // clear data outside the local range

  // for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
  //   for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

  //     for (int z = min_cut_m(2); z < min_cut(2); ++z) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }

  //     for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }
  //   }

  // for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
  //   for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

  //     for (int y = min_cut_m(1); y < min_cut(1); ++y) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }

  //     for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }
  //   }

  // for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
  //   for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

  //     for (int x = min_cut_m(0); x < min_cut(0); ++x) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }

  //     for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }
  //   }

  // update inflated occupied cells
  // clean outdated occupancy

  int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);

  for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
    for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
      for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
        md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate newest occpuied cells
  for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
    for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
      for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
        int id1 = toAddress(x, y, z);
        if (md_->occupancy_buffer_[id1] > mp_->min_occupancy_log_) {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (auto inf_pt : inf_pts) {
            int idx_inf = toAddress(inf_pt);
            if (idx_inf >= 0 &&
                idx_inf <
                    mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)) {
              md_->occupancy_buffer_inflate_[idx_inf] = 1;
            }
          }
        }
      }

  // add virtual ceiling to limit flight height
  // Skipped when only the inflated grid is being rebuilt from a restored map:
  // the saved occupancy buffer already carries the ceiling where the previous
  // flight observed it, and stamping it over the whole map would mark the
  // ceiling of never-visited columns as occupied.
  if (apply_ceiling && mp_->virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_->virtual_ceil_height_ - mp_->map_origin_(2)) * mp_->resolution_inv_);
    for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
      for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y) {
        // md_->occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
        md_->occupancy_buffer_[toAddress(x, y, ceil_id)] = mp_->clamp_max_log_;
      }
  }
}

double SDFMap::getResolution() {
  return mp_->resolution_;
}

int SDFMap::getVoxelNum() {
  return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  ori = mp_->map_origin_, size = mp_->map_size_;
}

void SDFMap::getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) {
  bmin = mp_->box_mind_;
  bmax = mp_->box_maxd_;
}

void SDFMap::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset) {
  bmin = md_->update_min_;
  bmax = md_->update_max_;
  if (reset) md_->reset_updated_box_ = true;
}

double SDFMap::getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_->resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * mp_->resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_->resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= mp_->resolution_inv_;

  return dist;
}

int SDFMap::knownVoxelCount() const {
  int known = 0, occupied = 0;
  voxelStats(known, occupied);
  return known;
}

void SDFMap::voxelStats(int& known, int& occupied) const {
  known = 0;
  occupied = 0;
  const double unknown = mp_->clamp_min_log_ - mp_->unknown_flag_;
  for (size_t i = 0; i < md_->occupancy_buffer_.size(); ++i) {
    const double occ = md_->occupancy_buffer_[i];
    if (occ <= unknown + 1e-6) continue;
    ++known;
    if (occ > mp_->min_occupancy_log_) ++occupied;
  }
}

void SDFMap::resetMap() {
  const double unknown = mp_->clamp_min_log_ - mp_->unknown_flag_;
  std::fill(md_->occupancy_buffer_.begin(), md_->occupancy_buffer_.end(), unknown);
  std::fill(md_->occupancy_buffer_inflate_.begin(), md_->occupancy_buffer_inflate_.end(), 0);
  std::fill(md_->distance_buffer_.begin(), md_->distance_buffer_.end(), mp_->default_dist_);
  std::fill(md_->distance_buffer_neg_.begin(), md_->distance_buffer_neg_.end(),
            mp_->default_dist_);
  std::fill(md_->count_hit_.begin(), md_->count_hit_.end(), 0);
  std::fill(md_->count_miss_.begin(), md_->count_miss_.end(), 0);
  std::fill(md_->count_hit_and_miss_.begin(), md_->count_hit_and_miss_.end(), 0);
  std::fill(md_->flag_rayend_.begin(), md_->flag_rayend_.end(), -1);
  std::fill(md_->flag_visited_.begin(), md_->flag_visited_.end(), -1);
  md_->cache_voxel_ = std::queue<int>();
  md_->raycast_num_ = 0;
  md_->local_bound_min_ = Eigen::Vector3i::Zero();
  md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
  md_->update_min_ = md_->update_max_ = mp_->map_origin_;
  md_->reset_updated_box_ = true;
  ROS_INFO("SDFMap: in-memory occupancy grid cleared");
}

bool SDFMap::saveMap(const std::string& filename) {
  if (filename.empty()) {
    ROS_WARN("SDFMap: map save path is empty, nothing written");
    return false;
  }
  if (md_->occupancy_buffer_.empty()) {
    ROS_WARN("SDFMap: occupancy buffer is not initialized, nothing written");
    return false;
  }
  // Keep an existing prior rather than replacing it with an untouched grid:
  // the operator may have taken off, aborted, and landed without exploring.
  const int known = knownVoxelCount();
  if (known <= 0) {
    ROS_WARN("SDFMap: map has no observed voxels, keeping the existing prior at %s",
             filename.c_str());
    return false;
  }

  const std::string dir = parentDirectory(filename);
  if (!dir.empty()) ensureDirectory(dir);

  std::ofstream file(filename, std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    ROS_ERROR("SDFMap: cannot open %s for writing: %s", filename.c_str(), strerror(errno));
    return false;
  }

  const int dims[3] = {mp_->map_voxel_num_(0), mp_->map_voxel_num_(1), mp_->map_voxel_num_(2)};
  const double origin[3] = {mp_->map_origin_(0), mp_->map_origin_(1), mp_->map_origin_(2)};
  const double resolution = mp_->resolution_;
  const double inflation = mp_->obstacles_inflation_;
  const size_t voxels = md_->occupancy_buffer_.size();

  file.write(kGridMagic, sizeof(kGridMagic));
  file.write(reinterpret_cast<const char*>(&kGridVersion), sizeof(kGridVersion));
  file.write(reinterpret_cast<const char*>(dims), sizeof(dims));
  file.write(reinterpret_cast<const char*>(origin), sizeof(origin));
  file.write(reinterpret_cast<const char*>(&resolution), sizeof(resolution));
  file.write(reinterpret_cast<const char*>(&inflation), sizeof(inflation));
  file.write(reinterpret_cast<const char*>(&voxels), sizeof(voxels));
  file.write(reinterpret_cast<const char*>(md_->occupancy_buffer_.data()),
             static_cast<std::streamsize>(voxels * sizeof(double)));
  file.flush();
  const bool ok = file.good();
  file.close();
  if (!ok) {
    ROS_ERROR("SDFMap: write to %s failed", filename.c_str());
    return false;
  }

  ROS_INFO("SDFMap: saved occupancy grid to %s (%zu voxels, %d observed)", filename.c_str(),
           voxels, known);
  return true;
}

bool SDFMap::loadMap(const std::string& filename) {
  if (filename.empty()) return false;

  std::ifstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    ROS_WARN("SDFMap: no prior map at %s, starting with an empty map", filename.c_str());
    return false;
  }

  char magic[sizeof(kGridMagic)];
  int version = 0;
  int dims[3] = {0, 0, 0};
  double origin[3] = {0.0, 0.0, 0.0};
  double resolution = 0.0;
  double inflation = 0.0;
  size_t voxels = 0;

  file.read(magic, sizeof(magic));
  file.read(reinterpret_cast<char*>(&version), sizeof(version));
  file.read(reinterpret_cast<char*>(dims), sizeof(dims));
  file.read(reinterpret_cast<char*>(origin), sizeof(origin));
  file.read(reinterpret_cast<char*>(&resolution), sizeof(resolution));
  file.read(reinterpret_cast<char*>(&inflation), sizeof(inflation));
  file.read(reinterpret_cast<char*>(&voxels), sizeof(voxels));

  if (!file.good() || memcmp(magic, kGridMagic, sizeof(magic)) != 0) {
    ROS_ERROR("SDFMap: %s is not a saved occupancy grid, ignoring it", filename.c_str());
    return false;
  }
  if (version != kGridVersion) {
    ROS_ERROR("SDFMap: %s uses grid format v%d, this build writes v%d; ignoring it",
              filename.c_str(), version, kGridVersion);
    return false;
  }

  const size_t expected =
      static_cast<size_t>(mp_->map_voxel_num_(0)) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
  if (dims[0] != mp_->map_voxel_num_(0) || dims[1] != mp_->map_voxel_num_(1) ||
      dims[2] != mp_->map_voxel_num_(2) || voxels != expected ||
      md_->occupancy_buffer_.size() != expected) {
    ROS_ERROR("SDFMap: prior map dims %dx%dx%d (%zu voxels) do not match current %dx%dx%d "
              "(%zu voxels); ignoring %s",
              dims[0], dims[1], dims[2], voxels, mp_->map_voxel_num_(0), mp_->map_voxel_num_(1),
              mp_->map_voxel_num_(2), expected, filename.c_str());
    return false;
  }
  if (std::fabs(resolution - mp_->resolution_) > 1e-6) {
    ROS_ERROR("SDFMap: prior map resolution %.3f does not match current %.3f; ignoring %s",
              resolution, mp_->resolution_, filename.c_str());
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(origin[i] - mp_->map_origin_(i)) > 1e-3) {
      ROS_ERROR("SDFMap: prior map origin [%.2f %.2f %.2f] does not match current "
                "[%.2f %.2f %.2f]; ignoring %s",
                origin[0], origin[1], origin[2], mp_->map_origin_(0), mp_->map_origin_(1),
                mp_->map_origin_(2), filename.c_str());
      return false;
    }
  }

  file.read(reinterpret_cast<char*>(md_->occupancy_buffer_.data()),
            static_cast<std::streamsize>(expected * sizeof(double)));
  if (!file.good()) {
    ROS_ERROR("SDFMap: %s is truncated, ignoring it", filename.c_str());
    resetBuffer();
    return false;
  }
  file.close();

  // The inflated grid is a pure function of the occupancy grid plus the current
  // inflation radius, so it is rebuilt here instead of being trusted from disk.
  // That keeps a prior recorded with a different inflation setting usable.
  md_->local_bound_min_ = Eigen::Vector3i::Zero();
  md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
  rebuildInflateFromOccupancy();

  md_->update_min_ = mp_->map_origin_;
  md_->update_max_ = mp_->map_max_boundary_;
  md_->reset_updated_box_ = false;
  updateESDF3d();

  ROS_INFO("SDFMap: loaded occupancy grid from %s (%zu voxels, %d observed, saved inflation "
           "%.2f m, active %.2f m)",
           filename.c_str(), expected, knownVoxelCount(), inflation, mp_->obstacles_inflation_);
  return true;
}

void SDFMap::rebuildInflateFromOccupancy() {
  // clearAndInflateLocalMap() rebuilds md_->occupancy_buffer_inflate_ over the
  // current local bounds, so pointing those at the whole map restores the
  // inflated grid for every loaded voxel. The virtual ceiling is not restamped:
  // it is already part of the restored occupancy grid.
  clearAndInflateLocalMap(false);
}
}  // namespace fast_planner
// SDFMap
