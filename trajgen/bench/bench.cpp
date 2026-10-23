// On-the-fly TrajoptLib solve-time benchmark using mainbot's choreo_config.chor.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <numbers>
#include <string>
#include <vector>

#include <trajopt/swerve_trajectory_generator.hpp>

using Clock = std::chrono::steady_clock;

static trajopt::SwerveDrivetrain mainbot() {
  const double gearing = 6.5;
  const double m = 0.2794;  // 11 in
  return {.mass = 68.0388555,
          .moi = 6.0,
          .wheel_radius = 0.0508,
          .wheel_max_angular_velocity = 628.3185307179587 / gearing,
          .wheel_max_torque = 1.0 * gearing,
          .wheel_cof = 1.5,
          .modules = {{+m, +m}, {+m, -m}, {-m, +m}, {-m, -m}}};
}

struct Case {
  std::string name;
  std::function<void(trajopt::SwervePathBuilder&)> setup;
};

int main(int argc, char** argv) {
  const int reps = argc > 1 ? std::atoi(argv[1]) : 5;
  constexpr double pi = std::numbers::pi;

  std::vector<Case> cases{
      {"short 1.5m +90deg",
       [&](auto& p) {
         p.pose_wpt(0, {2.0, 2.0, 0.0});
         p.pose_wpt(1, {3.5, 2.0, pi / 2});
       }},
      {"medium 4m diag",
       [&](auto& p) {
         p.pose_wpt(0, {2.0, 2.0, 0.0});
         p.pose_wpt(1, {5.0, 4.6, pi});
       }},
      {"long 9m",
       [&](auto& p) {
         p.pose_wpt(0, {2.0, 1.0, 0.0});
         p.pose_wpt(1, {11.0, 3.0, -pi / 2});
       }},
      {"4m + 1 keep-out",
       [&](auto& p) {
         p.pose_wpt(0, {2.0, 2.0, 0.0});
         p.pose_wpt(1, {6.0, 2.0, 0.0});
         p.sgmt_keep_out_circle(0, 1, {4.0, 2.1}, 0.5);
       }},
      {"7m + 2 keep-outs",
       [&](auto& p) {
         p.pose_wpt(0, {2.0, 2.0, 0.0});
         p.pose_wpt(1, {9.0, 3.0, pi / 2});
         p.sgmt_keep_out_circle(0, 1, {4.0, 2.2}, 0.6);
         p.sgmt_keep_out_circle(0, 1, {7.0, 2.8}, 0.6);
       }},
      {"4m + 1 keep-out +guess",
       [&](auto& p) {
         p.pose_wpt(0, {2.0, 2.0, 0.0});
         p.pose_wpt(1, {6.0, 2.0, 0.0});
         std::vector<trajopt::Pose2d> g{{4.0, 3.2, 0.0}};
         p.sgmt_initial_guess_points(0, g);
         p.sgmt_keep_out_circle(0, 1, {4.0, 2.1}, 0.5);
       }},
      {"7m + 2 keep-outs +guess",
       [&](auto& p) {
         p.pose_wpt(0, {2.0, 2.0, 0.0});
         p.pose_wpt(1, {9.0, 3.0, pi / 2});
         std::vector<trajopt::Pose2d> g{{4.0, 1.0, 0.5}, {7.0, 1.7, 1.0}};
         p.sgmt_initial_guess_points(0, g);
         p.sgmt_keep_out_circle(0, 1, {4.0, 2.2}, 0.6);
         p.sgmt_keep_out_circle(0, 1, {7.0, 2.8}, 0.6);
       }},
      {"moving start 3m/s",
       [&](auto& p) {
         p.pose_wpt(0, {2.0, 2.0, 0.0});
         p.pose_wpt(1, {5.0, 4.0, pi / 2});
         p.wpt_linear_velocity_direction(0, -pi / 2);  // moving away from goal
         p.wpt_linear_velocity_max_magnitude(0, 3.0);
       }},
  };

  std::printf("%-20s %4s %10s %10s %10s %9s %s\n", "case", "N", "build_ms",
              "solve_med", "solve_max", "traj_s", "status");
  for (const auto& c : cases) {
    for (size_t N : {20, 40, 80}) {
      std::vector<double> solve_ms;
      double build_ms = 0.0;
      double traj_s = 0.0;
      std::string status = "ok";
      for (int r = 0; r < reps; ++r) {
        auto t0 = Clock::now();
        trajopt::SwervePathBuilder path;
        path.set_drivetrain(mainbot());
        path.set_bumpers(0.4191, 0.4572, 0.4572, 0.4191);
        c.setup(path);
        if (c.name.rfind("moving", 0) != 0) {
          path.wpt_linear_velocity_max_magnitude(0, 0.0);
        }
        path.wpt_linear_velocity_max_magnitude(1, 0.0);
        path.set_control_interval_counts({N});
        trajopt::SwerveTrajectoryGenerator gen{path};
        auto t1 = Clock::now();
        auto sol = gen.generate(false);
        auto t2 = Clock::now();
        build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        solve_ms.push_back(
            std::chrono::duration<double, std::milli>(t2 - t1).count());
        if (!sol) {
          status = "FAIL(" +
                   std::to_string(static_cast<int>(sol.error())) + ")";
        } else {
          traj_s = 0.0;
          for (double dt : sol->dt) traj_s += dt;
        }
      }
      std::sort(solve_ms.begin(), solve_ms.end());
      std::printf("%-20s %4zu %10.1f %10.1f %10.1f %9.3f %s\n",
                  c.name.c_str(), N, build_ms, solve_ms[solve_ms.size() / 2],
                  solve_ms.back(), traj_s, status.c_str());
    }
  }
}
