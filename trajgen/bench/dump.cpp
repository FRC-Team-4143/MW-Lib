// Dumps TrajoptLib solutions as JSON for issue plots.
#include <chrono>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

#include <trajopt/swerve_trajectory_generator.hpp>

static trajopt::SwerveDrivetrain mainbot() {
  const double gearing = 6.5, m = 0.2794;
  return {.mass = 68.0388555,
          .moi = 6.0,
          .wheel_radius = 0.0508,
          .wheel_max_angular_velocity = 628.3185307179587 / gearing,
          .wheel_max_torque = 1.0 * gearing,
          .wheel_cof = 1.5,
          .modules = {{+m, +m}, {+m, -m}, {-m, +m}, {-m, -m}}};
}

static void dump(FILE* f, const char* name, trajopt::SwervePathBuilder& path,
                 bool first) {
  trajopt::SwerveTrajectoryGenerator gen{path};
  auto t0 = std::chrono::steady_clock::now();
  auto sol = gen.generate(false);
  double ms = std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - t0)
                  .count();
  std::fprintf(f, "%s\"%s\":{\"ms\":%.1f,\"ok\":%s", first ? "" : ",", name,
               ms, sol ? "true" : "false");
  if (sol) {
    auto arr = [&](const char* k, const std::vector<double>& v) {
      std::fprintf(f, ",\"%s\":[", k);
      for (size_t i = 0; i < v.size(); ++i)
        std::fprintf(f, "%s%.5f", i ? "," : "", v[i]);
      std::fprintf(f, "]");
    };
    arr("dt", sol->dt);
    arr("x", sol->x);
    arr("y", sol->y);
    arr("c", sol->thetacos);
    arr("s", sol->thetasin);
    arr("vx", sol->vx);
    arr("vy", sol->vy);
    arr("w", sol->omega);
    arr("ax", sol->ax);
    arr("ay", sol->ay);
    std::fprintf(f, ",\"fx\":[");
    for (size_t i = 0; i < sol->module_fx.size(); ++i) {
      std::fprintf(f, "%s", i ? "," : "");
      std::fprintf(f, "[");
      for (size_t j = 0; j < sol->module_fx[i].size(); ++j)
        std::fprintf(f, "%s%.3f", j ? "," : "", sol->module_fx[i][j]);
      std::fprintf(f, "]");
    }
    std::fprintf(f, "],\"fy\":[");
    for (size_t i = 0; i < sol->module_fy.size(); ++i) {
      std::fprintf(f, "%s", i ? "," : "");
      std::fprintf(f, "[");
      for (size_t j = 0; j < sol->module_fy[i].size(); ++j)
        std::fprintf(f, "%s%.3f", j ? "," : "", sol->module_fy[i][j]);
      std::fprintf(f, "]");
    }
    std::fprintf(f, "]");
  }
  std::fprintf(f, "}");
}

static trajopt::SwervePathBuilder base() {
  trajopt::SwervePathBuilder p;
  p.set_drivetrain(mainbot());
  p.set_bumpers(0.4191, 0.4572, 0.4572, 0.4191);
  return p;
}

int main(int argc, char** argv) {
  constexpr double pi = std::numbers::pi;
  FILE* f = std::fopen(argv[1], "w");
  std::fprintf(f, "{");
  {
    auto p = base();
    p.pose_wpt(0, {2.0, 2.0, 0.0});
    p.pose_wpt(1, {9.0, 3.0, pi / 2});
    std::vector<trajopt::Pose2d> g{{4.0, 1.0, 0.5}, {7.0, 1.7, 1.0}};
    p.sgmt_initial_guess_points(0, g);
    p.sgmt_keep_out_circle(0, 1, {4.0, 2.2}, 0.6);
    p.sgmt_keep_out_circle(0, 1, {7.0, 2.8}, 0.6);
    p.wpt_linear_velocity_max_magnitude(0, 0.0);
    p.wpt_linear_velocity_max_magnitude(1, 0.0);
    p.set_control_interval_counts({60});
    dump(f, "obstacles", p, true);
  }
  {
    auto p = base();
    p.pose_wpt(0, {2.0, 2.0, 0.0});
    p.pose_wpt(1, {6.0, 4.0, pi});
    p.wpt_linear_velocity_max_magnitude(0, 0.0);
    p.wpt_linear_velocity_max_magnitude(1, 0.0);
    p.set_control_interval_counts({60});
    dump(f, "medium", p, false);
  }
  {
    auto p = base();
    p.pose_wpt(0, {2.0, 2.0, 0.0});
    p.pose_wpt(1, {5.0, 4.0, pi / 2});
    p.wpt_linear_velocity_direction(0, -pi / 2);
    p.wpt_linear_velocity_max_magnitude(0, 3.0);
    p.wpt_linear_velocity_max_magnitude(1, 0.0);
    p.set_control_interval_counts({60});
    dump(f, "moving", p, false);
  }
  std::fprintf(f, "}\n");
  std::fclose(f);
}
