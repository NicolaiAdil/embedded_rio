// Offline replay of SD-card sensor logs through rio::RioEskf.
//
// Edit makeParams() / P0_diag below to retune sigmas and initial covariance,
// then rebuild and run:
//
//   cmake -S tools/replay -B build/replay -DCMAKE_BUILD_TYPE=Release
//   cmake --build build/replay -j
//   ./build/replay/rio_replay path/to/LOG0001.CSV out_dir/
//
// Outputs (in out_dir/):
//   state.csv         — t,source,p_WI,v_WI,q_WI,b_a,b_g,p_IR,q_IR
//   cov_diag.csv      — t,source,P00..P20
//   radar_innov.csv   — t,point_idx,status,residual,S,norm  (long format)
//   baro_innov.csv    — t,initialized,accepted,rejected,skipped,dz_meas,
//                       dz_pred,residual,S,norm

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include <rio/rio_eskf.h>

#include "log_reader.h"

namespace fs = std::filesystem;

// ── Tunable parameters ────────────────────────────────────────────────────────
// Mirrors src/main.cpp:24-64 verbatim. Edit these to try alternate tunings.

static rio::Params makeParams() {
  rio::Params p;

  p.g_W = rio::Vec3(0.0f, 0.0f, -9.80665f);

  p.sigma_acc = 2.2563e-3f;
  p.sigma_ba  = 2.2563e-4f;

  p.sigma_gyr = 2.443461e-4f;
  p.sigma_bg  = 2.443461e-5f;

  p.tau_ba = 700.0f;
  p.tau_bg = 450.0f;

  p.min_dt = 1e-6f;
  p.max_dt = 0.1f;

  p.q_IR = rio::Quat(0.68301f, -0.18301f, -0.18301f, 0.68301f);
  p.p_IR = rio::Vec3(-0.065f, 0.025013f, 0.020f);

  p.sigma_vr      = 0.038f;
  p.gating_enable = true;
  p.gate_nsigma   = 5.0f;
  p.vr_sign       = 1.0f;

  p.sigma_baro_dz      = 0.3f;
  p.baro_gating_enable = true;
  p.baro_gate_nsigma   = 5.0f;
  p.baro_z_sign        = 1.0f;
  return p;
}

static constexpr float P0_diag[21] = {
  1e-6f  , 1e-6f  , 1e-6f  ,
  1e-1f  , 1e-1f  , 1e-1f  ,
  1e-2f  , 1e-2f  , 1e-2f  ,
  1.1e-3f, 1.1e-3f, 1e-8f  ,
  1e-4f  , 1e-4f  , 1e-4f  ,
  2.0e-5f, 2.0e-5f, 2.0e-5f,
  1.0e-2f, 1.0e-2f, 1.0e-2f,
};

// ── Output writers ────────────────────────────────────────────────────────────

namespace {

struct Outputs {
  std::FILE* state{};
  std::FILE* cov{};
  std::FILE* radar{};
  std::FILE* baro{};
};

void writeStateHeader(std::FILE* f) {
  std::fprintf(f,
    "t,source,"
    "p_x,p_y,p_z,v_x,v_y,v_z,"
    "q_w,q_x,q_y,q_z,"
    "ba_x,ba_y,ba_z,bg_x,bg_y,bg_z,"
    "p_IR_x,p_IR_y,p_IR_z,q_IR_w,q_IR_x,q_IR_y,q_IR_z\n");
}

void writeCovHeader(std::FILE* f) {
  std::fprintf(f, "t,source");
  for (int i = 0; i < 21; ++i) std::fprintf(f, ",P%02d", i);
  std::fprintf(f, "\n");
}

void writeRadarHeader(std::FILE* f) {
  std::fprintf(f, "t,point_idx,status,residual,S,norm\n");
}

void writeBaroHeader(std::FILE* f) {
  std::fprintf(f,
    "t,initialized,accepted,rejected,skipped,"
    "dz_meas,dz_pred,residual,S,norm\n");
}

void writeStateRow(std::FILE* f, float t, const char* src,
                   const rio::NominalState& x) {
  std::fprintf(f,
    "%.6f,%s,"
    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
    "%.6f,%.6f,%.6f,%.6f,"
    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
    t, src,
    x.p_WI.x(), x.p_WI.y(), x.p_WI.z(),
    x.v_WI.x(), x.v_WI.y(), x.v_WI.z(),
    x.q_WI.w(), x.q_WI.x(), x.q_WI.y(), x.q_WI.z(),
    x.b_a.x(),  x.b_a.y(),  x.b_a.z(),
    x.b_g.x(),  x.b_g.y(),  x.b_g.z(),
    x.p_IR.x(), x.p_IR.y(), x.p_IR.z(),
    x.q_IR.w(), x.q_IR.x(), x.q_IR.y(), x.q_IR.z());
}

void writeCovRow(std::FILE* f, float t, const char* src, const rio::Mat21& P) {
  std::fprintf(f, "%.6f,%s", t, src);
  for (int i = 0; i < 21; ++i) std::fprintf(f, ",%.9g", P(i, i));
  std::fprintf(f, "\n");
}

void writeRadarInnov(std::FILE* f, float t, const rio::LastInnovations& inn) {
  for (size_t i = 0; i < inn.radar_n_recorded; ++i) {
    const float r    = inn.radar_residual[i];
    const float S    = inn.radar_S[i];
    const float norm = (S > 0.0f && std::isfinite(r) && std::isfinite(S))
                           ? std::fabs(r) / std::sqrt(S)
                           : NAN;
    std::fprintf(f, "%.6f,%zu,%u,%.6g,%.6g,%.6g\n",
                 t, i, static_cast<unsigned>(inn.radar_status[i]), r, S, norm);
  }
}

void writeBaroInnov(std::FILE* f, float t, const rio::LastInnovations& inn) {
  const float r    = inn.baro_residual;
  const float S    = inn.baro_S;
  const float norm = (S > 0.0f && std::isfinite(r) && std::isfinite(S))
                         ? std::fabs(r) / std::sqrt(S)
                         : NAN;
  std::fprintf(f, "%.6f,%d,%d,%d,%d,%.6g,%.6g,%.6g,%.6g,%.6g\n",
               t,
               inn.baro_initialized ? 1 : 0,
               inn.baro_accepted    ? 1 : 0,
               inn.baro_rejected    ? 1 : 0,
               inn.baro_skipped     ? 1 : 0,
               inn.baro_dz_meas, inn.baro_dz_pred, r, S, norm);
}

}  // namespace

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <log.csv> <out_dir>\n", argv[0]);
    return 2;
  }
  const std::string log_path = argv[1];
  const fs::path    out_dir  = argv[2];

  std::error_code ec;
  fs::create_directories(out_dir, ec);
  if (ec) {
    std::fprintf(stderr, "cannot create %s: %s\n",
                 out_dir.string().c_str(), ec.message().c_str());
    return 2;
  }

  Outputs out;
  out.state = std::fopen((out_dir / "state.csv").string().c_str(), "w");
  out.cov   = std::fopen((out_dir / "cov_diag.csv").string().c_str(), "w");
  out.radar = std::fopen((out_dir / "radar_innov.csv").string().c_str(), "w");
  out.baro  = std::fopen((out_dir / "baro_innov.csv").string().c_str(), "w");
  if (!out.state || !out.cov || !out.radar || !out.baro) {
    std::fprintf(stderr, "failed to open output files in %s\n",
                 out_dir.string().c_str());
    return 2;
  }
  writeStateHeader(out.state);
  writeCovHeader(out.cov);
  writeRadarHeader(out.radar);
  writeBaroHeader(out.baro);

  rio::RioEskf  eskf;
  rio::Params   params = makeParams();
  eskf.setParams(params);

  rio::NominalState x0;
  x0.p_IR = params.p_IR;
  x0.q_IR = params.q_IR;
  eskf.reset(x0, P0_diag, 0.0f);

  bool             time_init = false;
  bool             att_init  = false;
  float            last_t    = 0.0f;
  rio::ImuSample   last_imu{};

  // Counters for end-of-run summary.
  size_t n_imu = 0, n_radar_frames = 0, n_radar_pts = 0, n_baro = 0;
  size_t n_radar_accepted = 0, n_radar_rejected = 0, n_radar_skipped = 0;
  size_t n_baro_accepted = 0, n_baro_rejected = 0, n_baro_skipped = 0;

  replay::LogReader reader(log_path);
  replay::ReplayEvent ev;

  while (reader.nextEvent(ev)) {
    switch (ev.kind) {
      case replay::EventKind::IMU: {
        ++n_imu;

        if (!time_init) {
          last_t    = ev.t_s;
          time_init = true;
          break;
        }
        if (!att_init) {
          if (eskf.initAttitudeFromGravity(ev.imu.acc, P0_diag, ev.t_s)) {
            att_init = true;
            last_t   = ev.t_s;
            last_imu = ev.imu;
            writeStateRow(out.state, ev.t_s, "INIT", eskf.getState());
            writeCovRow  (out.cov,   ev.t_s, "INIT", eskf.getCovariance());
          }
          break;
        }

        const float dt = ev.t_s - last_t;
        last_t   = ev.t_s;
        last_imu = ev.imu;

        if (dt >= params.min_dt && dt <= params.max_dt) {
          eskf.predict       (ev.imu, dt);
          eskf.insPropagation(ev.imu, dt);
        }
        writeStateRow(out.state, ev.t_s, "IMU", eskf.getState());
        writeCovRow  (out.cov,   ev.t_s, "IMU", eskf.getCovariance());
        break;
      }

      case replay::EventKind::RADAR_FRAME: {
        ++n_radar_frames;
        n_radar_pts += ev.radar.size();
        if (!att_init || ev.radar.empty()) break;

        // Mirror processRadar: fill .sigma from current params.
        for (auto& d : ev.radar) d.sigma = params.sigma_vr;

        const auto res = eskf.correct(ev.radar.data(), ev.radar.size(), last_imu);
        n_radar_accepted += res.n_accepted;
        n_radar_rejected += res.n_rejected;
        n_radar_skipped  += res.n_skipped;

        writeRadarInnov(out.radar, ev.t_s, eskf.getLastInnovations());
        writeStateRow  (out.state, ev.t_s, "RAD",  eskf.getState());
        writeCovRow    (out.cov,   ev.t_s, "RAD",  eskf.getCovariance());
        break;
      }

      case replay::EventKind::BARO: {
        ++n_baro;
        if (!att_init) break;

        const auto br = eskf.correctBarometer(ev.baro);
        if (br.accepted) ++n_baro_accepted;
        if (br.rejected) ++n_baro_rejected;
        if (br.skipped)  ++n_baro_skipped;

        writeBaroInnov(out.baro,  ev.t_s, eskf.getLastInnovations());
        writeStateRow (out.state, ev.t_s, "BAR", eskf.getState());
        writeCovRow   (out.cov,   ev.t_s, "BAR", eskf.getCovariance());
        break;
      }
    }
  }

  std::fclose(out.state);
  std::fclose(out.cov);
  std::fclose(out.radar);
  std::fclose(out.baro);

  std::fprintf(stderr,
    "replay complete: imu=%zu radar_frames=%zu radar_pts=%zu baro=%zu\n"
    "  radar: %zu accepted, %zu rejected, %zu skipped\n"
    "  baro:  %zu accepted, %zu rejected, %zu skipped\n",
    n_imu, n_radar_frames, n_radar_pts, n_baro,
    n_radar_accepted, n_radar_rejected, n_radar_skipped,
    n_baro_accepted,  n_baro_rejected,  n_baro_skipped);
  return 0;
}
