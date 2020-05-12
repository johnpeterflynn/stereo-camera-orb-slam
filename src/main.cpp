/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include <sophus/se3.hpp>

#include <tbb/concurrent_unordered_map.h>
#include <tbb/tbb.h>

#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/image/image.h>
#include <pangolin/image/image_io.h>
#include <pangolin/image/typed_image.h>
#include <pangolin/pangolin.h>

#include <CLI/CLI.hpp>

#include "common_types.h"

#include "calibration.h"

#include "keyframe.h"
#include "keypoints.h"

#include "local_mapping.h"
#include "tracking.h"

#include "gui_helper.h"

#include "serialization.h"

///////////////////////////////////////////////////////////////////////////////
/// Declarations
///////////////////////////////////////////////////////////////////////////////

void draw_image_overlay(pangolin::View& v, size_t cam_id);
void change_display_to_image(const TimeCamId& tcid);
void draw_scene();
void load_data(const std::string& path, const std::string& calib_path);
bool next_step();
void compute_projections();

///////////////////////////////////////////////////////////////////////////////
/// Constants
///////////////////////////////////////////////////////////////////////////////

constexpr int UI_WIDTH = 200;
constexpr int NUM_CAMS = 2;

///////////////////////////////////////////////////////////////////////////////
/// Variables
///////////////////////////////////////////////////////////////////////////////

int current_frame = 0;

/// intrinsic calibration
Calibration calib_cam;

/// loaded images
tbb::concurrent_unordered_map<TimeCamId, std::string> images;

/// cashed info on reprojected landmarks; recomputed every time time from
/// cameras, landmarks, and feature_tracks; used for visualization and
/// determining outliers; indexed by images
ImageProjections image_projections;

///////////////////////////////////////////////////////////////////////////////
/// GUI parameters
///////////////////////////////////////////////////////////////////////////////

// The following GUI elements can be enabled / disabled from the main panel by
// switching the prefix from "ui" to "hidden" or vice verca. This way you can
// show only the elements you need / want for development.

pangolin::Var<bool> ui_show_hidden("ui.show_extra_options", false, false, true);

//////////////////////////////////////////////
/// Image display options

pangolin::Var<int> show_frame1("ui.show_frame1", 0, 0, 1500);
pangolin::Var<int> show_cam1("ui.show_cam1", 0, 0, NUM_CAMS - 1);
pangolin::Var<int> show_frame2("ui.show_frame2", 0, 0, 1500);
pangolin::Var<int> show_cam2("ui.show_cam2", 1, 0, NUM_CAMS - 1);
pangolin::Var<bool> lock_frames("ui.lock_frames", true, false, true);
pangolin::Var<bool> show_detected("ui.show_detected", true, false, true);
pangolin::Var<bool> show_matches("ui.show_matches", true, false, true);
pangolin::Var<bool> show_inliers("ui.show_inliers", true, false, true);
pangolin::Var<bool> show_reprojections("ui.show_reprojections", true, false,
                                       true);
pangolin::Var<bool> show_ids("ui.show_ids", false, false, true);
pangolin::Var<bool> show_cameras3d("ui.show_cameras", true, false, true);
pangolin::Var<bool> show_points3d("ui.show_points", true, false, true);
pangolin::Var<bool> show_cv_graph("ui.show_cv_graph", false, false, true);
pangolin::Var<bool> show_loop_edges("ui.show_loop_edges", false, false, true);
pangolin::Var<bool> show_spanning_tree("ui.show_spanning_tree", false, false,
                                       true);
pangolin::Var<bool> show_loop_detection_rad("ui.show_loop_detect_rad", false,
                                            false, true);
pangolin::Var<bool> show_unopt_poses("ui.show_unopt_poses", false, false, true);

//////////////////////////////////////////////
/// Feature extraction and matching options

pangolin::Var<int> num_features_per_image("hidden.num_features", 1500, 10,
                                          5000);
pangolin::Var<bool> rotate_features("hidden.rotate_features", true, false,
                                    true);
pangolin::Var<int> feature_match_max_dist("hidden.match_max_dist", 70, 1, 255);
pangolin::Var<double> feature_match_test_next_best("hidden.match_next_best",
                                                   1.2, 1, 4);

pangolin::Var<double> match_max_dist_2d("hidden.match_max_dist_2d", 20.0, 1.0,
                                        50);

pangolin::Var<int> new_kf_min_inliers("hidden.new_kf_min_inliers", 80, 1, 200);

pangolin::Var<double> cam_z_threshold("hidden.cam_z_threshold", 0.1, 1.0, 0.0);

//////////////////////////////////////////////
/// Adding cameras and landmarks options

pangolin::Var<double> reprojection_error_pnp_inlier_threshold_pixel(
    "hidden.pnp_inlier_thresh", 3.0, 0.1, 10);

//////////////////////////////////////////////
/// Loop Closing options

pangolin::Var<float> loop_dist_thresh("hidden.loop_dist_thresh", 0.75, 0.10,
                                      1.0);

pangolin::Var<float> loop_angle_thresh("hidden.loop_angle_thresh", 0.80, 0.10,
                                       1.0);

pangolin::Var<unsigned int> loop_min_inliers("hidden.loop_min_inliers", 15, 5,
                                             100);

//////////////////////////////////////////////
/// Bundle Adjustment Options

pangolin::Var<bool> ba_optimize_intrinsics("hidden.ba_opt_intrinsics", false,
                                           false, true);
pangolin::Var<int> ba_verbose("hidden.ba_verbose", 0, 0, 2);

pangolin::Var<bool> ba_use_huber("hidden.ba_use_huber", true, false, true);

pangolin::Var<double> localBA_huber_parameter("hidden.localBA_huber_parameter",
                                              1.0, 0.1, 10);

pangolin::Var<double> globalBA_huber_parameter(
    "hidden.globalBA_huber_parameter", 1.0, 0.1, 10);

pangolin::Var<int> localBA_max_iterations("hidden.localBA_max_iterations", 10,
                                          10, 50);

pangolin::Var<int> globalBA_max_iterations("hidden.globalBA_max_iterations", 30,
                                           10, 50);

///////////////////////////////////////////////////////////////////////////////
/// GUI buttons
///////////////////////////////////////////////////////////////////////////////

// if you enable this, next_step is called repeatedly until completion
pangolin::Var<bool> continue_next("ui.continue_next", false, false, true);

using Button = pangolin::Var<std::function<void(void)>>;

Button next_step_btn("ui.next_step", &next_step);

// Creating global objects for execution
Settings settings;
Map* map = new Map();
Tracking* tracking = new Tracking(map, calib_cam, settings);
LocalMapping* localMapping = new LocalMapping(map, calib_cam, settings);
LoopClosing* loopClosing = new LoopClosing(map, calib_cam, settings);

///////////////////////////////////////////////////////////////////////////////
/// GUI and Boilerplate Implementation
///////////////////////////////////////////////////////////////////////////////

// Parse parameters, load data, and create GUI window and event loop (or
// process everything in non-gui mode).
int main(int argc, char** argv) {
  bool show_gui = true;
  std::string dataset_path = "data/V1_01_easy/mav0";
  std::string cam_calib = "opt_calib.json";

  CLI::App app{"Visual odometry."};

  app.add_option("--show-gui", show_gui, "Show GUI");
  app.add_option("--dataset-path", dataset_path,
                 "Dataset path. Default: " + dataset_path);
  app.add_option("--cam-calib", cam_calib,
                 "Path to camera calibration. Default: " + cam_calib);
  app.add_option("--start-frame", current_frame,
                 "Start from frame " + current_frame);

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  load_data(dataset_path, cam_calib);

  tracking->setLocalMapper(localMapping);
  tracking->setLoopCloser(loopClosing);

  localMapping->setTracker(tracking);
  localMapping->setLoopCloser(loopClosing);

  loopClosing->setTracker(tracking);
  loopClosing->setLocalMapper(localMapping);

  if (show_gui) {
    pangolin::CreateWindowAndBind("Main", 1800, 1000);

    glEnable(GL_DEPTH_TEST);

    // main parent display for images and 3d viewer
    pangolin::View& main_view =
        pangolin::Display("main")
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
            .SetLayout(pangolin::LayoutEqualVertical);

    // parent display for images
    pangolin::View& img_view_display =
        pangolin::Display("images").SetLayout(pangolin::LayoutEqual);
    main_view.AddDisplay(img_view_display);

    // main ui panel
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));

    // extra options panel
    pangolin::View& hidden_panel = pangolin::CreatePanel("hidden").SetBounds(
        0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH),
        pangolin::Attach::Pix(2 * UI_WIDTH));
    ui_show_hidden.Meta().gui_changed = true;

    // 2D image views
    std::vector<std::shared_ptr<pangolin::ImageView>> img_view;
    while (img_view.size() < NUM_CAMS) {
      std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

      size_t idx = img_view.size();
      img_view.push_back(iv);

      img_view_display.AddDisplay(*iv);
      iv->extern_draw_function =
          std::bind(&draw_image_overlay, std::placeholders::_1, idx);
    }

    // 3D visualization (initial camera view optimized to see full map)
    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 0.001, 10000),
        pangolin::ModelViewLookAt(-3.4, -3.7, -8.3, 2.1, 0.6, 0.2,
                                  pangolin::AxisNegY));

    pangolin::View& display3D =
        pangolin::Display("scene")
            .SetAspect(-640 / 480.0)
            .SetHandler(new pangolin::Handler3D(camera));
    main_view.AddDisplay(display3D);

    while (!pangolin::ShouldQuit()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if (ui_show_hidden.GuiChanged()) {
        hidden_panel.Show(ui_show_hidden);
        const int panel_width = ui_show_hidden ? 2 * UI_WIDTH : UI_WIDTH;
        main_view.SetBounds(0.0, 1.0, pangolin::Attach::Pix(panel_width), 1.0);
      }

      display3D.Activate(camera);
      glClearColor(0.0f, 0.0f, 0.0f, 1.0f);  //black background
      //glClearColor(0.95f, 0.95f, 0.95f, 1.0f);  // light gray background

      draw_scene();

      img_view_display.Activate();

      if (lock_frames) {
        // in case of locking frames, changing one should change the other
        if (show_frame1.GuiChanged()) {
          change_display_to_image(std::make_pair(show_frame1, 0));
          change_display_to_image(std::make_pair(show_frame1, 1));
        } else if (show_frame2.GuiChanged()) {
          change_display_to_image(std::make_pair(show_frame2, 0));
          change_display_to_image(std::make_pair(show_frame2, 1));
        }
      }

      if (show_frame1.GuiChanged() || show_cam1.GuiChanged()) {
        size_t frame_id = show_frame1;
        size_t cam_id = show_cam1;

        TimeCamId tcid;
        tcid.first = frame_id;
        tcid.second = cam_id;
        if (images.find(tcid) != images.end()) {
          pangolin::TypedImage img = pangolin::LoadImage(images[tcid]);
          img_view[0]->SetImage(img);
        } else {
          img_view[0]->Clear();
        }
      }

      if (show_frame2.GuiChanged() || show_cam2.GuiChanged()) {
        size_t frame_id = show_frame2;
        size_t cam_id = show_cam2;

        TimeCamId tcid;
        tcid.first = frame_id;
        tcid.second = cam_id;
        if (images.find(tcid) != images.end()) {
          pangolin::GlPixFormat fmt;
          fmt.glformat = GL_LUMINANCE;
          fmt.gltype = GL_UNSIGNED_BYTE;
          fmt.scalable_internal_format = GL_LUMINANCE8;

          pangolin::TypedImage img = pangolin::LoadImage(images[tcid]);
          img_view[1]->SetImage(img);
        } else {
          img_view[1]->Clear();
        }
      }

      pangolin::FinishFrame();

      if (continue_next) {
        // stop if there is nothing left to do
        continue_next = next_step();
      } else {
        // if the gui is just idling, make sure we don't burn too much CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  } else {
    // non-gui mode: Process all frames, then exit
    while (next_step()) {
      // nop
    }
    std::cout << "EXITING !!!!!!!!!\n";
  }

  return 0;
}

// Visualize features and related info on top of the image views
void draw_image_overlay(pangolin::View& v, size_t view_id) {
  size_t frame_id = view_id == 0 ? show_frame1 : show_frame2;
  size_t cam_id = view_id == 0 ? show_cam1 : show_cam2;

  TimeCamId tcid = std::make_pair(frame_id, cam_id);

  float text_row = 20;

  if (show_detected) {
    glLineWidth(1.0);
    glColor3f(1.0, 0.0, 0.0);  // red
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (map->feature_corners.find(tcid) != map->feature_corners.end()) {
      const Keypoints& cr = map->feature_corners.at(tcid);

      for (size_t i = 0; i < cr.corners.size(); i++) {
        Eigen::Vector2d c = cr.corners[i];
        double angle = cr.corner_angles[i];
        pangolin::glDrawCirclePerimeter(c[0], c[1], 3.0);

        Eigen::Vector2d r(3, 0);
        Eigen::Rotation2Dd rot(angle);
        r = rot * r;

        pangolin::glDrawLine(c, c + r);
      }

      pangolin::GlFont::I()
          .Text("Detected %d corners", cr.corners.size())
          .Draw(5, text_row);

    } else {
      glLineWidth(1.0);

      pangolin::GlFont::I().Text("Corners not processed").Draw(5, text_row);
    }
    text_row += 20;
  }

  if (show_matches || show_inliers) {
    glLineWidth(1.0);
    glColor3f(0.0, 0.0, 1.0);  // blue
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    size_t o_frame_id = (view_id == 0 ? show_frame2 : show_frame1);
    size_t o_cam_id = (view_id == 0 ? show_cam2 : show_cam1);

    TimeCamId o_tcid = std::make_pair(o_frame_id, o_cam_id);

    int idx = -1;

    auto it = map->feature_matches.find(std::make_pair(tcid, o_tcid));

    if (it != map->feature_matches.end()) {
      idx = 0;
    } else {
      it = map->feature_matches.find(std::make_pair(o_tcid, tcid));
      if (it != map->feature_matches.end()) {
        idx = 1;
      }
    }

    if (idx >= 0 && show_matches) {
      if (map->feature_corners.find(tcid) != map->feature_corners.end()) {
        const Keypoints& cr = map->feature_corners.at(tcid);

        for (size_t i = 0; i < it->second.matches.size(); i++) {
          size_t c_idx = idx == 0 ? it->second.matches[i].first
                                  : it->second.matches[i].second;

          Eigen::Vector2d c = cr.corners[c_idx];
          double angle = cr.corner_angles[c_idx];
          pangolin::glDrawCirclePerimeter(c[0], c[1], 3.0);

          Eigen::Vector2d r(3, 0);
          Eigen::Rotation2Dd rot(angle);
          r = rot * r;

          pangolin::glDrawLine(c, c + r);

          if (show_ids) {
            pangolin::GlFont::I().Text("%d", i).Draw(c[0], c[1]);
          }
        }

        pangolin::GlFont::I()
            .Text("Detected %d matches", it->second.matches.size())
            .Draw(5, text_row);
        text_row += 20;
      }
    }

    glColor3f(0.0, 1.0, 0.0);  // green

    if (idx >= 0 && show_inliers) {
      if (map->feature_corners.find(tcid) != map->feature_corners.end()) {
        const Keypoints& cr = map->feature_corners.at(tcid);

        for (size_t i = 0; i < it->second.inliers.size(); i++) {
          size_t c_idx = idx == 0 ? it->second.inliers[i].first
                                  : it->second.inliers[i].second;

          Eigen::Vector2d c = cr.corners[c_idx];
          double angle = cr.corner_angles[c_idx];
          pangolin::glDrawCirclePerimeter(c[0], c[1], 3.0);

          Eigen::Vector2d r(3, 0);
          Eigen::Rotation2Dd rot(angle);
          r = rot * r;

          pangolin::glDrawLine(c, c + r);

          if (show_ids) {
            pangolin::GlFont::I().Text("%d", i).Draw(c[0], c[1]);
          }
        }

        pangolin::GlFont::I()
            .Text("Detected %d inliers", it->second.inliers.size())
            .Draw(5, text_row);
        text_row += 20;
      }
    }
  }

  if (show_reprojections) {
    if (image_projections.count(tcid) > 0) {
      glLineWidth(1.0);
      glColor3f(1.0, 0.0, 0.0);  // red
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      const size_t num_points = image_projections.at(tcid).obs.size();
      double error_sum = 0;
      size_t num_outliers = 0;

      // count up and draw all inlier projections
      for (const auto& lm_proj : image_projections.at(tcid).obs) {
        error_sum += lm_proj->reprojection_error;

        if (lm_proj->outlier_flags != OutlierNone) {
          // outlier point
          glColor3f(1.0, 0.0, 0.0);  // red
          ++num_outliers;
        } else if (lm_proj->reprojection_error > localBA_huber_parameter) {
          // close to outlier point
          glColor3f(1.0, 0.5, 0.0);  // orange
        } else {
          // clear inlier point
          glColor3f(1.0, 1.0, 0.0);  // yellow
        }
        pangolin::glDrawCirclePerimeter(lm_proj->point_reprojected, 3.0);
        pangolin::glDrawLine(lm_proj->point_measured,
                             lm_proj->point_reprojected);
      }

      glColor3f(1.0, 0.0, 0.0);  // red
      pangolin::GlFont::I()
          .Text("Average repr. error (%u points, %u new outliers): %.2f",
                num_points, num_outliers, error_sum / num_points)
          .Draw(5, text_row);
      text_row += 20;
    }
  }
}

// Update the image views to a given image id
void change_display_to_image(const TimeCamId& tcid) {
  if (0 == tcid.second) {
    // left view
    show_cam1 = 0;
    show_frame1 = tcid.first;
    show_cam1.Meta().gui_changed = true;
    show_frame1.Meta().gui_changed = true;
  } else {
    // right view
    show_cam2 = tcid.second;
    show_frame2 = tcid.first;
    show_cam2.Meta().gui_changed = true;
    show_frame2.Meta().gui_changed = true;
  }
}

// Render the 3D viewer scene of cameras and points
void draw_scene() {
  const TimeCamId tcid1 = std::make_pair(show_frame1, show_cam1);
  const TimeCamId tcid2 = std::make_pair(show_frame2, show_cam2);

  const u_int8_t color_camera_current[3]{255, 0, 0};  // red
  // const u_int8_t color_camera_left[3]{0, 125, 0};
  // dark green
  // const u_int8_t color_camera_right[3]{0, 0, 125};           // dark blue
  const u_int8_t color_points[3]{140, 140, 140};  // gray
  // const u_int8_t color_old_points[3]{170, 170, 170};         // gray
  const u_int8_t color_covisible[3]{0, 200, 0};               // green
  const u_int8_t color_selected_right[3]{0, 0, 250};          // blue
  const u_int8_t color_selected_current_frame[3]{200, 0, 0};  // red
  const u_int8_t color_unoptimized_kf[3]{140, 140, 140};      // gray
  const u_int8_t color_current_kf[3]{0, 200, 0};              // green
  const u_int8_t color_outlier_observation[3]{250, 0, 250};   // purple
  const u_int8_t color_loop_closing_radius[3]{0, 125, 0};     // dark green
  const u_int8_t color_kf_edge_loop_closure[3]{250, 250, 0};    // light green

  Keyframe* current_keyframe = localMapping->getCurrentKeyframe();

  // render cameras
  if (show_cameras3d || show_cv_graph || show_loop_edges ||
      show_spanning_tree || show_loop_detection_rad) {
    std::set<Keyframe*> drawn_kfs;

    if (show_cameras3d) {
      render_camera(tracking->getCurrentPose().matrix(), 2.0f,
                    color_camera_current, 0.2f);
    }

    for (const auto kf : map->getAllKeyframes()) {
      if (show_unopt_poses) {
        bool bSet = false;
        Sophus::SE3d kf_T;
        auto uncorrIter = loopClosing->mUncorrectedPoses.find(kf);

        if (uncorrIter != loopClosing->mUncorrectedPoses.end()) {
          kf_T = uncorrIter->second;
          bSet = true;
        } else {
          auto oldIter = loopClosing->mOldPoses.find(kf);

          if (oldIter != loopClosing->mOldPoses.end()) {
            kf_T = oldIter->second;
            bSet = true;
          }
        }

        if (bSet) {
          render_camera(kf_T.matrix(), 3.0f, color_unoptimized_kf, 0.2f);
        }
      }
      if (show_cameras3d) {
        if (current_keyframe->hasConnectedNeighbor(kf)) {
          render_camera(kf->getCamToWorld().matrix(), 3.0f, color_covisible,
                        0.2f);
        } else if (kf == current_keyframe) {
          render_camera(kf->getCamToWorld().matrix(), 3.0f, color_current_kf,
                        0.2f);

          if (show_loop_detection_rad) {
            // Draw loop closure line
            glLineWidth(2.0f);
            glColor3ubv(color_loop_closing_radius);
            draw_sphere(current_keyframe->getCamToWorld().translation(),
                        loop_dist_thresh);
          }

        } else {
          render_camera(kf->getCamToWorld().matrix(), 3.0f,
                        color_selected_right, 0.2f);
        }
      }

      Eigen::Vector3d kf_trans = kf->getCamToWorld().translation();

      if (show_cv_graph /* || show_essential_graph*/) {
        std::vector<Keyframe*> neighbors = kf->getSortedCovisibleKeyframes();

        for (auto& neighbor : neighbors) {
          if (drawn_kfs.find(neighbor) == drawn_kfs.end()) {
            Eigen::Vector3d n_trans = neighbor->getCamToWorld().translation();

            /*if (show_essential_graph &&
                kf->getNeighborWeight(neighbor) >=
                    Keyframe::MIN_ESSENTIAL_WEIGHT_THRESH) {
              glLineWidth(3.0f);
              glColor3ubv(color_kf_edge_loop_closure);
              glBegin(GL_LINES);
              glVertex3d(kf_trans[0], kf_trans[1], kf_trans[2]);
              glVertex3d(n_trans[0], n_trans[1], n_trans[2]);
              glEnd();
            } else*/
            if (show_cv_graph) {
              glLineWidth(1.5f);
              glColor3ubv(color_camera_current);
              glBegin(GL_LINES);
              glVertex3d(kf_trans[0], kf_trans[1], kf_trans[2]);
              glVertex3d(n_trans[0], n_trans[1], n_trans[2]);
              glEnd();
            }
          }
        }
      }

      if (show_loop_edges) {
        std::set<Keyframe*> kf_loop_edges = kf->getLoopEdges();

        for (auto& kf_edge : kf_loop_edges) {
          if (drawn_kfs.find(kf_edge) == drawn_kfs.end()) {
            Eigen::Vector3d edge_trans = kf_edge->getCamToWorld().translation();

            glLineWidth(3.0f);
            glColor3ubv(color_kf_edge_loop_closure);
            glBegin(GL_LINES);
            glVertex3d(kf_trans[0], kf_trans[1], kf_trans[2]);
            glVertex3d(edge_trans[0], edge_trans[1], edge_trans[2]);
            glEnd();
          }
        }
      }

      if (show_spanning_tree) {
        std::set<Keyframe*> kf_children = kf->getChildren();
        for (auto& child : kf_children) {
          if (drawn_kfs.find(child) == drawn_kfs.end()) {
            Eigen::Vector3d c_trans = child->getCamToWorld().translation();

            glLineWidth(1.0f);
            glColor3ubv(color_outlier_observation);
            glBegin(GL_LINES);
            glVertex3d(kf_trans[0], kf_trans[1], kf_trans[2]);
            glVertex3d(c_trans[0], c_trans[1], c_trans[2]);
            glEnd();
          }
        }
      }

      // Put in set so that it is not drawn twice
      drawn_kfs.emplace(kf);
    }
  }

  // render points
  if (show_points3d && map->getAllLandmarks().size() > 0) {
    std::vector<Landmark*> proj_landmarks = tracking->getProjectedLandmarks();
    glPointSize(3.0);
    glBegin(GL_POINTS);
    for (const auto& kv_lm : map->getAllLandmarks()) {
      FrameId frame1_id = tcid1.first;
      FrameId frame2_id = tcid2.first;
      const bool in_cam_1 = kv_lm->isInObservations(current_keyframe->leftView);
      const bool in_cam_2 =
          kv_lm->isInObservations(current_keyframe->rightView);
      const bool observed_in_current_kf = in_cam_1 || in_cam_2;
      bool observed_in_neighbor_kf = false;
      bool observed_in_current_frame = false;

      const bool outlier_in_cam_1 = kv_lm->isInOutlierObservations(frame1_id);
      const bool outlier_in_cam_2 = kv_lm->isInOutlierObservations(frame2_id);

      // Disable this for now: Draws landmarks of neighbor keyframes in green

      for (auto& neighbor : current_keyframe->getSortedCovisibleKeyframes()) {
        if (kv_lm->isInObservations(neighbor->leftView) ||
            kv_lm->isInObservations(neighbor->rightView)) {
          observed_in_neighbor_kf = true;
          break;
        }
      }

      if (!proj_landmarks.empty()) {
        auto iter =
            std::find(proj_landmarks.begin(), proj_landmarks.end(), kv_lm);

        if (iter != proj_landmarks.end()) {
          observed_in_current_frame = true;
        }
      }

      if (observed_in_current_frame) {
        glColor3ubv(color_selected_current_frame);
        //} else if (observed_in_current_kf) {
        //  glColor3ubv(color_selected_both);
      } else if (observed_in_neighbor_kf || observed_in_current_kf) {
        glColor3ubv(color_covisible);
      } else if (outlier_in_cam_1 || outlier_in_cam_2) {
        glColor3ubv(color_outlier_observation);
      } else {
        glColor3ubv(color_points);
      }

      pangolin::glVertex(kv_lm->getWorldPos());
    }
    glEnd();
  }
}

// Load images, calibration, and features / matches if available
void load_data(const std::string& dataset_path, const std::string& calib_path) {
  const std::string timestams_path = dataset_path + "/cam0/data.csv";

  {
    std::ifstream times(timestams_path);

    int id = 0;

    while (times) {
      std::string line;
      std::getline(times, line);

      if (line.size() < 20 || line[0] == '#' || id > 2700) continue;

      std::string img_name = line.substr(20, line.size() - 21);

      // ensure that we actually read a new timestamp (and not e.g. just newline
      // at the end of the file)
      if (times.fail()) {
        times.clear();
        std::string temp;
        times >> temp;
        if (temp.size() > 0) {
          std::cerr << "Skipping '" << temp << "' while reading times."
                    << std::endl;
        }
        continue;
      }

      for (int i = 0; i < NUM_CAMS; i++) {
        TimeCamId tcid(id, i);

        //        std::stringstream ss;
        //        ss << dataset_path << "/" << timestamp << "_" << i << ".jpg";
        //        pangolin::TypedImage img = pangolin::LoadImage(ss.str());
        //        images[tcid] = std::move(img);

        std::stringstream ss;
        ss << dataset_path << "/cam" << i << "/data/" << img_name;

        images[tcid] = ss.str();
      }

      id++;
    }

    std::cerr << "Loaded " << id << " images " << std::endl;
  }

  {
    std::ifstream os(calib_path, std::ios::binary);

    if (os.is_open()) {
      cereal::JSONInputArchive archive(os);
      archive(calib_cam);
      std::cout << "Loaded camera" << std::endl;

    } else {
      std::cerr << "could not load camera calibration " << calib_path
                << std::endl;
      std::abort();
    }
  }

  show_frame1.Meta().range[1] = images.size() / NUM_CAMS - 1;
  show_frame1.Meta().gui_changed = true;
  show_frame2.Meta().range[1] = images.size() / NUM_CAMS - 1;
  show_frame2.Meta().gui_changed = true;
}

// WARNING: This is a bit hacky and possibly not threadsafe. Consider making
// Settings member variables atomic.
void updateSettings() {
  settings.cam_z_threshold = cam_z_threshold;
  settings.num_features_per_image = num_features_per_image;
  settings.rotate_features = rotate_features;
  settings.match_max_dist_2d = match_max_dist_2d;
  settings.feature_match_max_dist = feature_match_max_dist;
  settings.feature_match_test_next_best = feature_match_test_next_best;
  settings.reprojection_error_pnp_inlier_threshold_pixel =
      reprojection_error_pnp_inlier_threshold_pixel;
  settings.new_kf_min_inliers = new_kf_min_inliers;
  settings.loop_dist_thresh = loop_dist_thresh;
  settings.loop_angle_thresh = loop_angle_thresh;
  settings.loop_min_inliers = loop_min_inliers;
  settings.ba_optimize_intrinsics = ba_optimize_intrinsics;
  settings.ba_verbose = ba_verbose;
  settings.ba_use_huber = ba_use_huber;
  settings.localBA_huber_parameter = localBA_huber_parameter;
  settings.globalBA_huber_parameter = globalBA_huber_parameter;
  settings.localBA_max_iterations = localBA_max_iterations;
  settings.globalBA_max_iterations = globalBA_max_iterations;
}

///////////////////////////////////////////////////////////////////////////////
/// Here the algorithmically interesting implementation begins
///////////////////////////////////////////////////////////////////////////////

// Execute next step in the overall odometry pipeline. Call this repeatedly
// until it returns false for automatic execution.
bool next_step() {
  if (current_frame >= int(images.size()) / NUM_CAMS) return false;

  bool newKF = false;

  updateSettings();

  TimeCamId tcidl(current_frame, 0), tcidr(current_frame, 1);

  newKF = tracking->run(images[tcidl], images[tcidr]);

  if (newKF) {
    localMapping->run();
    loopClosing->run();
  }

  // update image views
  change_display_to_image(tcidl);
  change_display_to_image(tcidr);

  compute_projections();

  current_frame++;

  return true;
}

// Compute reprojections for all landmark observations for visualization and
// outlier removal.
void compute_projections() {
  image_projections.clear();

  std::vector<Landmark*> Landmarks = map->getAllLandmarks();
  for (const auto& kv_lm : Landmarks) {
    for (auto& kv_obs : kv_lm->getObservations()) {
      View* view = kv_obs.first;
      const TimeCamId tcid = kv_obs.first->id();
      const Eigen::Vector2d p_2d_corner =
          view->getKeypoints().corners[kv_obs.second];

      const Eigen::Vector3d p_c =
          view->getCamToWorld().inverse() * kv_lm->getWorldPos();
      const Eigen::Vector2d p_2d_repoj =
          calib_cam.intrinsics.at(tcid.second)->project(p_c);

      ProjectedLandmarkPtr proj_lm(new ProjectedLandmark);
      proj_lm->point_measured = p_2d_corner;
      proj_lm->point_reprojected = p_2d_repoj;
      proj_lm->point_3d_c = p_c;
      proj_lm->reprojection_error = (p_2d_corner - p_2d_repoj).norm();

      image_projections[tcid].obs.push_back(proj_lm);
    }

    for (const auto& kv_obs : kv_lm->getOutlierObservations()) {
      View* view = kv_obs.first;
      const TimeCamId& tcid = kv_obs.first->id();
      const Eigen::Vector2d p_2d_corner =
          view->getKeypoints().corners[kv_obs.second];

      const Eigen::Vector3d p_c =
          view->getCamToWorld().inverse() * kv_lm->getWorldPos();
      const Eigen::Vector2d p_2d_repoj =
          calib_cam.intrinsics.at(tcid.second)->project(p_c);

      ProjectedLandmarkPtr proj_lm(new ProjectedLandmark);
      proj_lm->point_measured = p_2d_corner;
      proj_lm->point_reprojected = p_2d_repoj;
      proj_lm->point_3d_c = p_c;
      proj_lm->reprojection_error = (p_2d_corner - p_2d_repoj).norm();

      image_projections[tcid].outlier_obs.push_back(proj_lm);
    }
  }
}
