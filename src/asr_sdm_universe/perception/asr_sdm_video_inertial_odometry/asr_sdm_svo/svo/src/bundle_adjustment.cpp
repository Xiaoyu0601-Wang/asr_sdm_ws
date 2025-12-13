// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/**
 * @file bundle_adjustment.cpp
 * @brief Bundle adjustment optimization using g2o graph optimization framework.
 * 
 * This module implements bundle adjustment (BA) for refining camera poses and
 * 3D point positions simultaneously. It provides:
 * - Two-view BA: Optimizes poses and points from two keyframes
 * - Local BA: Optimizes a local window of keyframes and their observed points
 * - Global BA: Full optimization of all keyframes and map points
 * 
 * The optimization uses the Levenberg-Marquardt algorithm with Huber robust
 * kernels to handle outliers. The Schur complement trick is used to efficiently
 * solve the sparse linear system by marginalizing 3D point variables.
 */

#include <vikit/math_utils.h>
#include <boost/thread.hpp>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include <svo/bundle_adjustment.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/config.h>
#include <svo/map.h>

/// Enable Schur complement trick for efficient BA (marginalizes 3D points)
#define SCHUR_TRICK 1

namespace svo {
namespace ba {

/**
 * @brief Performs two-view bundle adjustment between two keyframes.
 * 
 * Optimizes the poses of two keyframes and positions of all commonly observed
 * 3D points. The first keyframe is fixed as the reference frame. Points with
 * reprojection error exceeding the threshold are removed from the map.
 * 
 * @param frame1 First keyframe (fixed during optimization)
 * @param frame2 Second keyframe (optimized)
 * @param reproj_thresh Reprojection error threshold in pixels for outlier removal
 * @param map Pointer to the map for point deletion
 */
void twoViewBA(
    Frame* frame1,
    Frame* frame2,
    double reproj_thresh,
    Map* map)
{
  // Scale reprojection threshold from pixels to normalized image plane (unit plane)
  // errorMultiplier2() returns focal length, converting pixel error to angular error
  reproj_thresh /= frame1->cam_->errorMultiplier2();

  // Initialize the g2o sparse optimizer with Levenberg-Marquardt algorithm
  g2o::SparseOptimizer optimizer;
  setupG2o(&optimizer);

  // Container for edges (reprojection constraints) for later outlier removal
  list<EdgeContainerSE3> edges;
  size_t v_id = 0;  // Vertex ID counter

  // Add first keyframe as a fixed vertex (anchor for the optimization)
  g2oFrameSE3* v_frame1 = createG2oFrameSE3(frame1, v_id++, true);
  optimizer.addVertex(v_frame1);

  // Add second keyframe as an optimizable vertex
  g2oFrameSE3* v_frame2 = createG2oFrameSE3(frame2, v_id++, false);
  optimizer.addVertex(v_frame2);

  // Create vertices for all 3D points observed in frame1 and add reprojection edges
  for(Features::iterator it_ftr=frame1->fts_.begin(); it_ftr!=frame1->fts_.end(); ++it_ftr)
  {
    Point* pt = (*it_ftr)->point;
    if(pt == NULL)
      continue;
    
    // Create 3D point vertex
    g2oPoint* v_pt = createG2oPoint(pt->pos_, v_id++, false);
    optimizer.addVertex(v_pt);
    pt->v_pt_ = v_pt;  // Store vertex pointer for later update
    
    // Add edge from frame1 to point (reprojection constraint)
    // Huber width scaled by robust kernel width parameter
    g2oEdgeSE3* e = createG2oEdgeSE3(v_frame1, v_pt, vk::project2d((*it_ftr)->f), true, reproj_thresh*Config::lobaRobustHuberWidth());
    optimizer.addEdge(e);
    edges.push_back(EdgeContainerSE3(e, frame1, *it_ftr));

    // Find corresponding observation in frame2 and add edge
    Feature* ftr_frame2 = pt->findFrameRef(frame2);
    e = createG2oEdgeSE3(v_frame2, v_pt, vk::project2d(ftr_frame2->f), true, reproj_thresh*Config::lobaRobustHuberWidth());
    optimizer.addEdge(e);
    edges.push_back(EdgeContainerSE3(e, frame2, ftr_frame2));
  }

  // Run the optimization
  double init_error, final_error;
  runSparseBAOptimizer(&optimizer, Config::lobaNumIter(), init_error, final_error);
  printf("2-View BA: Error before/after = %f / %f\n", init_error, final_error);

  // Update keyframe poses from optimized vertices
  frame1->T_f_w_.rotationMatrix() = v_frame1->estimate().rotation().toRotationMatrix();
  frame1->T_f_w_.translation() = v_frame1->estimate().translation();
  frame2->T_f_w_.rotationMatrix() = v_frame2->estimate().rotation().toRotationMatrix();
  frame2->T_f_w_.translation() = v_frame2->estimate().translation();

  // Update 3D point positions from optimized vertices
  for(Features::iterator it=frame1->fts_.begin(); it!=frame1->fts_.end(); ++it)
  {
    if((*it)->point == NULL)
     continue;
    (*it)->point->pos_ = (*it)->point->v_pt_->estimate();
    (*it)->point->v_pt_ = NULL;  // Clear temporary vertex pointer
  }

  // Remove points with reprojection error exceeding threshold (outliers)
  const double reproj_thresh_squared = reproj_thresh*reproj_thresh;
  size_t n_incorrect_edges = 0;
  for(list<EdgeContainerSE3>::iterator it_e = edges.begin(); it_e != edges.end(); ++it_e)
    if(it_e->edge->chi2() > reproj_thresh_squared)
    {
      if(it_e->feature->point != NULL)
      {
        map->safeDeletePoint(it_e->feature->point);
        it_e->feature->point = NULL;
      }
      ++n_incorrect_edges;
    }

  printf("2-View BA: Wrong edges =  %zu\n", n_incorrect_edges);
}

/**
 * @brief Performs local bundle adjustment on a window of keyframes.
 * 
 * Optimizes core keyframes and all their observed 3D points. Neighboring keyframes
 * that observe the same points are added as fixed constraints. This provides a
 * good balance between accuracy and computational efficiency.
 * 
 * @param center_kf The central keyframe around which to perform local BA
 * @param core_kfs Set of keyframes to optimize (excluding fixed neighbors)
 * @param map Pointer to the map for point/reference removal
 * @param n_incorrect_edges_1 Output: number of edges exceeding pose optimization threshold
 * @param n_incorrect_edges_2 Output: number of edges exceeding local BA threshold
 * @param init_error Output: initial reprojection error before optimization
 * @param final_error Output: final reprojection error after optimization
 */
void localBA(
    Frame* center_kf,
    set<FramePtr>* core_kfs,
    Map* map,
    size_t& n_incorrect_edges_1,
    size_t& n_incorrect_edges_2,
    double& init_error,
    double& final_error)
{

  // Initialize g2o optimizer
  g2o::SparseOptimizer optimizer;
  setupG2o(&optimizer);

  // Data structures for tracking optimization elements
  list<EdgeContainerSE3> edges;  // All reprojection edges
  set<Point*> mps;               // Set of all map points to optimize
  list<Frame*> neib_kfs;         // Neighboring keyframes (fixed)
  size_t v_id = 0;               // Vertex ID counter
  size_t n_mps = 0;              // Number of map points
  size_t n_fix_kfs = 0;          // Number of fixed keyframes
  size_t n_var_kfs = 1;          // Number of variable keyframes
  size_t n_edges = 0;            // Number of edges
  n_incorrect_edges_1 = 0;
  n_incorrect_edges_2 = 0;

  // Add all core keyframes as optimizable vertices
  for(set<FramePtr>::iterator it_kf = core_kfs->begin(); it_kf != core_kfs->end(); ++it_kf)
  {
    g2oFrameSE3* v_kf = createG2oFrameSE3(it_kf->get(), v_id++, false);
    (*it_kf)->v_kf_ = v_kf;  // Store vertex pointer for later update
    ++n_var_kfs;
    assert(optimizer.addVertex(v_kf));

    // Collect all points observed by core keyframes for optimization
    for(Features::iterator it_pt=(*it_kf)->fts_.begin(); it_pt!=(*it_kf)->fts_.end(); ++it_pt)
      if((*it_pt)->point != NULL)
        mps.insert((*it_pt)->point);
  }

  // Convert thresholds from pixels to normalized image plane
  double reproj_thresh_2 = Config::lobaThresh() / center_kf->cam_->errorMultiplier2();
  double reproj_thresh_1 = Config::poseOptimThresh() / center_kf->cam_->errorMultiplier2();
  double reproj_thresh_1_squared = reproj_thresh_1*reproj_thresh_1;
  
  // Process all collected map points
  for(set<Point*>::iterator it_pt = mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    // Create 3D point vertex
    g2oPoint* v_pt = createG2oPoint((*it_pt)->pos_, v_id++, false);
    (*it_pt)->v_pt_ = v_pt;
    assert(optimizer.addVertex(v_pt));
    ++n_mps;

    // Add edges for all observations of this point
    list<Feature*>::iterator it_obs=(*it_pt)->obs_.begin();
    while(it_obs!=(*it_pt)->obs_.end())
    {
      // Compute current reprojection error for reference
      Vector2d error = vk::project2d((*it_obs)->f) - vk::project2d((*it_obs)->frame->w2f((*it_pt)->pos_));

      if((*it_obs)->frame->v_kf_ == NULL)
      {
        // Frame is not in core keyframes - add as fixed neighbor
        g2oFrameSE3* v_kf = createG2oFrameSE3((*it_obs)->frame, v_id++, true);
        (*it_obs)->frame->v_kf_ = v_kf;
        ++n_fix_kfs;
        assert(optimizer.addVertex(v_kf));
        neib_kfs.push_back((*it_obs)->frame);
      }

      // Create reprojection edge with pyramid level-dependent weight
      // Higher pyramid levels have lower weight (1/(2^level))
      g2oEdgeSE3* e = createG2oEdgeSE3((*it_obs)->frame->v_kf_, v_pt,
                                       vk::project2d((*it_obs)->f),
                                       true,
                                       reproj_thresh_2*Config::lobaRobustHuberWidth(),
                                       1.0 / (1<<(*it_obs)->level));
      assert(optimizer.addEdge(e));
      edges.push_back(EdgeContainerSE3(e, (*it_obs)->frame, *it_obs));
      ++n_edges;
      ++it_obs;
    }
  }

  // Structure-only optimization: first optimize only 3D point positions
  // This provides a better initialization for full BA
  g2o::StructureOnlySolver<3> structure_only_ba;
  g2o::OptimizableGraph::VertexContainer points;
  for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
  {
    g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
      if (v->dimension() == 3 && v->edges().size() >= 2)  // 3D points with 2+ observations
        points.push_back(v);
  }
  structure_only_ba.calc(points, 10);  // 10 iterations of structure-only optimization

  // Run full bundle adjustment (joint optimization of poses and points)
  if(Config::lobaNumIter() > 0)
    runSparseBAOptimizer(&optimizer, Config::lobaNumIter(), init_error, final_error);

  // Update core keyframe poses from optimized vertices
  for(set<FramePtr>::iterator it = core_kfs->begin(); it != core_kfs->end(); ++it)
  {
    (*it)->T_f_w_ = SE3( (*it)->v_kf_->estimate().rotation(),
                         (*it)->v_kf_->estimate().translation());
    (*it)->v_kf_ = NULL;  // Clear temporary vertex pointer
  }

  // Clear vertex pointers from neighbor keyframes
  for(list<Frame*>::iterator it = neib_kfs.begin(); it != neib_kfs.end(); ++it)
    (*it)->v_kf_ = NULL;

  // Update 3D point positions from optimized vertices
  for(set<Point*>::iterator it = mps.begin(); it != mps.end(); ++it)
  {
    (*it)->pos_ = (*it)->v_pt_->estimate();
    (*it)->v_pt_ = NULL;
  }

  // Remove observations with reprojection error exceeding threshold
  double reproj_thresh_2_squared = reproj_thresh_2*reproj_thresh_2;
  for(list<EdgeContainerSE3>::iterator it = edges.begin(); it != edges.end(); ++it)
  {
    if(it->edge->chi2() > reproj_thresh_2_squared)
    {
      map->removePtFrameRef(it->frame, it->feature);
      ++n_incorrect_edges_2;
    }
  }

  // Convert errors back to pixel units for reporting
  init_error = sqrt(init_error)*center_kf->cam_->errorMultiplier2();
  final_error = sqrt(final_error)*center_kf->cam_->errorMultiplier2();
}

/**
 * @brief Performs global bundle adjustment on all keyframes and map points.
 * 
 * Full optimization of the entire map. This is computationally expensive and
 * should only be called when necessary (e.g., loop closure or final refinement).
 * 
 * @param map Pointer to the map containing all keyframes and points
 */
void globalBA(Map* map)
{
  // Initialize g2o optimizer
  g2o::SparseOptimizer optimizer;
  setupG2o(&optimizer);

  list<EdgeContainerSE3> edges;
  list< pair<FramePtr,Feature*> > incorrect_edges;

  // Process all keyframes in the map
  size_t v_id = 0;
  double reproj_thresh_2 = Config::lobaThresh() / map->lastKeyframe()->cam_->errorMultiplier2();
  double reproj_thresh_1_squared = Config::poseOptimThresh() / map->lastKeyframe()->cam_->errorMultiplier2();
  reproj_thresh_1_squared *= reproj_thresh_1_squared;
  
  for(list<FramePtr>::iterator it_kf = map->keyframes_.begin();
      it_kf != map->keyframes_.end(); ++it_kf)
  {
    // Add keyframe vertex
    g2oFrameSE3* v_kf = createG2oFrameSE3(it_kf->get(), v_id++, false);
    (*it_kf)->v_kf_ = v_kf;
    optimizer.addVertex(v_kf);
    
    // Process all features in this keyframe
    for(Features::iterator it_ftr=(*it_kf)->fts_.begin(); it_ftr!=(*it_kf)->fts_.end(); ++it_ftr)
    {
      Point* mp = (*it_ftr)->point;
      if(mp == NULL)
        continue;
      
      g2oPoint* v_mp = mp->v_pt_;
      if(v_mp == NULL)
      {
        // Create point vertex if it doesn't exist yet
        v_mp = createG2oPoint(mp->pos_, v_id++, false);
        mp->v_pt_ = v_mp;
        optimizer.addVertex(v_mp);
      }

      // Check for large reprojection errors (possible due to point merging)
      // These are excluded from optimization to prevent distorted results
      Vector2d error = vk::project2d((*it_ftr)->f) - vk::project2d((*it_kf)->w2f(mp->pos_));
      if(error.squaredNorm() > reproj_thresh_1_squared)
        incorrect_edges.push_back(pair<FramePtr,Feature*>(*it_kf, *it_ftr));
      else
      {
        // Add reprojection edge
        g2oEdgeSE3* e = createG2oEdgeSE3(v_kf, v_mp, vk::project2d((*it_ftr)->f),
                                         true,
                                         reproj_thresh_2*Config::lobaRobustHuberWidth());

        edges.push_back(EdgeContainerSE3(e, it_kf->get(), *it_ftr));
        optimizer.addEdge(e);
      }
    }
  }

  // Run optimization
  double init_error=0.0, final_error=0.0;
  if(Config::lobaNumIter() > 0)
    runSparseBAOptimizer(&optimizer, Config::lobaNumIter(), init_error, final_error);

  // Update all keyframe poses and point positions
  for(list<FramePtr>::iterator it_kf = map->keyframes_.begin();
        it_kf != map->keyframes_.end(); ++it_kf)
  {
    (*it_kf)->T_f_w_ = SE3( (*it_kf)->v_kf_->estimate().rotation(),
                            (*it_kf)->v_kf_->estimate().translation());
    (*it_kf)->v_kf_ = NULL;
    for(Features::iterator it_ftr=(*it_kf)->fts_.begin(); it_ftr!=(*it_kf)->fts_.end(); ++it_ftr)
    {
      Point* mp = (*it_ftr)->point;
      if(mp == NULL)
        continue;
      if(mp->v_pt_ == NULL)
        continue;       // Point was already updated
      mp->pos_ = mp->v_pt_->estimate();
      mp->v_pt_ = NULL;
    }
  }

  // Remove pre-identified incorrect edges
  for(list< pair<FramePtr,Feature*> >::iterator it=incorrect_edges.begin();
      it!=incorrect_edges.end(); ++it)
    map->removePtFrameRef(it->first.get(), it->second);

  // Remove edges with large reprojection error after optimization
  double reproj_thresh_2_squared = reproj_thresh_2*reproj_thresh_2;
  for(list<EdgeContainerSE3>::iterator it = edges.begin(); it != edges.end(); ++it)
  {
    if(it->edge->chi2() > reproj_thresh_2_squared)
    {
      map->removePtFrameRef(it->frame, it->feature);
    }
  }
}

/**
 * @brief Configures the g2o optimizer with appropriate solver and algorithm.
 * 
 * Sets up a Levenberg-Marquardt optimizer with Cholmod linear solver.
 * Uses block solver 6x3 when Schur trick is enabled (6-DOF poses, 3D points).
 * 
 * @param optimizer Pointer to the g2o sparse optimizer to configure
 */
void setupG2o(g2o::SparseOptimizer * optimizer)
{
  // Note: Linear solver and block solver are allocated on heap but owned by optimizer
  optimizer->setVerbose(false);

#if SCHUR_TRICK
  // Block solver 6x3: 6-DOF camera poses, 3D point positions
  // Uses Schur complement to efficiently marginalize 3D points
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;
  linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
#else
  // Generic block solver (less efficient, used when Schur trick is disabled)
  g2o::BlockSolverX::LinearSolverType * linearSolver;
  linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
#endif

  solver->setMaxTrialsAfterFailure(5);  // Retry with different damping if step fails
  optimizer->setAlgorithm(solver);

  // Setup camera parameters (unit focal length, zero principal point for normalized coords)
  g2o::CameraParameters * cam_params = new g2o::CameraParameters(1.0, Vector2d(0.,0.), 0.);
  cam_params->setId(0);
  if (!optimizer->addParameter(cam_params)) {
    assert(false);
  }
}

/**
 * @brief Runs the sparse BA optimization for a specified number of iterations.
 * 
 * @param optimizer Pointer to the configured g2o optimizer
 * @param num_iter Number of optimization iterations
 * @param init_error Output: chi-squared error before optimization
 * @param final_error Output: chi-squared error after optimization
 */
void
runSparseBAOptimizer(g2o::SparseOptimizer* optimizer,
                     unsigned int num_iter,
                     double& init_error, double& final_error)
{
  optimizer->initializeOptimization();
  optimizer->computeActiveErrors();
  init_error = optimizer->activeChi2();
  optimizer->optimize(num_iter);
  final_error = optimizer->activeChi2();
}

/**
 * @brief Creates a g2o vertex for a camera frame with SE3 pose.
 * 
 * @param frame Pointer to the frame
 * @param id Unique vertex ID
 * @param fixed Whether the vertex should be fixed (not optimized)
 * @return Pointer to the created g2o frame vertex
 */
g2oFrameSE3*
createG2oFrameSE3(Frame* frame, size_t id, bool fixed)
{
  g2oFrameSE3* v = new g2oFrameSE3();
  v->setId(id);
  v->setFixed(fixed);
  // Convert SE3 pose to g2o's SE3Quat representation (quaternion + translation)
  v->setEstimate(g2o::SE3Quat(frame->T_f_w_.unit_quaternion(), frame->T_f_w_.translation()));
  return v;
}

/**
 * @brief Creates a g2o vertex for a 3D map point.
 * 
 * @param pos 3D position of the point
 * @param id Unique vertex ID
 * @param fixed Whether the vertex should be fixed (not optimized)
 * @return Pointer to the created g2o point vertex
 */
g2oPoint*
createG2oPoint(Vector3d pos,
               size_t id,
               bool fixed)
{
  g2oPoint* v = new g2oPoint();
  v->setId(id);
#if SCHUR_TRICK
  v->setMarginalized(true);  // Enable marginalization for efficient Schur complement
#endif
  v->setFixed(fixed);
  v->setEstimate(pos);
  return v;
}

/**
 * @brief Creates a g2o edge representing a reprojection constraint.
 * 
 * The edge connects a camera pose vertex to a 3D point vertex, constraining
 * the point to project to the observed 2D location in the image.
 * 
 * @param v_frame Camera pose vertex
 * @param v_point 3D point vertex
 * @param f_up Observed 2D point in normalized image coordinates
 * @param robust_kernel Whether to use robust Huber kernel
 * @param huber_width Width parameter for Huber kernel
 * @param weight Information matrix weight (default 1.0)
 * @return Pointer to the created g2o edge
 */
g2oEdgeSE3*
createG2oEdgeSE3( g2oFrameSE3* v_frame,
                  g2oPoint* v_point,
                  const Vector2d& f_up,
                  bool robust_kernel,
                  double huber_width,
                  double weight)
{
  g2oEdgeSE3* e = new g2oEdgeSE3();
  // Note: vertex order matters for g2o's internal structure
  e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_point));
  e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_frame));
  e->setMeasurement(f_up);  // Observed 2D point
  e->information() = weight * Eigen::Matrix2d::Identity(2,2);  // Isotropic measurement noise
  
  // Huber robust kernel for outlier handling
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
  rk->setDelta(huber_width);
  e->setRobustKernel(rk);
  e->setParameterId(0, 0);  // Reference camera parameters
  return e;
}

} // namespace ba
} // namespace svo
