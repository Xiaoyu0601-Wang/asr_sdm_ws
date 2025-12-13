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
 * @file map.cpp
 * @brief Map management including keyframes, points, and candidates.
 * 
 * The Map class manages the visual odometry state consisting of:
 * - Keyframes: Camera poses with associated features
 * - Points: 3D landmarks observed by multiple keyframes
 * - Candidates: Potential new 3D points from depth filter
 * 
 * Key responsibilities:
 * - Keyframe insertion and removal
 * - Point lifecycle management (creation, deletion, garbage collection)
 * - Spatial queries (closest/furthest keyframes)
 * - Reference consistency between frames and points
 * 
 * The candidate point system allows depth filter outputs to be validated
 * before full inclusion in the map.
 */

#include <set>
#include <svo/map.h>
#include <svo/point.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <boost/bind.hpp>

namespace svo {

/**
 * @brief Default constructor.
 */
Map::Map() {}

/**
 * @brief Destructor - cleans up all map data.
 */
Map::~Map()
{
  reset();
  SVO_INFO_STREAM("Map destructed");
}

/**
 * @brief Resets map to empty state.
 * 
 * Clears all keyframes, candidates, and garbage collects trash.
 */
void Map::reset()
{
  keyframes_.clear();
  point_candidates_.reset();
  emptyTrash();
}

/**
 * @brief Safely deletes a keyframe and all its references.
 * 
 * Removes all point-frame references before deletion to maintain
 * consistency. Also removes any candidate points associated with
 * the frame.
 * 
 * @param frame Keyframe to delete
 * @return true if frame was found and deleted
 */
bool Map::safeDeleteFrame(FramePtr frame)
{
  bool found = false;
  for(auto it=keyframes_.begin(), ite=keyframes_.end(); it!=ite; ++it)
  {
    if(*it == frame)
    {
      // Remove all point references from this frame's features
      std::for_each((*it)->fts_.begin(), (*it)->fts_.end(), [&](Feature* ftr){
        removePtFrameRef(it->get(), ftr);
      });
      keyframes_.erase(it);
      found = true;
      break;
    }
  }

  // Also remove any candidate points from this frame
  point_candidates_.removeFrameCandidates(frame);

  if(found)
    return true;

  SVO_ERROR_STREAM("Tried to delete Keyframe in map which was not there.");
  return false;
}

/**
 * @brief Removes a point-frame reference and potentially deletes the point.
 * 
 * When a point loses a reference, check if it has too few observations
 * to remain viable. Points with <=2 observations are deleted.
 * 
 * @param frame Frame containing the feature
 * @param ftr Feature referencing the point
 */
void Map::removePtFrameRef(Frame* frame, Feature* ftr)
{
  if(ftr->point == NULL)
    return;  // Point may have been deleted already
  
  Point* pt = ftr->point;
  ftr->point = NULL;  // Clear feature's reference to point
  
  if(pt->obs_.size() <= 2)
  {
    // Too few observations - delete the point entirely
    safeDeletePoint(pt);
    return;
  }
  
  pt->deleteFrameRef(frame);    // Remove reference from point's observer list
  frame->removeKeyPoint(ftr);   // Check if point was a key point in frame
}

/**
 * @brief Safely deletes a point and all its references.
 * 
 * Iterates through all features observing this point and clears their
 * references, then moves point to trash for deferred deletion.
 * 
 * @param pt Point to delete
 */
void Map::safeDeletePoint(Point* pt)
{
  // Clear all feature references to this point
  std::for_each(pt->obs_.begin(), pt->obs_.end(), [&](Feature* ftr){
    ftr->point=NULL;
    ftr->frame->removeKeyPoint(ftr);
  });
  pt->obs_.clear();

  // Move to trash for deferred deletion
  deletePoint(pt);
}

/**
 * @brief Marks a point as deleted and moves to trash.
 * 
 * Actual memory deallocation happens in emptyTrash() to allow
 * visualization of deleted points before cleanup.
 * 
 * @param pt Point to delete
 */
void Map::deletePoint(Point* pt)
{
  pt->type_ = Point::TYPE_DELETED;
  trash_points_.push_back(pt);
}

/**
 * @brief Adds a new keyframe to the map.
 * 
 * @param new_keyframe Keyframe to add
 */
void Map::addKeyframe(FramePtr new_keyframe)
{
  keyframes_.push_back(new_keyframe);
}

/**
 * @brief Finds keyframes with overlapping field of view.
 * 
 * Uses key points for efficient visibility testing. If any key point
 * of a keyframe is visible in the query frame, the keyframe is
 * considered to have overlap.
 * 
 * @param frame Query frame
 * @param close_kfs Output: list of (keyframe, distance) pairs
 */
void Map::getCloseKeyframes(
    const FramePtr& frame,
    std::list< std::pair<FramePtr,double> >& close_kfs) const
{
  for(auto kf : keyframes_)
  {
    // Check if any key point is visible in frame
    for(auto keypoint : kf->key_pts_)
    {
      if(keypoint == nullptr)
        continue;

      if(frame->isVisible(keypoint->point->pos_))
      {
        // Keyframe has overlapping field of view
        close_kfs.push_back(
            std::make_pair(
                kf, (frame->T_f_w_.translation()-kf->T_f_w_.translation()).norm()));
        break;
      }
    }
  }
}

/**
 * @brief Finds the closest keyframe to the query frame.
 * 
 * Among keyframes with overlapping field of view, returns the one
 * closest in terms of translation.
 * 
 * @param frame Query frame
 * @return Closest keyframe with overlap, or nullptr if none found
 */
FramePtr Map::getClosestKeyframe(const FramePtr& frame) const
{
  list< pair<FramePtr,double> > close_kfs;
  getCloseKeyframes(frame, close_kfs);
  if(close_kfs.empty())
  {
    return nullptr;
  }

  // Sort by distance (ascending)
  close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) <
                 boost::bind(&std::pair<FramePtr, double>::second, _2));

  // Return closest that isn't the query frame itself
  if(close_kfs.front().first != frame)
    return close_kfs.front().first;
  close_kfs.pop_front();
  return close_kfs.front().first;
}

/**
 * @brief Finds the keyframe furthest from a 3D position.
 * 
 * Used for keyframe culling - remove the most distant keyframe
 * when the map grows too large.
 * 
 * @param pos Query 3D position
 * @return Furthest keyframe
 */
FramePtr Map::getFurthestKeyframe(const Vector3d& pos) const
{
  FramePtr furthest_kf;
  double maxdist = 0.0;
  for(auto it=keyframes_.begin(), ite=keyframes_.end(); it!=ite; ++it)
  {
    double dist = ((*it)->pos()-pos).norm();
    if(dist > maxdist) {
      maxdist = dist;
      furthest_kf = *it;
    }
  }
  return furthest_kf;
}

/**
 * @brief Retrieves a keyframe by its unique ID.
 * 
 * @param id Frame ID to search for
 * @param frame Output: found keyframe
 * @return true if found
 */
bool Map::getKeyframeById(const int id, FramePtr& frame) const
{
  bool found = false;
  for(auto it=keyframes_.begin(), ite=keyframes_.end(); it!=ite; ++it)
    if((*it)->id_ == id) {
      found = true;
      frame = *it;
      break;
    }
  return found;
}

/**
 * @brief Applies a similarity transformation to the entire map.
 * 
 * Used for scale correction or alignment with external systems.
 * Transforms all keyframe poses and 3D point positions.
 * 
 * @param R Rotation component
 * @param t Translation component
 * @param s Scale factor
 */
void Map::transform(const Matrix3d& R, const Vector3d& t, const double& s)
{
  for(auto it=keyframes_.begin(), ite=keyframes_.end(); it!=ite; ++it)
  {
    // Transform keyframe position
    Vector3d pos = s*R*(*it)->pos() + t;
    Matrix3d rot = R*(*it)->T_f_w_.rotationMatrix().inverse();
    (*it)->T_f_w_ = SE3(rot, pos).inverse();
    
    // Transform observed points (mark with timestamp to avoid duplicates)
    for(auto ftr=(*it)->fts_.begin(); ftr!=(*it)->fts_.end(); ++ftr)
    {
      if((*ftr)->point == NULL)
        continue;
      if((*ftr)->point->last_published_ts_ == -1000)
        continue;  // Already transformed
      (*ftr)->point->last_published_ts_ = -1000;  // Mark as transformed
      (*ftr)->point->pos_ = s*R*(*ftr)->point->pos_ + t;
    }
  }
}

/**
 * @brief Garbage collects deleted points.
 * 
 * Called at the end of each frame to actually deallocate memory
 * for points marked as deleted.
 */
void Map::emptyTrash()
{
  std::for_each(trash_points_.begin(), trash_points_.end(), [&](Point*& pt){
    delete pt;
    pt=NULL;
  });
  trash_points_.clear();
  point_candidates_.emptyTrash();
}

// ============================================================================
// MapPointCandidates - Manages candidate 3D points from depth filter
// ============================================================================

/**
 * @brief Default constructor.
 */
MapPointCandidates::MapPointCandidates()
{}

/**
 * @brief Destructor.
 */
MapPointCandidates::~MapPointCandidates()
{
  reset();
}

/**
 * @brief Adds a new candidate point from the depth filter.
 * 
 * Candidate points are potential 3D landmarks that haven't been
 * validated through reprojection yet.
 * 
 * @param point New 3D point
 * @param depth_sigma2 Depth variance (not currently used)
 */
void MapPointCandidates::newCandidatePoint(Point* point, double depth_sigma2)
{
  point->type_ = Point::TYPE_CANDIDATE;
  boost::unique_lock<boost::mutex> lock(mut_);
  candidates_.push_back(PointCandidate(point, point->obs_.front()));
}

/**
 * @brief Promotes candidate points to the frame's feature list.
 * 
 * Called when a frame becomes a keyframe. Candidates whose original
 * observation was in this frame are promoted to regular map points.
 * 
 * @param frame Keyframe to add candidates to
 */
void MapPointCandidates::addCandidatePointToFrame(FramePtr frame)
{
  boost::unique_lock<boost::mutex> lock(mut_);
  PointCandidateList::iterator it=candidates_.begin();
  while(it != candidates_.end())
  {
    if(it->first->obs_.front()->frame == frame.get())
    {
      // Promote to regular map point
      it->first->type_ = Point::TYPE_UNKNOWN;
      it->first->n_failed_reproj_ = 0;
      it->second->frame->addFeature(it->second);
      it = candidates_.erase(it);
    }
    else
      ++it;
  }
}

/**
 * @brief Deletes a specific candidate point.
 * 
 * @param point Point to delete
 * @return true if found and deleted
 */
bool MapPointCandidates::deleteCandidatePoint(Point* point)
{
  boost::unique_lock<boost::mutex> lock(mut_);
  for(auto it=candidates_.begin(), ite=candidates_.end(); it!=ite; ++it)
  {
    if(it->first == point)
    {
      deleteCandidate(*it);
      candidates_.erase(it);
      return true;
    }
  }
  return false;
}

/**
 * @brief Removes all candidates associated with a frame.
 * 
 * Called when a keyframe is deleted.
 * 
 * @param frame Frame whose candidates to remove
 */
void MapPointCandidates::removeFrameCandidates(FramePtr frame)
{
  boost::unique_lock<boost::mutex> lock(mut_);
  auto it=candidates_.begin();
  while(it!=candidates_.end())
  {
    if(it->second->frame == frame.get())
    {
      deleteCandidate(*it);
      it = candidates_.erase(it);
    }
    else
      ++it;
  }
}

/**
 * @brief Resets all candidates.
 */
void MapPointCandidates::reset()
{
  boost::unique_lock<boost::mutex> lock(mut_);
  std::for_each(candidates_.begin(), candidates_.end(), [&](PointCandidate& c){
    delete c.first;
    delete c.second;
  });
  candidates_.clear();
}

/**
 * @brief Moves a candidate to trash for deferred deletion.
 * 
 * @param c Candidate to delete
 */
void MapPointCandidates::deleteCandidate(PointCandidate& c)
{
  // Delete feature immediately, but defer point deletion
  // (another frame might still reference it in camera-rig setups)
  delete c.second; c.second=NULL;
  c.first->type_ = Point::TYPE_DELETED;
  trash_points_.push_back(c.first);
}

/**
 * @brief Garbage collects deleted candidate points.
 */
void MapPointCandidates::emptyTrash()
{
  std::for_each(trash_points_.begin(), trash_points_.end(), [&](Point*& p){
    delete p; p=NULL;
  });
  trash_points_.clear();
}

// ============================================================================
// map_debug namespace - Validation and debugging utilities
// ============================================================================

namespace map_debug {

/**
 * @brief Validates all frames in the map for consistency.
 * 
 * @param map Map to validate
 * @param id Debug ID for error messages
 */
void mapValidation(Map* map, int id)
{
  for(auto it=map->keyframes_.begin(); it!=map->keyframes_.end(); ++it)
    frameValidation(it->get(), id);
}

/**
 * @brief Validates a single frame for consistency.
 * 
 * Checks:
 * - No references to deleted points
 * - Bidirectional references between features and points
 * - Key point validity
 * 
 * @param frame Frame to validate
 * @param id Debug ID for error messages
 */
void frameValidation(Frame* frame, int id)
{
  for(auto it = frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point==NULL)
      continue;

    // Check for deleted point references
    if((*it)->point->type_ == Point::TYPE_DELETED)
      printf("ERROR DataValidation %i: Referenced point was deleted.\n", id);

    // Check bidirectional reference
    if(!(*it)->point->findFrameRef(frame))
      printf("ERROR DataValidation %i: Frame has reference but point does not have a reference back.\n", id);

    pointValidation((*it)->point, id);
  }
  
  // Validate key points
  for(auto it=frame->key_pts_.begin(); it!=frame->key_pts_.end(); ++it)
    if(*it != NULL)
      if((*it)->point == NULL)
        printf("ERROR DataValidation %i: KeyPoints not correct!\n", id);
}

/**
 * @brief Validates a single point for consistency.
 * 
 * Checks that all observer features actually reference this point.
 * 
 * @param point Point to validate
 * @param id Debug ID for error messages
 */
void pointValidation(Point* point, int id)
{
  for(auto it=point->obs_.begin(); it!=point->obs_.end(); ++it)
  {
    bool found=false;
    for(auto it_ftr=(*it)->frame->fts_.begin(); it_ftr!=(*it)->frame->fts_.end(); ++it_ftr)
     if((*it_ftr)->point == point) {
       found=true; break;
     }
    if(!found)
      printf("ERROR DataValidation %i: Point %i has inconsistent reference in frame %i, is candidate = %i\n", id, point->id_, (*it)->frame->id_, (int) point->type_);
  }
}

/**
 * @brief Prints map statistics.
 * 
 * Reports average observations per frame and per point.
 * 
 * @param map Map to analyze
 */
void mapStatistics(Map* map)
{
  // Average point observations per frame
  size_t n_pt_obs(0);
  for(auto it=map->keyframes_.begin(); it!=map->keyframes_.end(); ++it)
    n_pt_obs += (*it)->nObs();
  printf("\n\nMap Statistics: Frame avg. point obs = %f\n", (float) n_pt_obs/map->size());

  // Average frame observations per point
  size_t n_frame_obs(0);
  size_t n_pts(0);
  std::set<Point*> points;
  for(auto it=map->keyframes_.begin(); it!=map->keyframes_.end(); ++it)
  {
    for(auto ftr=(*it)->fts_.begin(); ftr!=(*it)->fts_.end(); ++ftr)
    {
      if((*ftr)->point == NULL)
        continue;
      if(points.insert((*ftr)->point).second) {
        ++n_pts;
        n_frame_obs += (*ftr)->point->nRefs();
      }
    }
  }
  printf("Map Statistics: Point avg. frame obs = %f\n\n", (float) n_frame_obs/n_pts);
}

} // namespace map_debug
} // namespace svo
