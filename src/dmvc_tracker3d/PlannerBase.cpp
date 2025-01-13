//
// Created by larr-planning on 25. 1. 13.
//
#include <dmvc_tracker3d/PlannerBase.h>

void
dmvc3d::PlannerBase::SetTrackerPrimitives(const std::vector<std::vector<dmvc3d::PrimitivePlanning>> &tracker_primitive) {
    tracker_raw_primitives_.clear();
    tracker_raw_primitives_ = tracker_primitive;
}

void dmvc3d::PlannerBase::EraseTrackerPrimitives() {
    tracker_raw_primitives_.clear();
}

void dmvc3d::PlannerBase::EraseFeasibleIndex() {
    tracker_feasible_index_.clear();
}

void dmvc3d::PlannerBase::SetFeasibleIndex(const std::vector<std::vector<uint>> &feasible_index) {
    tracker_feasible_index_.clear();
    tracker_feasible_index_ = feasible_index;
//    printf("EMPTY?: %d\n",feasible_index.size());
}

void dmvc3d::PlannerBase::EraseBestIndex() {
    tracker_best_index_.clear();
}

void dmvc3d::PlannerBase::SetBestIndex(const std::vector<uint> &best_index) {
    tracker_best_index_.clear();
    tracker_best_index_ = best_index;
}

void dmvc3d::PlannerBase::EraseMovingBufferedVoronoiCell() {
    buffered_voronoi_cell_.clear();
}

void
dmvc3d::PlannerBase::SetMovingBufferedVoronoiCell(const std::vector<std::vector<dmvc3d::AffineCoeff2D>> &voronoi_cell) {
    buffered_voronoi_cell_.clear();
    buffered_voronoi_cell_ = voronoi_cell;
}

void dmvc3d::PlannerBase::SetVisibilityCell(const std::vector<std::vector<dmvc3d::AffineCoeff2D>> &visibility_cell) {
    visibility_cell_.clear();
    visibility_cell_ = visibility_cell;
}

void dmvc3d::PlannerBase::EraseVisibilityCell() {
    visibility_cell_.clear();
}

void dmvc3d::PlannerBase::SetCorridorVis(const vec_E<Polyhedron3D> &poly) {
    polys_.clear();
    polys_ = poly;
}