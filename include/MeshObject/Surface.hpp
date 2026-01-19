#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "MeshObject/EdgeBVH.hpp"

#include "MeshObject/MeshObject.hpp"
#include "MeshObject/Settings.hpp"

#include "Cache/FIFOCache.hpp"

// forward declarations
class Vertex;
class Edge;
class Face;
class InteriorPoint;
class Storage;
class GenericPoint;

#include <omp.h>

enum class RelativePosition
{
    IN_FRONT,
    WITHIN,
    BEHIND,
    NO_RELATIVE_POSITION,
    HIGH_INCIDENT_ANGLE
};

class Surface : public std::enable_shared_from_this<Surface> 
{
protected:
    friend class Storage;
    void initialize_(const std::shared_ptr<Storage>& storage);
    void delete_();

public:

    // read and write lock
    mutable std::shared_mutex rwlock_surface_fitting_;
    mutable std::shared_mutex rwlock_vertices_;
    mutable std::shared_mutex rwlock_edges_;
    mutable std::shared_mutex rwlock_faces_;
    mutable std::shared_mutex rwlock_interior_points_;

    mutable std::shared_mutex rwlock_lifecycle_;

    double compute_point_to_plane_distance(const Eigen::Vector3d& point) const;
    double compute_point_projective_distance(const Eigen::Vector3d& origin, const Eigen::Vector3d& point) const;
    double compute_point_projective_distance(const std::shared_ptr<GenericPoint>& generic_point) const;
    double compute_point_projective_distance(const std::shared_ptr<Vertex>& vertex) const;
    double compute_point_projective_distance_with_improved_covariance(const Eigen::Vector3d& origin, const Eigen::Vector3d& point) const;
    Eigen::Vector3d compute_point_projective_position(const Eigen::Vector3d& origin, const Eigen::Vector3d& point) const;

    RelativePosition check_relative_position(double distance_travelled, const Eigen::Vector3d& origin, const Eigen::Vector3d& point, const Eigen::Vector3d& direction, double& projected_uncertainty);
    RelativePosition check_relative_position(const std::shared_ptr<GenericPoint>& generic_point);
    RelativePosition check_relative_position(const std::shared_ptr<Vertex>& vertex);
    RelativePosition check_relative_position(const std::shared_ptr<InteriorPoint>& interior_point);

    void merge_surface(const std::shared_ptr<Surface>& surface);

    const int& get_id() const;
    const std::unordered_set<std::shared_ptr<Vertex>, MeshObjectHash>& get_vertices() const;
    const std::unordered_set<std::shared_ptr<InteriorPoint>, MeshObjectHash>& get_interior_points() const;
    const std::unordered_set<std::shared_ptr<Edge>, MeshObjectHash>& get_edges() const;
    const std::unordered_set<std::shared_ptr<Face>, MeshObjectHash>& get_faces() const;
    const Eigen::Vector3d& get_mean() const;
    const Eigen::Matrix3d& get_covariance() const;
    const Eigen::Matrix3d& get_eigenvectors() const;
    const Eigen::Vector3d& get_eigenvalues() const;
    const Eigen::Vector3d& get_normal() const;
    std::size_t get_total_point_size() const;
    double get_average_distance_travelled() const;
    double get_max_distance_travelled() const;
    const std::tuple<int, int, int>& get_color() const;
    const std::vector<double>& get_point_to_plane_distance_stats();
    const std::vector<double>& get_projective_distance_stats();
    double get_average_projective_distance();
    bool is_expired() const;
    bool is_abnormal();
    bool is_seed() const;

    double get_surface_area() const;

    bool can_merge(const std::shared_ptr<Surface>& surface) const;

    void connect(const std::shared_ptr<Vertex>& vertex);
    void connect(const std::shared_ptr<Edge>& edge);
    void connect(const std::shared_ptr<Face>& face);
    void connect(const std::shared_ptr<InteriorPoint>& interior_point);
    void disconnect(const std::shared_ptr<Vertex>& vertex);
    void disconnect(const std::shared_ptr<Edge>& edge);
    void disconnect(const std::shared_ptr<Face>& face);
    void disconnect(const std::shared_ptr<InteriorPoint>& interior_point);

    std::unordered_set<std::shared_ptr<Vertex>, MeshObjectHash> get_boundary_vertices();
    bool try_close_holes_repeatedly();
    bool try_close_holes();

    void remove_non_manifold_edges();

    void swap(const std::shared_ptr<Vertex>& vertex1, const std::shared_ptr<Vertex>& vertex2);
    void pause_normal_std_update();
    void resume_normal_std_update();

    void set_random_color();

    bool tree_intersect_edge(const std::shared_ptr<Vertex>& vertex0, const std::shared_ptr<Vertex>& vertex1);
    bool connect_by_edges_and_faces(const std::shared_ptr<Vertex>& vertex, const std::unordered_set<std::shared_ptr<Vertex>, MeshObjectHash>& all_nearby_vertices);

    void compute_surface_position_std_in_normal_direction();
    double get_surface_position_std_in_normal_direction();

    bool remove_unmatched_points();
    void remove_singular_components();
    void split_surface_by_connected_components();

    void add_searchable_edge(const std::shared_ptr<Edge>& edge);
    void remove_searchable_edge(const std::shared_ptr<Edge>& edge);
    
    void print_info();

    void set_ith_cloud(unsigned int ith_cloud);
    unsigned int get_ith_cloud() const;

private:
    static Settings settings_;

    bool deleting_ = false;
    bool is_expired_ = true;

    bool update_normal_position_std_ = true;

    EdgeBVH edge_bvh_;

    int id_;
    std::shared_ptr<Storage> storage_;

    std::size_t composition_hash_;

    std::unordered_set<std::shared_ptr<Vertex>, MeshObjectHash> vertices_;
    std::unordered_set<std::shared_ptr<Edge>, MeshObjectHash> edges_;
    std::unordered_set<std::shared_ptr<Face>, MeshObjectHash> faces_;
    std::unordered_set<std::shared_ptr<InteriorPoint>, MeshObjectHash> interior_points_;

    void add_point_to_surface_fitting(const Eigen::Vector3d& point, const Eigen::Vector3d& origin, double distance_travelled, double projection_uncertainty);
    void remove_point_from_surface_fitting(const Eigen::Vector3d& position, const Eigen::Vector3d& origin, double distance_travelled, double projection_uncertainty);
    double weight_ = 0;
    Eigen::Vector3d mean_;
    Eigen::Matrix3d covariance_;
    Eigen::Matrix3d eigenvectors_;
    Eigen::Vector3d eigenvalues_;
    Eigen::Vector3d normal_;
    double characteristic_length_;
    double normal_uncertainty_ = 0;
    double sum_of_average_distance_travelled_;
    double max_distance_travelled_ = 0;
    unsigned int total_point_size_ = 0;

    unsigned int ith_cloud_ = 0;

    Eigen::Matrix3d covariance_mean_;
    double variance_mean_in_normal_direction_;

    bool is_seed_ = true;
    void update_seed_status();

    std::vector<double> stored_projective_distance_stats_;
    std::vector<double> stored_point_to_plane_distance_stats_;
    std::size_t previous_total_point_size_for_projective_;
    std::size_t previous_total_point_size_for_point_to_plane_;

    FIFOCache<std::size_t, double> buffer_surface_position_std_in_normal_direction{std::numeric_limits<std::size_t>::max()};
    
    double previous_normal_distance_;
    double previous_normal_std_;

    std::tuple<int, int, int> color_;

    double surface_area_;
};

bool operator<(const std::shared_ptr<Surface>& lhs, const std::shared_ptr<Surface>& rhs);
bool operator==(const std::shared_ptr<Surface>& lhs, const std::shared_ptr<Surface>& rhs);
bool operator>=(const std::shared_ptr<Surface>& lhs, const std::shared_ptr<Surface>& rhs);
bool operator!=(const std::shared_ptr<Surface>& lhs, const std::shared_ptr<Surface>& rhs);