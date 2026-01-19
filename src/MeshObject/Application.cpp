#define PCL_NO_PRECOMPILE

#include "MeshObject/Application.hpp"
#include "MeshObject/Vertex.hpp"
#include "MeshObject/Edge.hpp"
#include "MeshObject/Face.hpp"
#include "MeshObject/Surface.hpp"
#include "MeshObject/GenericPoint.hpp"
#include "MeshObject/InteriorPoint.hpp"
#include "MeshObject/Storage.hpp"
#include "utilities/utilities.hpp"
#include "utilities/covariance_math.hpp"
#include "utilities/simplified_mesh.hpp"

#include <iostream>
#include <random>
#include <algorithm>
#include <unordered_set>
#include "MeshObject/MeshObject.hpp"
#include "MeshObject/Simulation.hpp"

#include "utilities/omp_utilities.hpp"
#include "MeshObject/RRSTree.hpp"

#include "MeshObject/TriangleBVH.hpp"
#include <pcl/filters/passthrough.h>

#include "point_type/VilensPointT.hpp"
#include "point_type/BagPointT.hpp"

#include <pcl/io/ply_io.h>
#include <boost/filesystem.hpp>

template <typename PointT>
Application<PointT>::Application() 
{
    restart();
}

template <typename PointT>
Eigen::Matrix3d Application<PointT>::merge_covariances_of_surfaces(std::shared_ptr<Surface> surface1, std::shared_ptr<Surface> surface2) 
{
    const Eigen::Matrix3d& cov1 = surface1->get_covariance();
    const Eigen::Matrix3d& cov2 = surface2->get_covariance();
    const Eigen::Vector3d& mean1 = surface1->get_mean();
    const Eigen::Vector3d& mean2 = surface2->get_mean();
    int size1 = surface1->get_total_point_size();
    int size2 = surface2->get_total_point_size();
    return merge_covariance(cov1, cov2, mean1, mean2, size1, size2);
}

// compute eigen value of merged surfaces
template <typename PointT>
double Application<PointT>::compute_eigenvalue_of_merged_surfaces(std::shared_ptr<Surface> surface1, std::shared_ptr<Surface> surface2)
{
    Eigen::Matrix3d covariance_matrix = merge_covariances_of_surfaces(surface1, surface2);
    return Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(covariance_matrix).eigenvalues()[0];
}

// template <typename PointT>
// void Application<PointT>::try_merge_surfaces(std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash>& surfaces_to_merge)
// {
    // while (true) 
    // {
    //     std::set<std::pair<std::shared_ptr<Surface>, std::shared_ptr<Surface>>> surface_pairs;
    //     for (std::shared_ptr<Surface> surface1 : surfaces_to_merge) 
    //     {
    //         for (std::shared_ptr<Surface> surface2 : surfaces_to_merge) 
    //         {
    //             if (surface1 >= surface2) continue;
    //             surface_pairs.insert(std::make_pair(surface1, surface2));
    //         }
    //     }
        
    //     bool again = false;
    //     for (const auto& pairs : surface_pairs) 
    //     {
    //         Eigen::Matrix3d covariance_matrix = merge_covariances_of_surfaces(pairs.first, pairs.second);
    //         double eigenvalue = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(covariance_matrix).eigenvalues()[0];
    //         if (eigenvalue > settings_.merged_eigenvalue_threshold) continue;

    //         surfaces_to_merge.erase(pairs.second);
    //         pairs.first->merge_surface(pairs.second);

    //         again = true;
    //         break;
    //     }
    //     if (!again) break;
    // }
// }


template <typename PointT>
void Application<PointT>::load_point_cloud()
{
    std::cout << data_loader.size() << std::endl;

    // fix ith_cloud
    if (ith_cloud < 0)
    {
        if (settings_.log.load_point_cloud) std::cout << "reached the first pointcloud" << std::endl;
        ith_cloud = 0;
    }
    if (ith_cloud >= data_loader.size())
    {
        if (settings_.log.load_point_cloud) std::cout << "reached the last pointcloud" << std::endl;
        ith_cloud = data_loader.size() - 1;
    }
    
    // get cloud and pose
    typename pcl::PointCloud<PointT>::Ptr pointcloud_local = data_loader.get_cloud(ith_cloud);
    Eigen::Affine3d pose = data_loader.get_pose(ith_cloud);

    // get number of points in the current cloud
    ith_size = pointcloud_local->size();

    // transform cloud to global
    pointcloud = transform_cloud_to_global<PointT>(pointcloud_local, pose);

    // update starting point if first cloud
    if (first_cloud_)
    {
        origin = pose.translation();
        first_cloud_ = false;
    }

    // update origin and distance traveled
    Eigen::Vector3d previous_origin = origin;
    origin = pose.translation();
    distance_travelled_ += (origin - previous_origin).norm();
    storage_->set_distance_travelled(distance_travelled_);
    storage_->set_ith_cloud(ith_cloud);
    std::cout << "distance traveled: " << distance_travelled_ << std::endl;

    // shuffle pointcloud
    if (settings_.shuffle_pointcloud) 
    {
        // Create a random device and generator
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(pointcloud->points.begin(), pointcloud->points.end(), g);
    }

    // log
    if (settings_.log.load_point_cloud) std::cout << "loaded pointcloud " << ith_cloud << " with " << pointcloud->size() << " points" << std::endl;
}

template <typename PointT>
void Application<PointT>::write_mesh()
{
    // update settings
    settings_.point_mode = PointMode::PROJECTED;
    settings_.color_mode = ColorMode::ID;

    // get vertex and faces
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertex_pointcloud = compute_vertex_point_pointcloud(settings_);
    std::unordered_set<std::shared_ptr<Face>, MeshObjectHash> faces = storage_->get_faces();

    // log
    std::cout << "filtering faces by edge length radius" << std::endl;

    // filter face by edge length radius
    std::unordered_set<std::shared_ptr<Face>, MeshObjectHash> filtered_faces;
    unsigned int count = 0;
    unsigned int total = faces.size();
    for (const std::shared_ptr<Face>& face : faces)
    {
        // skip if seed surface
        if (face->get_surface()->is_seed()) continue;

        // log
        count++;
        if (count % 10000 == 0) std::cout << "filtering faces by edge length radius " << count << " of " << total << std::endl;

        // skip if expired
        if (face->is_expired()) continue;

        // get vertices
        std::shared_ptr<Vertex> vertex0 = face->get_vertex(0);
        std::shared_ptr<Vertex> vertex1 = face->get_vertex(1);
        std::shared_ptr<Vertex> vertex2 = face->get_vertex(2);

        // get projected position
        Eigen::Vector3d position0 = vertex0->compute_projected_position();
        Eigen::Vector3d position1 = vertex1->compute_projected_position();
        Eigen::Vector3d position2 = vertex2->compute_projected_position();

        // get projected edge length
        double edge_length0 = (position0 - position1).norm();
        double edge_length1 = (position1 - position2).norm();
        double edge_length2 = (position2 - position0).norm();

        // skip if edge length radius is too long
        if (edge_length0 > vertex0->get_radius() || edge_length0 > vertex1->get_radius()) continue;
        if (edge_length1 > vertex1->get_radius() || edge_length1 > vertex2->get_radius()) continue;
        if (edge_length2 > vertex2->get_radius() || edge_length2 > vertex0->get_radius()) continue;

        // store
        filtered_faces.insert(face);
    }

    // log
    std::cout << "creating triangle mesh" << std::endl;

    // create triangle mesh
    pcl::PolygonMesh triangle_mesh;
    pcl::toPCLPointCloud2(*vertex_pointcloud, triangle_mesh.cloud);
    for (const std::shared_ptr<Face>& face : filtered_faces)
    {
        // skip if can't find all indices
        if (vertex_to_cloud_indices_map.find(face->get_vertex(0)) == vertex_to_cloud_indices_map.end()) continue;
        if (vertex_to_cloud_indices_map.find(face->get_vertex(1)) == vertex_to_cloud_indices_map.end()) continue;
        if (vertex_to_cloud_indices_map.find(face->get_vertex(2)) == vertex_to_cloud_indices_map.end()) continue;

        pcl::Vertices triangle;
        triangle.vertices.push_back(vertex_to_cloud_indices_map.at(face->get_vertex(0)));
        triangle.vertices.push_back(vertex_to_cloud_indices_map.at(face->get_vertex(1)));
        triangle.vertices.push_back(vertex_to_cloud_indices_map.at(face->get_vertex(2)));
        triangle_mesh.polygons.push_back(triangle);
    }

    // create folder for triangle mesh if not exists
    if (!boost::filesystem::exists(settings_.save_folder))
    {
        boost::filesystem::create_directories(settings_.save_folder);
    }

    // save triangle mesh
    std::string write_path = settings_.save_folder + "mesh.ply";
    pcl::io::savePLYFileBinary(write_path, triangle_mesh);
    std::cout << "write mesh to " << write_path << std::endl;

    // save duration to file for triangle mesh
    std::string duration_file_path = settings_.save_folder + "duration.txt";
    std::ofstream duration_file(duration_file_path);
    for (double duration : duration_list)
    {
        duration_file << duration << std::endl;
    }

    // create simplified mesh
    pcl::PolygonMesh simplified_mesh;
    for (const std::shared_ptr<Surface>& surface : storage_->get_surfaces())
    {
        // skip if seed surface
        if (surface->is_seed()) continue;

        pcl::PolygonMesh surface_mesh = create_simplified_mesh(surface);
        merge_polygon_mesh(simplified_mesh, surface_mesh);
    }
    
    // create folder for simplified mesh if not exists
    std::string simplified_folder = settings_.save_folder;
    simplified_folder.insert(simplified_folder.size() - 1, "_simplified_mesh");
    if (!boost::filesystem::exists(simplified_folder))
    {
        boost::filesystem::create_directories(simplified_folder);
    }

    // save simplified mesh
    std::string simplified_mesh_path = simplified_folder + "simplified_mesh.ply";
    pcl::io::savePLYFileBinary(simplified_mesh_path, simplified_mesh);
    std::cout << "write simplified mesh to " << simplified_mesh_path << std::endl;

    // create simplified mesh with color
    pcl::PolygonMesh simplified_mesh_with_color;
    for (const std::shared_ptr<Surface>& surface : storage_->get_surfaces())
    {
        // skip if seed surface
        if (surface->is_seed()) continue;

        pcl::PolygonMesh surface_mesh_with_color = create_simplified_mesh(surface, true);
        merge_polygon_mesh(simplified_mesh_with_color, surface_mesh_with_color);
    }
    
    // save simplified mesh with color
    std::string simplified_mesh_with_color_path = simplified_folder + "simplified_mesh_with_color.ply";
    pcl::io::savePLYFileBinary(simplified_mesh_with_color_path, simplified_mesh_with_color);
    std::cout << "write simplified mesh with color to " << simplified_mesh_with_color_path << std::endl;
    
    // save duration to file for simplified mesh
    std::string simplified_duration_file_path = simplified_folder + "duration.txt";
    std::ofstream simplified_duration_file(simplified_duration_file_path);
    for (double duration : duration_list)
    {
        simplified_duration_file << duration << std::endl;
    }

    // save stack duration to file for simplified mesh
    std::string stack_duration_file_path = simplified_folder + "stack_duration.txt";
    std::ofstream stack_duration_file(stack_duration_file_path);
    for (unsigned int index = 0; index < duration_list.size(); index++)
    {
        double total_duration = duration_list[index];
        double rrs_search_duration = rrs_search_duration_list[index];
        double rrs_update_duration = rrs_update_duration_list[index];
        double bvh_search_duration = bvh_search_duration_list[index];
        double bvh_update_duration = bvh_update_duration_list[index];
        double add_to_map_duration = add_to_map_duration_list[index];
        double delete_from_map_duration = delete_from_map_duration_list[index];
        double relative_position_duration = relative_position_duration_list[index];
        
        const std::string separator = ",";
        stack_duration_file << total_duration << separator
                            << rrs_search_duration << separator
                            << rrs_update_duration << separator
                            << bvh_search_duration << separator
                            << bvh_update_duration << separator
                            << add_to_map_duration << separator
                            << delete_from_map_duration << separator
                            << relative_position_duration << std::endl;
    }
}

template <typename PointT>
void Application<PointT>::process_point(const std::shared_ptr<GenericPoint>& generic_point)
{
    // bvh search duration
    std::chrono::time_point<std::chrono::high_resolution_clock> bvh_search_duration_start = std::chrono::high_resolution_clock::now();

    std::vector<std::shared_ptr<Face>> bvh_results;
    BVHReturnType BVH_return = storage_->face_intersection_search(generic_point, bvh_results);

    std::chrono::time_point<std::chrono::high_resolution_clock> bvh_search_duration_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> bvh_search_duration = bvh_search_duration_end - bvh_search_duration_start;
    bvh_search_duration_per_thread[omp_get_thread_num()] += bvh_search_duration.count();

    // rrs search duration
    std::chrono::time_point<std::chrono::high_resolution_clock> rrs_search_duration_start = std::chrono::high_resolution_clock::now();

    std::vector<std::shared_ptr<Vertex>> rrs_results;
    RRSReturnType RRS_return = storage_->reverse_radius_search(generic_point, rrs_results);    

    std::chrono::time_point<std::chrono::high_resolution_clock> rrs_search_duration_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> rrs_search_duration = rrs_search_duration_end - rrs_search_duration_start;
    rrs_search_duration_per_thread[omp_get_thread_num()] += rrs_search_duration.count();

    // update stats
    num_of_concurrent_processes++;
    accumulated_points++;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_now = std::chrono::high_resolution_clock::now();

    // compute duration
    std::chrono::duration<double> duration_accumulated = t_now - t_init;
    std::chrono::duration<double> duration_instantaneous;
    {
        // lock
        std::unique_lock<std::mutex> lock(t_last_mutex);

        // compute duration    
        duration_instantaneous = t_now - t_last;

        // update timestamp
        t_last = t_now;
    }

    // compute speed
    double speed_accumulated = accumulated_points / duration_accumulated.count();
    double speed_instantaneous = 1.0 / duration_instantaneous.count();
    
    // // output number of concurrent threads that reaches this point
    if (settings_.log.num_of_concurrent_processes)
    {
        std::stringstream ss;
        ss  << " | number of concurrent processes = " << num_of_concurrent_processes
            << " | processing point " << generic_point->get_id()
            << " | by thread " << omp_get_thread_num()
            << " | BVH tree size = " << storage_->get_bvh_size()
            << std::endl;
        std::cout << ss.str();
    }

    if (settings_.log.total_processed_points)
    {
        std::stringstream ss;
        ss  << " | total processed points = " << accumulated_points
            << " | speed accumulated = " << speed_accumulated
            << " | speed instantaneous = " << speed_instantaneous
            << std::endl;
        std::cout << ss.str();
    }
    
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // add to map duration
    std::chrono::time_point<std::chrono::high_resolution_clock> add_to_map_duration_start = std::chrono::high_resolution_clock::now();

    // process
    add_point_to_map(generic_point, bvh_results, rrs_results);

    num_of_concurrent_processes--;
    
    // after unlocking all locks, add the point in queue to the search tree
    storage_->update_vertices_that_have_added_publishers();

    std::chrono::time_point<std::chrono::high_resolution_clock> add_to_map_duration_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> add_to_map_duration = add_to_map_duration_end - add_to_map_duration_start;
    add_to_map_duration_per_thread[omp_get_thread_num()] += add_to_map_duration.count();

    // delete from map duration
    std::chrono::time_point<std::chrono::high_resolution_clock> delete_from_map_duration_start = std::chrono::high_resolution_clock::now();
    
    storage_->delete_to_be_deleted_repeatedly();

    std::chrono::time_point<std::chrono::high_resolution_clock> delete_from_map_duration_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> delete_from_map_duration = delete_from_map_duration_end - delete_from_map_duration_start;
    delete_from_map_duration_per_thread[omp_get_thread_num()] += delete_from_map_duration.count();


    // update rrs tree duration
    std::chrono::time_point<std::chrono::high_resolution_clock> rrs_update_duration_start = std::chrono::high_resolution_clock::now();

    storage_->update_vertices_that_have_changed_box();
    storage_->add_or_remove_vertices_from_rrs_tree();

    std::chrono::time_point<std::chrono::high_resolution_clock> rrs_update_duration_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> rrs_update_duration = rrs_update_duration_end - rrs_update_duration_start;
    rrs_update_duration_per_thread[omp_get_thread_num()] += rrs_update_duration.count();

    // update bvh tree duration
    std::chrono::time_point<std::chrono::high_resolution_clock> bvh_update_duration_start = std::chrono::high_resolution_clock::now();
    storage_->add_or_remove_faces_from_bvh_tree();

    std::chrono::time_point<std::chrono::high_resolution_clock> bvh_update_duration_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> bvh_update_duration = bvh_update_duration_end - bvh_update_duration_start;
    bvh_update_duration_per_thread[omp_get_thread_num()] += bvh_update_duration.count();

    // update edge bvh tree duration
    std::chrono::time_point<std::chrono::high_resolution_clock> add_to_map_duration_start2 = std::chrono::high_resolution_clock::now();

    storage_->add_or_remove_edges_from_edgeBVH_tree();

    std::chrono::time_point<std::chrono::high_resolution_clock> add_to_map_duration_end2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> add_to_map_duration2 = add_to_map_duration_end2 - add_to_map_duration_start2;
    add_to_map_duration_per_thread[omp_get_thread_num()] += add_to_map_duration2.count();
}

template <typename PointT>
void Application<PointT>::add_point_to_map(const std::shared_ptr<GenericPoint>& generic_point, std::vector<std::shared_ptr<Face>> unfiltered_bvh_results, std::vector<std::shared_ptr<Vertex>> unfiltered_rrs_results)
{

    // from bvh results and rrs results, get surfaces
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> bvh_surfaces;
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> rrs_surfaces;
    std::unordered_set<std::shared_ptr<Face>, MeshObjectHash> bvh_results;
    std::unordered_set<std::shared_ptr<Vertex>, MeshObjectHash> rrs_results;
    for (const std::shared_ptr<Face>& face : unfiltered_bvh_results)
    {
        // skip if nullptr
        if (face == nullptr) continue;

        // read lock
        std::shared_lock<std::shared_mutex> lock(face->rwlock_lifecycle_);

        // skip if expired
        if (face->is_expired()) continue;

        // skip if does not intersect
        if (!face->intersects_point(generic_point->get_origin(), generic_point->get_direction())) continue;
        
        // store
        bvh_surfaces.insert(face->get_surface());
        bvh_results.insert(face);
    }
    for (const std::shared_ptr<Vertex>& vertex : unfiltered_rrs_results)
    {
        // skip if nullptr
        if (vertex == nullptr) continue;

        // read lock
        std::shared_lock<std::shared_mutex> lock(vertex->rwlock_lifecycle_); // this to prevent vertex from being deleted

        // skip if expired
        if (vertex->is_expired()) continue;

        // skip if does not contain
        if (!vertex->contains(generic_point->get_position())) continue;

        // store
        rrs_surfaces.insert(vertex->get_surface());
        rrs_results.insert(vertex);
    }

    // split surfaces into bvh and rrs seed, within, behind, in front
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> surfaces_bvh_seed;
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> surfaces_bvh_within;
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> surfaces_bvh_behind;
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> surfaces_bvh_in_front;
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> surfaces_rrs_seed;
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> surfaces_rrs_within;
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> surfaces_rrs_behind;
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> surfaces_rrs_in_front;
    for (const std::shared_ptr<Surface>& surface : bvh_surfaces)
    {
        // read lock
        std::shared_lock<std::shared_mutex> lock(surface->rwlock_lifecycle_);

        // skip if the surface is expired
        if (surface->is_expired()) continue;

        // check relative position
        // time this
        std::chrono::time_point<std::chrono::high_resolution_clock> relative_position_start = std::chrono::high_resolution_clock::now();
        RelativePosition relative_position = surface->check_relative_position(generic_point);
        std::chrono::time_point<std::chrono::high_resolution_clock> relative_position_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> relative_position_duration = relative_position_end - relative_position_start;
        relative_position_duration_per_thread[omp_get_thread_num()] += relative_position_duration.count();

        // skip if no relative position
        if (relative_position == RelativePosition::NO_RELATIVE_POSITION) surfaces_bvh_seed.insert(surface);

        // add to within
        if (relative_position == RelativePosition::WITHIN) surfaces_bvh_within.insert(surface);

        // add to behind
        if (relative_position == RelativePosition::BEHIND) surfaces_bvh_behind.insert(surface);

        // add to in front
        if (relative_position == RelativePosition::IN_FRONT) surfaces_bvh_in_front.insert(surface);
    }
    for (const std::shared_ptr<Surface>& surface : rrs_surfaces)
    {
        // read lock
        std::shared_lock<std::shared_mutex> lock(surface->rwlock_lifecycle_);

        // skip if the surface is expired
        if (surface->is_expired()) continue;

        // check relative position
        std::chrono::time_point<std::chrono::high_resolution_clock> relative_position_start = std::chrono::high_resolution_clock::now();
        RelativePosition relative_position = surface->check_relative_position(generic_point);
        std::chrono::time_point<std::chrono::high_resolution_clock> relative_position_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> relative_position_duration = relative_position_end - relative_position_start;
        relative_position_duration_per_thread[omp_get_thread_num()] += relative_position_duration.count();

        // skip if no relative position
        if (relative_position == RelativePosition::NO_RELATIVE_POSITION) surfaces_rrs_seed.insert(surface);

        // add to within
        if (relative_position == RelativePosition::WITHIN) surfaces_rrs_within.insert(surface);

        // add to behind
        if (relative_position == RelativePosition::BEHIND) surfaces_rrs_behind.insert(surface);

        // add to in front
        if (relative_position == RelativePosition::IN_FRONT) surfaces_rrs_in_front.insert(surface);
    }

    // construct a list of surfaces to add to
    std::vector<std::shared_ptr<Surface>> list_of_surfaces_to_add_to;

    // 1. surfaces_bvh_within
    if (surfaces_bvh_within.size() > 0)
    {
        // create vector of surface paired with size
        std::vector<std::pair<std::shared_ptr<Surface>, double>> surfaces_bvh_within_sorted;
        for (const std::shared_ptr<Surface>& surface : surfaces_bvh_within)
        {
            // read lock
            std::shared_lock<std::shared_mutex> lock(surface->rwlock_lifecycle_);

            // skip if nullptr
            if (surface == nullptr) continue;

            // skip if the surface is expired
            if (surface->is_expired()) continue;

            // add to list
            surfaces_bvh_within_sorted.emplace_back(surface, surface->get_total_point_size());
        }

        // sort by surface size
        std::sort(surfaces_bvh_within_sorted.begin(), surfaces_bvh_within_sorted.end(), 
            [](const std::pair<std::shared_ptr<Surface>, double>& a, const std::pair<std::shared_ptr<Surface>, double>& b) 
            {
                // sort by surface size
                return a.second > b.second; 
            });

        // add to list
        for (const std::pair<std::shared_ptr<Surface>, double>& surface : surfaces_bvh_within_sorted)
        {
            list_of_surfaces_to_add_to.push_back(surface.first);
        }
    }

    // 2. surfaces_rrs_within
    if (surfaces_rrs_within.size() > 0)
    {
        // create vector of surface paired with size
        std::vector<std::pair<std::shared_ptr<Surface>, double>> surfaces_rrs_within_sorted;
        for (const std::shared_ptr<Surface>& surface : surfaces_rrs_within)
        {
            // read lock
            std::shared_lock<std::shared_mutex> lock(surface->rwlock_lifecycle_);

            // skip if nullptr
            if (surface == nullptr) continue;

            // skip if the surface is expired
            if (surface->is_expired()) continue;

            // add to list
            surfaces_rrs_within_sorted.emplace_back(surface, surface->get_total_point_size());
        }

        // sort by surface size
        std::sort(surfaces_rrs_within_sorted.begin(), surfaces_rrs_within_sorted.end(), 
            [](const std::pair<std::shared_ptr<Surface>, double>& a, const std::pair<std::shared_ptr<Surface>, double>& b)
            { 
                // sort by surface size
                return a.second > b.second;
            });

        // add to list
        for (const std::pair<std::shared_ptr<Surface>, double>& surface : surfaces_rrs_within_sorted)
        {
            list_of_surfaces_to_add_to.push_back(surface.first);
        }
    }

    // 3. surfaces_rrs_seed
    if (surfaces_rrs_seed.size() > 0)
    {        
        // sort by surface closeness
        std::unordered_map<std::shared_ptr<Surface>, double, MeshObjectHash> surfaces_rrs_seed_distances;
        for (std::shared_ptr<Surface> surface : surfaces_rrs_seed)
        {
            // read lock
            std::shared_lock<std::shared_mutex> lock(surface->rwlock_lifecycle_);

            // skip if the surface is expired
            if (surface->is_expired()) continue;

            for (std::shared_ptr<Vertex> vertex : rrs_results)
            {
                // read lock
                std::shared_lock<std::shared_mutex> lock(vertex->rwlock_lifecycle_); // this to prevent vertex from being deleted

                // skip if the vertex is expired
                if (vertex->is_expired()) continue;

                // skip if the vertex is not in surfaces_rrs_seed
                if (vertex->get_surface() != surface) continue;

                // compute distance
                double distance_vertex = (vertex->get_position() - generic_point->get_position()).norm();
                if (distance_vertex < surfaces_rrs_seed_distances[surface])
                {
                    surfaces_rrs_seed_distances[surface] = distance_vertex;
                }
            }
        }
        std::vector<std::shared_ptr<Surface>> surfaces_rrs_seed_sorted(surfaces_rrs_seed.begin(), surfaces_rrs_seed.end());
        std::sort(surfaces_rrs_seed_sorted.begin(), surfaces_rrs_seed_sorted.end(), 
            [&surfaces_rrs_seed_distances](const std::shared_ptr<Surface>& a, const std::shared_ptr<Surface>& b) 
            { 
                // sort by surface distance
                return surfaces_rrs_seed_distances[a] < surfaces_rrs_seed_distances[b]; 
            });

        // add to list
        list_of_surfaces_to_add_to.insert(list_of_surfaces_to_add_to.end(), surfaces_rrs_seed_sorted.begin(), surfaces_rrs_seed_sorted.end());
    }

    // add to surface according to the list of bvh and rrs surfaces
    std::shared_ptr<Surface> surface_to_add_to = nullptr;
    std::shared_ptr<InteriorPoint> new_interior_point = nullptr;
    std::shared_ptr<Vertex> new_vertex = nullptr;
    for (std::shared_ptr<Surface> surface : list_of_surfaces_to_add_to)
    {
        // read lock
        std::shared_lock<std::shared_mutex> lock(surface->rwlock_lifecycle_);

        // skip if the surface is expired
        if (surface->is_expired()) continue;

        // store current surface to add to
        surface_to_add_to = surface;

        // 1. if added to bvh within
        if (surfaces_bvh_within.find(surface_to_add_to) != surfaces_bvh_within.end())
        {
            // get the first face
            for (const std::shared_ptr<Face>& face : bvh_results)
            {
                // read lock
                std::shared_lock<std::shared_mutex> lock(face->rwlock_lifecycle_);

                // skip if the face is expired
                if (face->is_expired()) continue;

                // skip if the face is not from the surface
                if (face->get_surface() != surface_to_add_to) continue;

                // create interior point
                new_interior_point = storage_->add_interior_point(surface_to_add_to, face, generic_point);

                break;
            }

            // return
            break;
        }
        
        // 2. if added to rrs within
        if (surfaces_rrs_within.find(surface_to_add_to) != surfaces_rrs_within.end())
        {
            // added as vertex
            new_vertex = storage_->add_vertex(surface_to_add_to, generic_point);

            // reduce radius of the new vertex
            for (std::shared_ptr<Vertex> vertex : rrs_results)
            {
                // read lock
                std::shared_lock<std::shared_mutex> lock(vertex->rwlock_lifecycle_); // this to prevent vertex from being deleted

                // skip if the vertex is expired
                if (vertex->is_expired()) continue;

                // skip if the vertex is the same as the new vertex
                if (vertex->get_surface() == surface_to_add_to) continue;

                // skip if the vertex is in seed surface
                if (surfaces_rrs_seed.find(vertex->get_surface()) != surfaces_rrs_seed.end()) continue;

                // add neighboring vertex
                new_vertex->add_vertex_point_distance_publisher(vertex);
            }
            new_vertex->upon_adding_publisher();

            // need to prevent vertex from being deleted from newly connected edges
            new_vertex->set_connecting_to_edges_and_faces(true);
            const bool connected = surface_to_add_to->connect_by_edges_and_faces(new_vertex, rrs_results);
            new_vertex->set_connecting_to_edges_and_faces(false);

            // if not connected, delete the vertex
            if (!connected)
            {
                // delete this and retry
                new_vertex->set_do_not_add_back_due_to_not_connected(true);
                storage_->delete_vertex(new_vertex);
                continue;
            }
            else
            {
                break;
            }
        }

        // 3. if added to rrs seed
        if (surfaces_rrs_seed.find(surface_to_add_to) != surfaces_rrs_seed.end())
        {
            // added as vertex
            new_vertex = storage_->add_vertex(surface_to_add_to, generic_point);

            // reduce radius of the new vertex
            for (std::shared_ptr<Vertex> vertex : rrs_results)
            {
                // read lock
                std::shared_lock<std::shared_mutex> lock(vertex->rwlock_lifecycle_); // this to prevent vertex from being deleted

                // skip if the vertex is expired
                if (vertex->is_expired()) continue;

                // skip if the vertex is the same as the new vertex
                if (vertex->get_surface() == surface_to_add_to) continue;

                // add neighboring vertex
                new_vertex->add_vertex_point_distance_publisher(vertex);
            }
            new_vertex->upon_adding_publisher();

            // need to prevent vertex from being deleted from newly connected edges
            new_vertex->set_connecting_to_edges_and_faces(true);
            const bool connected = surface_to_add_to->connect_by_edges_and_faces(new_vertex, rrs_results);
            new_vertex->set_connecting_to_edges_and_faces(false);

            // if not connected, delete the vertex
            if (!connected)
            {
                // delete this and retry
                new_vertex->set_do_not_add_back_due_to_not_connected(true);
                storage_->delete_vertex(new_vertex);
                continue;
            }
            else
            {
                break;
            }
        }
    }

    // 4. add to new surface if not added to list of bvh or rrs
    if (surface_to_add_to == nullptr)
    {
        // add new surface
        surface_to_add_to = storage_->add_surface();

        // add as vertex
        new_vertex = storage_->add_vertex(surface_to_add_to, generic_point);

        // reduce radius of the new vertex
        for (std::shared_ptr<Vertex> vertex : rrs_results)
        {
            // read lock
            std::shared_lock<std::shared_mutex> lock(vertex->rwlock_lifecycle_); // this to prevent vertex from being deleted

            // skip if the vertex is expired
            if (vertex->is_expired()) continue;

            // skip if the vertex is the same as the new vertex
            if (vertex->get_surface() == surface_to_add_to) continue;

            // add neighboring vertex
            new_vertex->add_vertex_point_distance_publisher(vertex);
        }
        new_vertex->upon_adding_publisher();
    }

    surface_to_add_to->set_ith_cloud(ith_cloud);

    // 1. if added to bvh within / 2. if added to rrs within
    if (surfaces_bvh_within.find(surface_to_add_to) != surfaces_bvh_within.end() || surfaces_rrs_within.find(surface_to_add_to) != surfaces_rrs_within.end())
    {
        // bvh - delete penetrated 
        for (const std::shared_ptr<Face>& face : bvh_results)
        {
            {
                // read lock
                std::shared_lock<std::shared_mutex> lock(face->rwlock_lifecycle_);

                // skip if expired
                if (face->is_expired()) continue;

                // skip if same surface
                if (face->get_surface() == surface_to_add_to) continue;

                // skip if no relative position
                if (surfaces_bvh_seed.find(face->get_surface()) != surfaces_bvh_seed.end()) continue;

                // skip if in front 
                if (surfaces_bvh_in_front.find(face->get_surface()) != surfaces_bvh_in_front.end()) continue;
            }

            // delete penetrated face
            storage_->add_face_to_be_deleted(face);
        }
        
        // rrs - reduce radius
        for (const std::shared_ptr<Vertex>& vertex : rrs_results)
        {
            // read lock
            std::shared_lock<std::shared_mutex> lock(vertex->rwlock_lifecycle_); // this to prevent vertex from being deleted

            // skip if expired
            if (vertex->is_expired()) continue;

            // skip if same surface
            if (vertex->get_surface() == surface_to_add_to) continue;

            // reduce radius of nearby vertices
            if (new_interior_point) new_interior_point->add_interior_point_distance_subscriber(vertex);
            if (new_vertex) new_vertex->add_vertex_point_distance_subscriber(vertex);

            // add to list that have added publishers
            storage_->add_vertex_that_have_added_publishers(vertex);
        }
    }
    // 3. if added to rrs seed / 4. if added to new surface
    else
    {        
        // rrs - reduce radius of seed vertices only
        for (const std::shared_ptr<Vertex>& vertex : rrs_results)
        {
            // read lock
            std::shared_lock<std::shared_mutex> lock(vertex->rwlock_lifecycle_); // this to prevent vertex from being deleted

            // skip if expired
            if (vertex->is_expired()) continue;

            // skip if same surface
            if (vertex->get_surface() == surface_to_add_to) continue;

            // skip if vertex is not in seed surface
            if (surfaces_rrs_seed.find(vertex->get_surface()) == surfaces_rrs_seed.end()) continue;

            // reduce radius of nearby seed vertices
            new_vertex->add_vertex_point_distance_subscriber(vertex);

            // add to list that have added publishers
            storage_->add_vertex_that_have_added_publishers(vertex);
        }
    }
}

template <typename PointT>
void Application<PointT>::get_lidar_data(Eigen::Vector3d& origin, Eigen::Vector3d& position)
{
    origin = this->origin;
    position = pointcloud->points[ith_point].getVector3fMap().template cast<double>();
    ith_point += settings_.process_every_n_points;

    if (ith_point >= ith_size)
    {
        ith_cloud += 1;
        ith_point = 0;
        load_point_cloud();
    }
}

template <typename PointT>
void Application<PointT>::get_sim_data(Eigen::Vector3d& origin, Eigen::Vector3d& position)
{
    Simulation sim;
    sim.set_object(settings_.sim_object);
    sim.set_noise(settings_.range_precision, settings_.range_accuracy);
    sim.get_data_pair(origin, position);
}

template <typename PointT>
void Application<PointT>::step()
{        
    Eigen::Vector3d thisPointVEC;
    Eigen::Vector3d thisPointOriginVEC;
    if (settings_.use_sim_data)
    {
        get_sim_data(thisPointOriginVEC, thisPointVEC);
    }
    else
    {
        get_lidar_data(thisPointOriginVEC, thisPointVEC);
    }

    // log
    if (settings_.log.step) std::cout << "==================================================================== Processing point " << ith_point << " of cloud " << ith_cloud << std::endl;


    // initialize vector to store duration
    rrs_search_duration_per_thread.resize(settings_.num_threads);
    rrs_update_duration_per_thread.resize(settings_.num_threads);
    bvh_search_duration_per_thread.resize(settings_.num_threads);
    bvh_update_duration_per_thread.resize(settings_.num_threads);
    add_to_map_duration_per_thread.resize(settings_.num_threads);
    delete_from_map_duration_per_thread.resize(settings_.num_threads);
    relative_position_duration_per_thread.resize(settings_.num_threads);

    std::shared_ptr<GenericPoint> generic_point = std::make_shared<GenericPoint>();
    generic_point->initialize_(storage_, thisPointVEC, thisPointOriginVEC, distance_travelled_);
    process_point(generic_point);
}

template <typename PointT>
void Application<PointT>::loop()
{
    // add all points from the cloud to the storage's processing queue
    for (std::size_t i = 0; i < ith_size; i++)
    {
        Eigen::Vector3d thisPointVEC = pointcloud->points[i].getVector3fMap().template cast<double>();
        Eigen::Vector3d thisPointOriginVEC = this->origin;

        storage_->add_to_main_queue(thisPointVEC, thisPointOriginVEC, distance_travelled_);
    }

    // split the queue into smaller queues
    storage_->split_main_queue_into_smaller_queues();

    // initialize vector to store duration
    rrs_search_duration_per_thread.resize(settings_.num_threads);
    rrs_update_duration_per_thread.resize(settings_.num_threads);
    bvh_search_duration_per_thread.resize(settings_.num_threads);
    bvh_update_duration_per_thread.resize(settings_.num_threads);
    add_to_map_duration_per_thread.resize(settings_.num_threads);
    delete_from_map_duration_per_thread.resize(settings_.num_threads);
    relative_position_duration_per_thread.resize(settings_.num_threads);

    // reset stats
    t_init = std::chrono::high_resolution_clock::now();
    accumulated_points = 0;

    // process all points in the queue
    unsigned int num_iteration = 0;
    #pragma omp parallel num_threads(settings_.num_threads)
    {
        while (true)
        {
            std::shared_ptr<GenericPoint> generic_point = nullptr;
            unsigned int total_points_in_queue = 0;

            total_points_in_queue = storage_->get_queue_size();
            if (total_points_in_queue > 0)
            {
                generic_point = storage_->pop_from_queue();
                if (settings_.log.step) 
                {
                    std::stringstream ss;
                    ss << " | remaining point " << total_points_in_queue << " | Processing point " << generic_point->get_id() << " | by thread " << omp_get_thread_num() << std::endl;
                    std::cout << ss.str();
                }
            }

            // If the queue is empty, break the loop
            if (!generic_point) break;

            // Process the point (outside the critical section)
            if (generic_point)
            {
                process_point(generic_point);
            }
        }

        // clean up surfaces
        storage_->cleanup_surfaces(); // split by surface id modulus num_threads
    }
    storage_->add_points_in_smaller_repeated_queues_to_main_queue();

    num_iteration ++;

    std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = t_end - t_init;
    total_duration += duration.count();
    total_loops += 1;
    std::cout << "==================================================================== Processed " << accumulated_points << " points in " << duration.count() << " s, " << "average duration: " << total_duration / total_loops << std::endl;

    // store duration into list
    duration_list.push_back(duration.count());

    // average duration per thread
    double rrs_search_duration = std::accumulate(rrs_search_duration_per_thread.begin(), rrs_search_duration_per_thread.end(), 0.0) / settings_.num_threads;
    double rrs_update_duration = std::accumulate(rrs_update_duration_per_thread.begin(), rrs_update_duration_per_thread.end(), 0.0) / settings_.num_threads;
    double bvh_search_duration = std::accumulate(bvh_search_duration_per_thread.begin(), bvh_search_duration_per_thread.end(), 0.0) / settings_.num_threads;
    double bvh_update_duration = std::accumulate(bvh_update_duration_per_thread.begin(), bvh_update_duration_per_thread.end(), 0.0) / settings_.num_threads;
    double add_to_map_duration = std::accumulate(add_to_map_duration_per_thread.begin(), add_to_map_duration_per_thread.end(), 0.0) / settings_.num_threads;
    double delete_from_map_duration = std::accumulate(delete_from_map_duration_per_thread.begin(), delete_from_map_duration_per_thread.end(), 0.0) / settings_.num_threads;
    double relative_position_duration = std::accumulate(relative_position_duration_per_thread.begin(), relative_position_duration_per_thread.end(), 0.0) / settings_.num_threads;

    rrs_search_duration_per_thread.clear();
    rrs_update_duration_per_thread.clear();
    bvh_search_duration_per_thread.clear();
    bvh_update_duration_per_thread.clear();
    add_to_map_duration_per_thread.clear();
    delete_from_map_duration_per_thread.clear();
    relative_position_duration_per_thread.clear();

    rrs_search_duration_list.push_back(rrs_search_duration);
    rrs_update_duration_list.push_back(rrs_update_duration);
    bvh_search_duration_list.push_back(bvh_search_duration);
    bvh_update_duration_list.push_back(bvh_update_duration);
    add_to_map_duration_list.push_back(add_to_map_duration);
    delete_from_map_duration_list.push_back(delete_from_map_duration);
    relative_position_duration_list.push_back(relative_position_duration);

    // print
    std::cout << "total duration: " << duration.count() << " = " << rrs_search_duration << " + " << rrs_update_duration << " + " << bvh_search_duration << " + " << bvh_update_duration << " + " << add_to_map_duration << " + " << delete_from_map_duration << " + " << relative_position_duration << std::endl;

    // print size of rrs, vertices, and boundary vertices
    std::cout << "rrs size: " << storage_->get_rrs_size() << " | vertices size: " << storage_->get_vertices_size() << " | total point size: " << storage_->get_vertices_size() + storage_->get_interior_points_size() << std::endl;
    // print size of bvh, faces
    std::cout << "bvh size: " << storage_->get_bvh_size() << " | faces size: " << storage_->get_faces_size() << std::endl;

    // store time and ith_cloud into file
    if (settings_.output_time)
    {
        std::ofstream file;
        file.open(settings_.output_file_name, std::ios_base::app);
        file << ith_cloud << " " << duration.count() << std::endl;
        file.close();
    }

    // print repeated queue size
    if (settings_.log.step) std::cout << "==================================================================== repeated queue size: " << storage_->get_repeated_queue_size() << std::endl;

    // load next cloud
    ith_cloud += 1;
    load_point_cloud();
}

template <typename PointT>
void Application<PointT>::process_the_rest()
{
    for (int i = ith_cloud; i < data_loader.size() - 1; i++)
    {
        loop();
    }
}

template <typename PointT>
void Application<PointT>::restart()
{
    // reset objects
    storage_ = std::make_shared<Storage>();
    data_loader.load_dataset(settings_.data_loader_settings);

    // reset state
    first_cloud_ = true;
    ith_cloud = settings_.data_loader_settings.start_cloud;
    ith_point = 0;
    ith_size = 0;
    origin = Eigen::Vector3d(0, 0, 0);
    distance_travelled_ = 0;

    total_duration = 0;
    total_loops = 0;
    
    // load point cloud
    load_point_cloud();
}

template <typename PointT>
std::shared_ptr<Storage> Application<PointT>::get_storage()
{
    return storage_;
}

template <typename PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Application<PointT>::compute_generic_point_pointcloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const std::shared_ptr<GenericPoint>& generic_point : storage_->get_generic_points())
    {
        pcl::PointXYZRGB point;
        point.x = generic_point->get_position()[0];
        point.y = generic_point->get_position()[1];
        point.z = generic_point->get_position()[2];
        point.r = 255;
        point.g = 0;
        point.b = 0;
        cloud->push_back(point);
    }
    return cloud;
}

template <typename PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Application<PointT>::compute_interior_point_pointcloud(const Settings& settings)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const std::shared_ptr<InteriorPoint>& interior_point : storage_->get_interior_points())
    {        
        // skip if from seed surface
        if (!settings.show_seed_surface && interior_point->get_surface()->is_seed()) continue;

        pcl::PointXYZRGB point;
        if (settings.point_mode == PointMode::PROJECTED)
        {
            point.x = interior_point->compute_projected_position()[0];
            point.y = interior_point->compute_projected_position()[1];
            point.z = interior_point->compute_projected_position()[2];
        }
        else if (settings.point_mode == PointMode::ORIGINAL)
        {
            point.x = interior_point->get_original_position()[0];
            point.y = interior_point->get_original_position()[1];
            point.z = interior_point->get_original_position()[2];
        }
        else if (settings.point_mode == PointMode::USED)
        {
            point.x = interior_point->get_position()[0];
            point.y = interior_point->get_position()[1];
            point.z = interior_point->get_position()[2];
        }
        if (settings.color_mode == ColorMode::ID)
        {
            const std::tuple<int, int, int>& color = interior_point->get_surface()->get_color();
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (settings.color_mode == ColorMode::POSITIONAL_UNCERTAINTY)
        {
            double distance = std::fabs(interior_point->compute_projected_distance() / 0.05);
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (settings.color_mode == ColorMode::POSITIONAL_UNCERTAINTY_NORMALIZED)
        {
            // print
            std::cout << "uncertainty: " << interior_point->get_projected_uncertainty() << std::endl;

            double distance = std::abs(interior_point->compute_projected_distance()) / interior_point->get_projected_uncertainty() / 3.f;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (settings.color_mode == ColorMode::PROJECTED_UNCERTAINTY)
        {
            double distance = interior_point->get_projected_uncertainty() / 0.1f;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (settings.color_mode == ColorMode::RADIUS)
        {
            double ratio = interior_point->get_radius() / settings.radius_denominator;
            std::tuple<int, int, int> color = valueToJet(1.0-ratio);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (settings.color_mode == ColorMode::SURFACE_UNCERTAINTY)
        {
            double distance = interior_point->get_surface()->get_surface_position_std_in_normal_direction() / settings.positional_uncertainty_denominator;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (settings.color_mode == ColorMode::MAX_DISTANCE_TRAVELLED)
        {
            double distance = interior_point->get_surface()->get_max_distance_travelled() / 20.f;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (settings.color_mode == ColorMode::DISTANCE_TRAVELLED)
        {
            double distance = interior_point->get_distance_travelled() / 20.f;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (settings.color_mode == ColorMode::WEIGHT)
        {
            // the lowest uncertainty is 0.02 which is the range precision
            // 1/(0.02*0.02) = 2500
            // the best weight is 2500
            // all weight should be lower than this number
            const double best_weight = 1.f / (settings.range_precision * settings.range_precision);
            double distance = interior_point->weight_ / best_weight; 
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        cloud->push_back(point);
    }
    return cloud;
}

template <typename PointT>
const std::unordered_set<std::shared_ptr<Face>, MeshObjectHash>& Application<PointT>::get_faces() 
{
    return storage_->get_faces();
}

template <typename PointT>
const std::unordered_set<std::shared_ptr<Edge>, MeshObjectHash>& Application<PointT>::get_edges() 
{
    return storage_->get_edges();
}

template <typename PointT>
std::vector<std::shared_ptr<Vertex>> Application<PointT>::get_rrs_vertices() 
{
    return storage_->get_rrs_vertices();
}

template <typename PointT>
void Application<PointT>::refine_surfaces()
{
    std::unordered_set<std::shared_ptr<Surface>, MeshObjectHash> copy_of_surfaces = storage_->get_surfaces();
    for (const std::shared_ptr<Surface>& surface : copy_of_surfaces)
    {
        surface->remove_unmatched_points();
        surface->remove_singular_components();
        surface->split_surface_by_connected_components();
    }
    if (settings_.log.refine_surfaces) std::cout << "number of generic points after refine: " << storage_->get_generic_points().size() << std::endl;
}

template <typename PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Application<PointT>::compute_vertex_point_pointcloud(const Settings& setting)
{
    vertex_to_cloud_indices_map.clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const std::shared_ptr<Vertex>& vertex : storage_->get_vertices_ref())
    {
        // skip if internal
        if (!setting.show_internal_vertices && !vertex->is_boundary()) continue;

        // skip if seed surface
        if (!setting.show_seed_surface && vertex->get_surface()->is_seed()) continue;

        pcl::PointXYZRGB point;
        if (setting.point_mode == PointMode::PROJECTED)
        {
            point.x = vertex->compute_projected_position()[0];
            point.y = vertex->compute_projected_position()[1];
            point.z = vertex->compute_projected_position()[2];
        }
        else if (setting.point_mode == PointMode::ORIGINAL)
        {
            point.x = vertex->get_original_position()[0];
            point.y = vertex->get_original_position()[1];
            point.z = vertex->get_original_position()[2];
        }
        else if (setting.point_mode == PointMode::USED)
        {
            point.x = vertex->get_position()[0];
            point.y = vertex->get_position()[1];
            point.z = vertex->get_position()[2];
        }
        if (setting.color_mode == ColorMode::ID)
        {
            const std::tuple<int, int, int>& color = vertex->get_surface()->get_color();
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (setting.color_mode == ColorMode::POSITIONAL_UNCERTAINTY)
        {
            double distance = std::abs(vertex->compute_projected_distance() / 0.05);
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (setting.color_mode == ColorMode::POSITIONAL_UNCERTAINTY_NORMALIZED)
        {
            double distance = std::abs(vertex->compute_projected_distance()) / vertex->get_projected_uncertainty() / 3.f;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (setting.color_mode == ColorMode::PROJECTED_UNCERTAINTY)
        {
            double distance = vertex->get_projected_uncertainty() / 0.1f;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (setting.color_mode == ColorMode::RADIUS)
        {
            double ratio = vertex->get_radius() / setting.radius_denominator;
            std::tuple<int, int, int> color = valueToJet(1.0-ratio);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (setting.color_mode == ColorMode::SURFACE_UNCERTAINTY)
        {
            double distance = vertex->get_surface()->get_surface_position_std_in_normal_direction() / setting.positional_uncertainty_denominator;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (setting.color_mode == ColorMode::MAX_DISTANCE_TRAVELLED)
        {
            double distance = vertex->get_surface()->get_max_distance_travelled() / 20.f;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (setting.color_mode == ColorMode::DISTANCE_TRAVELLED)
        {
            double distance = vertex->get_distance_travelled() / 20.f;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        else if (setting.color_mode == ColorMode::WEIGHT)
        {
            const double best_weight = 1.f / (settings_.range_precision * settings_.range_precision);
            double distance = vertex->weight_ / best_weight;
            std::tuple<int, int, int> color = valueToJet(distance);
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }
        cloud->push_back(point);
        vertex->index_in_cloud_ = cloud->size() - 1;
    }
    return cloud;
}

template <typename PointT>
void Application<PointT>::change_color()
{
    for (const std::shared_ptr<Surface>& surface : storage_->get_surfaces())
    {
        surface->set_random_color();
    }
}

template class Application<VilensPointT>;
template class Application<BagPointT>;