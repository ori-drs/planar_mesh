#include "MeshObject/Settings.hpp"
#include <map>
#include <math.h>

struct DatasetParameters
{
    // path
    std::string pcd_file_folder;
    std::string pose_file_path;

    // lidar parameters
    double range_precision = 0.01;
    double range_accuracy = 0.03;
    bool remove_double_return_flag = false;
    bool filter_low_intensity_flag = false;

    // related parameters
    double radius_value = 1;
    double extra_radius = 0.1;

    // radius ratio
    double radius_ratio = 0.02;
};

Settings::Settings() 
{
    // application settings
    std::map<std::string, DatasetParameters> dataset_map;

    dataset_map["osney"] = DatasetParameters();
    dataset_map["osney"].pcd_file_folder = "/home/jiahao/datasets/osney power station/2024-03-26_13-47-27_rec004_osney_power_station/slam_clouds/";
    dataset_map["osney"].pose_file_path = "/home/jiahao/datasets/osney power station/2024-03-26_13-47-27_rec004_osney_power_station/slam_pose_graph.slam";

    dataset_map["blenheim"] = DatasetParameters();
    dataset_map["blenheim"].pcd_file_folder = "/home/jiahao/datasets/2024-03-14-09-09-02-lenord-walk-for-lintong/individual_clouds/";
    dataset_map["blenheim"].pose_file_path = "/home/jiahao/datasets/2024-03-14-09-09-02-lenord-walk-for-lintong/slam_pose_graph.g2o";
    dataset_map["blenheim"].remove_double_return_flag = true;

    dataset_map["math"] = DatasetParameters();
    dataset_map["math"].pcd_file_folder = "/home/jiahao/datasets/math/individual_clouds/";
    dataset_map["math"].pose_file_path = "/home/jiahao/datasets/math/slam_pose_graph.g2o";
    dataset_map["math"].remove_double_return_flag = true;

    dataset_map["nottinghill"] = DatasetParameters();
    dataset_map["nottinghill"].pcd_file_folder = "/home/jiahao/datasets/2024-10-23_14-40-02_rec004_nottinghill-rad101/slam_clouds/";
    dataset_map["nottinghill"].pose_file_path = "/home/jiahao/datasets/2024-10-23_14-40-02_rec004_nottinghill-rad101/slam_pose_graph.slam";

    dataset_map["office"] = DatasetParameters();
    dataset_map["office"].pcd_file_folder = "/home/jiahao/datasets/2024-11-19_14-05-08_rec013/slam_clouds/";
    dataset_map["office"].pose_file_path = "/home/jiahao/datasets/2024-11-19_14-05-08_rec013/slam_pose_graph.slam";

    dataset_map["abingdon"] = DatasetParameters();
    dataset_map["abingdon"].pcd_file_folder = "/home/jiahao/datasets/abingdon logs/2024-12-03_10-25-53_rec001_rad301_run2/slam_clouds/";
    dataset_map["abingdon"].pose_file_path = "/home/jiahao/datasets/abingdon logs/2024-12-03_10-25-53_rec001_rad301_run2/slam_pose_graph.slam";

    dataset_map["bodleian01"] = DatasetParameters();
    dataset_map["bodleian01"].pcd_file_folder = "/home/jiahao/datasets/spires_benchmark/bodleian01/undist-clouds/";
    dataset_map["bodleian01"].pose_file_path = "/home/jiahao/datasets/spires_benchmark/bodleian01/slam-poses.csv";
    dataset_map["bodleian01"].remove_double_return_flag = true;
    dataset_map["bodleian01"].range_precision = 0.02;

    dataset_map["blenheim04"] = DatasetParameters();
    dataset_map["blenheim04"].pcd_file_folder = "/home/jiahao/datasets/spires_benchmark/blenheim04/undist-clouds/";
    dataset_map["blenheim04"].pose_file_path = "/home/jiahao/datasets/spires_benchmark/blenheim04/slam-poses.csv";
    dataset_map["blenheim04"].remove_double_return_flag = true;

    dataset_map["kitti01"] = DatasetParameters();
    dataset_map["kitti01"].pcd_file_folder = "/home/jiahao/datasets/spires_benchmark/kitti_dataset/sequences/01/pcd/";
    dataset_map["kitti01"].pose_file_path = "/home/jiahao/datasets/spires_benchmark/kitti_dataset/poses/01.txt";
    dataset_map["kitti01"].range_precision = 0.02; // Velodyne HDL-64E Laserscanner

    dataset_map["mac_keble03"] = DatasetParameters();
    dataset_map["mac_keble03"].pcd_file_folder = "/Users/jiahao/dataset/keble03/undist-clouds-filtered/";
    dataset_map["mac_keble03"].pose_file_path = "/Users/jiahao/dataset/keble03/gt-tum.txt";
    dataset_map["mac_keble03"].remove_double_return_flag = false;
    dataset_map["mac_keble03"].filter_low_intensity_flag = false;

    dataset_map["sample"] = DatasetParameters();
    dataset_map["sample"].pcd_file_folder = "../sample_data/slam_clouds/";
    dataset_map["sample"].pose_file_path = "../sample_data/slam_pose_graph.slam";
    dataset_map["sample"].remove_double_return_flag = false;
    dataset_map["sample"].filter_low_intensity_flag = false;

    // ======= benchmark final =======

    dataset_map["christchurch03"] = DatasetParameters();
    dataset_map["christchurch03"].pcd_file_folder = "/home/jiahao/datasets/spires_benchmark/christchurch03/undist-clouds-filtered/";
    dataset_map["christchurch03"].pose_file_path = "/home/jiahao/datasets/spires_benchmark/christchurch03/gt-tum.txt";
    dataset_map["christchurch03"].remove_double_return_flag = false;
    dataset_map["christchurch03"].filter_low_intensity_flag = false;
    // 290 scans

    dataset_map["keble03"] = DatasetParameters();
    dataset_map["keble03"].pcd_file_folder = "/home/jiahao/datasets/spires_benchmark/keble03/undist-clouds-filtered/";
    dataset_map["keble03"].pose_file_path = "/home/jiahao/datasets/spires_benchmark/keble03/gt-tum.txt";
    dataset_map["keble03"].remove_double_return_flag = false;
    dataset_map["keble03"].filter_low_intensity_flag = false;
    // 300 scans

    dataset_map["observatory01"] = DatasetParameters();
    dataset_map["observatory01"].pcd_file_folder = "/home/jiahao/datasets/spires_benchmark/observatory01/undist-clouds-filtered/";
    dataset_map["observatory01"].pose_file_path = "/home/jiahao/datasets/spires_benchmark/observatory01/gt-tum.txt";
    dataset_map["observatory01"].remove_double_return_flag = false;
    dataset_map["observatory01"].filter_low_intensity_flag = false;
    // 300 scans

    std::string dataset = "sample";
    
    headless_mode = false;
    num_scans = 50;
    save_folder = "/home/jiahao/datasets/spires_benchmark/" + dataset + "/Benchmark_final/PlanarMesh/";

    data_loader_settings.pcd_file_folder = dataset_map[dataset].pcd_file_folder;
    data_loader_settings.pose_file_path = dataset_map[dataset].pose_file_path;
    range_precision = dataset_map[dataset].range_precision;
    range_accuracy = dataset_map[dataset].range_accuracy;
    data_loader_settings.remove_double_return_flag = dataset_map[dataset].remove_double_return_flag;
    data_loader_settings.filter_low_intensity_flag = dataset_map[dataset].filter_low_intensity_flag;
    radius_value = dataset_map[dataset].radius_value;
    extra_radius = dataset_map[dataset].extra_radius;
    radius_ratio = dataset_map[dataset].radius_ratio;

    high_incident_angle_threshold_std = 0.1;

    cleanup_seed_surface_after_ith_cloud = -1;
    cleanup_seed_surface_after_distance_travelled = 3.0;

    simplify_surfaces_density_threshold = 0;
    simplify_surfaces_radius_lower_bound = 0.05;
    simplify_surfaces_radius_lower_ratio = 1.0;

    // cap radius of boundary vertices when simplify mesh
    simplify_surfaces_boundary_radius_upper_bound = 0.05;

    use_sim_data = false;
    sim_object = 0;
    data_loader_settings.start_cloud = 0;
    data_loader_settings.start_point = 0;

    process_every_n_points = 1;
    
    fit_plane_threshold = 3;
    shuffle_pointcloud = true;
    use_radius_value = true;
    pointcloud_fraction = 1;
    duplicated_point_distance_threshold = 0.0; // if two points are closer than this distance, they are considered the same point

    odometry_position_uncertainty_rate = 0.000; // kitti sota 0.005m/m (0.5%)
    odometry_angular_uncertainty_rate = 0.000; // kitti sota 0.001 deg/m

    confidence_interval_multiplier = 1.96; // for 95% confidence interval
    // confidence_interval_multiplier = 2.576; // for 99% confidence interval
    // confidence_interval_multiplier = 1.0; // for testing

    abnormal_size = 1.5;
    envelope_size = 3.5;

    seed_surface_area_threshold = 0.02; // below which is considered as a seed surface | 0.01 m^2 = 100 cm^2

    // num_of_delete_before_put_to_repeated_queue = 2;
    
    num_threads = 1;
    record_countent_surface_count = false;

    use_queue = true;
    retry_threshold = 10;
    num_iterations = 10;

    // min_face_angle = 20;

    // log settings
    bool show_all = false;
    if (show_all)
    {
        log.add_point_by_radius_search = true;
        log.load_point_cloud = true;
        log.step = true;
        log.refine_surfaces = true;
        log.process_point = true;
        log.initialize = true;
        log.deletion = true;
        log.review_surfaces = true;
        log.connect_by_edges_and_faces = true;
        log.can_merge = true;
        log.merge_surface = true;    
        log.duplicated_point = true;
        log.num_of_concurrent_processes = true;
        log.total_processed_points = true;
        log.show_contented_surface = true;
    }
    else
    {
        log.add_point_by_radius_search = false;
        log.load_point_cloud = true;
        log.step = false;
        log.refine_surfaces = false;
        log.process_point = false;
        log.initialize = false;
        log.deletion = false;
        log.review_surfaces = false;
        log.connect_by_edges_and_faces = false;
        log.can_merge = false;
        log.merge_surface = false;
        log.duplicated_point = false;
        log.num_of_concurrent_processes = false;
        log.total_processed_points = false;
        log.show_contented_surface = false;
    }

    // output time
    output_time = false;
    output_file_name = "with_sah_threads_more_iteration_" + std::to_string(num_threads) + ".txt";
    turn_off_sah = false;

    // interactive viewer settings
    update_display = true;
    flip_color = false;
    show_internal_vertices = true;
    show_generic_points = false;
    show_interior_points = false;
    show_pointcloud = false;
    show_triangle = true;
    show_edge = false;
    show_keycode = false;
    show_seed_surface = false;
    color_mode = ColorMode::ID;
    point_mode = PointMode::ORIGINAL;
    surface_denominator = 10.0;
    siblings_denominator = 3.0;
    radius_denominator = 0.3;
    positional_uncertainty_denominator = 0.002;
    contention_denominator = 500;
    show_wireframe = true;
    show_sphere = false;
    number_of_spheres_to_display = 60;
}

bool Settings::edge_is_short_enough(const double& edge_length, const double& radius0, const double& radius1) const
{
    const double minimum_edge_length = 0.0f;
    return edge_length < radius0 + minimum_edge_length && edge_length < radius1 + minimum_edge_length;
}

double Settings::compute_rrs_half_size(const double& radius) const
{
    const double minimum_rrs_half_size = 0.0f;
    return std::max(minimum_rrs_half_size, radius);
}