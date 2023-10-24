#include <rclcpp/rclcpp.hpp>
#include <elevation_mapping_ros2/ElevationMapping.hpp>
#include <elevation_mapping_ros2/post_processing/PostProcessor.hpp>
#include <elevation_ccl/elevation_ccl.h>
#include <convex_plane_extractor/convex_plane_extractor.h>
#include <convex_plane_visualizer/convex_plane_visualizer.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options = rclcpp::NodeOptions().use_intra_process_comms(true);

    rclcpp::Node::SharedPtr elevation_mapping = std::make_shared<elevation_mapping::ElevationMapping>(options);
    rclcpp::Node::SharedPtr post_processor = std::make_shared<elevation_mapping::PostProcessor>(options);
    rclcpp::Node::SharedPtr elevation_ccl = std::make_shared<elevation_ccl::ElevationCCL>(options);
    rclcpp::Node::SharedPtr convex_plane_extractor = std::make_shared<convex_plane::ConvexPlaneExtractor>(options);
    rclcpp::Node::SharedPtr convex_plane_visualizer = std::make_shared<convex_plane::ConvexPlaneVisualizer>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(elevation_mapping);
    executor.add_node(post_processor);
    executor.add_node(elevation_ccl);
    executor.add_node(convex_plane_extractor);
    executor.add_node(convex_plane_visualizer);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}