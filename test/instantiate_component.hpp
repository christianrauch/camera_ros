#pragma once
#include <class_loader/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <rclcpp_components/node_factory_template.hpp>


/**
 * @brief instantiate a component as node
 *
 * Search all linked libraries for the factory of the given component and
 * instantiate a node with optional options.
 *
 * Base on the `linktime_composition` example of the `composition` package:
 * https://github.com/ros2/demos/blob/rolling/composition/src/linktime_composition.cpp
 *
 * @throws class_loader::CreateClassException if component does not exist.
 * @throws rclcpp::exceptions::RCLInvalidArgument if `rclcpp::init()` has not been called before
 *
 * @param component name of the component
 * @param options node options
 * @return instance of the component as node
 */
rclcpp_components::NodeInstanceWrapper
instantiate_component(const std::string &component,
                      const rclcpp::NodeOptions &options = rclcpp::NodeOptions {})
{
  const std::string node_factory_class_name = "rclcpp_components::NodeFactoryTemplate<" + component + ">";

  // load all classes from statically or dynamically linked libraries
  std::unique_ptr<class_loader::ClassLoader> loader =
    std::make_unique<class_loader::ClassLoader>(std::string {});

  // instantiate the node factory for the component
  class_loader::ClassLoader::UniquePtr<rclcpp_components::NodeFactory> node_factory =
    loader->createUniqueInstance<rclcpp_components::NodeFactory>(node_factory_class_name);

  // instantiate the node
  return node_factory->create_node_instance(options);
}
