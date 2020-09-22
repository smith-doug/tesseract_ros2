/**
 * @file utils.cpp
 * @brief Tesseract ROS utility functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2018, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <octomap_msgs/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <ros/package.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/utils.h>

namespace tesseract_rosutils
{
std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ros::package::getPath(package);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }
  else if (url.find("file://") == 0)
  {
    mod_url.erase(0, strlen("file://"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }
  }

  return mod_url;
}

ROSResourceLocator::ROSResourceLocator() : SimpleResourceLocator(::tesseract_rosutils::locateResource) {}

bool isMsgEmpty(const sensor_msgs::JointState& msg)
{
  return msg.name.empty() && msg.position.empty() && msg.velocity.empty() && msg.effort.empty();
}

bool isMsgEmpty(const sensor_msgs::MultiDOFJointState& msg)
{
  return msg.joint_names.empty() && msg.transforms.empty() && msg.twist.empty() && msg.wrench.empty();
}

bool isIdentical(const tesseract_geometry::Geometry& shape1, const tesseract_geometry::Geometry& shape2)
{
  if (shape1.getType() != shape2.getType())
    return false;

  switch (shape1.getType())
  {
    case tesseract_geometry::GeometryType::BOX:
    {
      const auto& s1 = static_cast<const tesseract_geometry::Box&>(shape1);
      const auto& s2 = static_cast<const tesseract_geometry::Box&>(shape2);

      if (std::abs(s1.getX() - s2.getX()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getY() - s2.getY()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getZ() - s2.getZ()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::SPHERE:
    {
      const auto& s1 = static_cast<const tesseract_geometry::Sphere&>(shape1);
      const auto& s2 = static_cast<const tesseract_geometry::Sphere&>(shape2);

      if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      const auto& s1 = static_cast<const tesseract_geometry::Cylinder&>(shape1);
      const auto& s2 = static_cast<const tesseract_geometry::Cylinder&>(shape2);

      if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      const auto& s1 = static_cast<const tesseract_geometry::Cone&>(shape1);
      const auto& s2 = static_cast<const tesseract_geometry::Cone&>(shape2);

      if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      const auto& s1 = static_cast<const tesseract_geometry::Mesh&>(shape1);
      const auto& s2 = static_cast<const tesseract_geometry::Mesh&>(shape2);

      if (s1.getVerticeCount() != s2.getVerticeCount())
        return false;

      if (s1.getTriangleCount() != s2.getTriangleCount())
        return false;

      if (s1.getTriangles() != s2.getTriangles())
        return false;

      if (s1.getVertices() != s2.getVertices())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      const auto& s1 = static_cast<const tesseract_geometry::ConvexMesh&>(shape1);
      const auto& s2 = static_cast<const tesseract_geometry::ConvexMesh&>(shape2);

      if (s1.getVerticeCount() != s2.getVerticeCount())
        return false;

      if (s1.getFaceCount() != s2.getFaceCount())
        return false;

      if (s1.getFaces() != s2.getFaces())
        return false;

      if (s1.getVertices() != s2.getVertices())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::SDF_MESH:
    {
      const auto& s1 = static_cast<const tesseract_geometry::Mesh&>(shape1);
      const auto& s2 = static_cast<const tesseract_geometry::Mesh&>(shape2);

      if (s1.getVerticeCount() != s2.getVerticeCount())
        return false;

      if (s1.getTriangleCount() != s2.getTriangleCount())
        return false;

      if (s1.getTriangles() != s2.getTriangles())
        return false;

      if (s1.getVertices() != s2.getVertices())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      const auto& s1 = static_cast<const tesseract_geometry::Octree&>(shape1);
      const auto& s2 = static_cast<const tesseract_geometry::Octree&>(shape2);

      if (s1.getOctree()->getTreeType() != s2.getOctree()->getTreeType())
        return false;

      if (s1.getOctree()->size() != s2.getOctree()->size())
        return false;

      if (s1.getOctree()->getTreeDepth() != s2.getOctree()->getTreeDepth())
        return false;

      if (s1.getOctree()->memoryUsage() != s2.getOctree()->memoryUsage())
        return false;

      if (s1.getOctree()->memoryFullGrid() != s2.getOctree()->memoryFullGrid())
        return false;

      break;
    }
    default:
      ROS_ERROR("This geometric shape type (%d) is not supported", static_cast<int>(shape1.getType()));
      return false;
  }

  return true;
}

bool isIdentical(const tesseract_scene_graph::Visual& /*visual1*/, const tesseract_scene_graph::Visual& /*visual2*/)
{
  assert(false);
  return false;
}

bool isIdentical(const tesseract_scene_graph::Collision& /*collision1*/,
                 const tesseract_scene_graph::Collision& /*collision2*/)
{
  assert(false);
  return false;
}

bool isIdentical(const tesseract_scene_graph::Link& link1, const tesseract_scene_graph::Link& link2)
{
  if (link1.getName() != link2.getName())
    return false;

  if (link1.collision.size() != link2.collision.size())
    return false;

  for (unsigned i = 0; i < link1.collision.size(); ++i)
  {
    if (!isIdentical(*link1.collision[i], *link2.collision[i]))
      return false;
  }

  // Check Visual
  if (link1.visual.size() != link2.visual.size())
    return false;

  for (unsigned i = 0; i < link1.visual.size(); ++i)
  {
    if (!isIdentical(*link1.visual[i], *link2.visual[i]))
      return false;
  }

  return true;
}

bool fromMsg(Eigen::Isometry3d& pose, const geometry_msgs::Pose& pose_msg)
{
  tf::poseMsgToEigen(pose_msg, pose);
  return true;
}

bool toMsg(geometry_msgs::Pose& pose_msg, const Eigen::Isometry3d& pose)
{
  tf::poseEigenToMsg(pose, pose_msg);
  return true;
}

/** \brief Construct the message that corresponds to the shape. Return false on failure. */
bool toMsg(tesseract_msgs::Geometry& geometry_msgs, const tesseract_geometry::Geometry& geometry)
{
  switch (geometry.getType())
  {
    case tesseract_geometry::GeometryType::SPHERE:
    {
      const auto& sphere = static_cast<const tesseract_geometry::Sphere&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::SPHERE;
      geometry_msgs.sphere_radius = sphere.getRadius();
      break;
    }
    case tesseract_geometry::GeometryType::BOX:
    {
      const auto& box = static_cast<const tesseract_geometry::Box&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::BOX;
      geometry_msgs.box_dimensions[0] = box.getX();
      geometry_msgs.box_dimensions[1] = box.getY();
      geometry_msgs.box_dimensions[2] = box.getZ();
      break;
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      const auto& cylinder = static_cast<const tesseract_geometry::Cylinder&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::CYLINDER;
      geometry_msgs.cylinder_dimensions[0] = cylinder.getRadius();
      geometry_msgs.cylinder_dimensions[1] = cylinder.getLength();
      break;
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      const auto& cone = static_cast<const tesseract_geometry::Cone&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::CONE;
      geometry_msgs.cone_dimensions[0] = cone.getRadius();
      geometry_msgs.cone_dimensions[1] = cone.getLength();
      break;
    }
    case tesseract_geometry::GeometryType::PLANE:
    {
      const auto& plane = static_cast<const tesseract_geometry::Plane&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::PLANE;
      geometry_msgs.plane_coeff[0] = plane.getA();
      geometry_msgs.plane_coeff[1] = plane.getB();
      geometry_msgs.plane_coeff[2] = plane.getC();
      geometry_msgs.plane_coeff[3] = plane.getD();
      break;
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      const auto& octree = static_cast<const tesseract_geometry::Octree&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::OCTREE;
      octomap_msgs::fullMapToMsg(*(octree.getOctree()), geometry_msgs.octomap);
      break;
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      const auto& mesh = static_cast<const tesseract_geometry::Mesh&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::MESH;

      const tesseract_common::VectorVector3d& vertices = *(mesh.getVertices());
      geometry_msgs.mesh.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        geometry_msgs.mesh.vertices[i].x = vertices[i](0);
        geometry_msgs.mesh.vertices[i].y = vertices[i](1);
        geometry_msgs.mesh.vertices[i].z = vertices[i](2);
      }

      const Eigen::VectorXi& faces = *(mesh.getTriangles());
      geometry_msgs.mesh.faces.resize(static_cast<size_t>(faces.size()));
      for (size_t i = 0; i < static_cast<size_t>(faces.size()); ++i)
        geometry_msgs.mesh.faces[i] = static_cast<unsigned>(faces[static_cast<unsigned>(i)]);

      if (mesh.getResource() && mesh.getResource()->isFile())
      {
        geometry_msgs.mesh.file_path = mesh.getResource()->getFilePath();
        ;
      }
      if (geometry_msgs.mesh.file_path.empty())
      {
        geometry_msgs.mesh.scale[0] = 1;
        geometry_msgs.mesh.scale[1] = 1;
        geometry_msgs.mesh.scale[2] = 1;
      }
      else
      {
        const Eigen::Vector3f& scale = mesh.getScale().cast<float>();
        geometry_msgs.mesh.scale[0] = scale.x();
        geometry_msgs.mesh.scale[1] = scale.y();
        geometry_msgs.mesh.scale[2] = scale.z();
      }

      break;
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      const auto& mesh = static_cast<const tesseract_geometry::ConvexMesh&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::CONVEX_MESH;

      const tesseract_common::VectorVector3d& vertices = *(mesh.getVertices());
      geometry_msgs.mesh.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        geometry_msgs.mesh.vertices[i].x = vertices[i](0);
        geometry_msgs.mesh.vertices[i].y = vertices[i](1);
        geometry_msgs.mesh.vertices[i].z = vertices[i](2);
      }

      const Eigen::VectorXi& faces = *(mesh.getFaces());
      geometry_msgs.mesh.faces.resize(static_cast<size_t>(faces.size()));
      for (size_t i = 0; i < static_cast<size_t>(faces.size()); ++i)
        geometry_msgs.mesh.faces[i] = static_cast<unsigned>(faces[static_cast<unsigned>(i)]);

      if (mesh.getResource() && mesh.getResource()->isFile())
      {
        geometry_msgs.mesh.file_path = mesh.getResource()->getFilePath();
        ;
      }
      if (geometry_msgs.mesh.file_path.empty())
      {
        geometry_msgs.mesh.scale[0] = 1;
        geometry_msgs.mesh.scale[1] = 1;
        geometry_msgs.mesh.scale[2] = 1;
      }
      else
      {
        const Eigen::Vector3f& scale = mesh.getScale().cast<float>();
        geometry_msgs.mesh.scale[0] = scale.x();
        geometry_msgs.mesh.scale[1] = scale.y();
        geometry_msgs.mesh.scale[2] = scale.z();
      }

      break;
    }
    case tesseract_geometry::GeometryType::SDF_MESH:
    {
      const auto& mesh = static_cast<const tesseract_geometry::SDFMesh&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::SDF_MESH;

      const tesseract_common::VectorVector3d& vertices = *(mesh.getVertices());
      geometry_msgs.mesh.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        geometry_msgs.mesh.vertices[i].x = vertices[i](0);
        geometry_msgs.mesh.vertices[i].y = vertices[i](1);
        geometry_msgs.mesh.vertices[i].z = vertices[i](2);
      }

      const Eigen::VectorXi& faces = *(mesh.getTriangles());
      geometry_msgs.mesh.faces.resize(static_cast<size_t>(faces.size()));
      for (size_t i = 0; i < static_cast<size_t>(faces.size()); ++i)
        geometry_msgs.mesh.faces[i] = static_cast<unsigned>(faces[static_cast<unsigned>(i)]);

      if (mesh.getResource() && mesh.getResource()->isFile())
      {
        geometry_msgs.mesh.file_path = mesh.getResource()->getFilePath();
        ;
      }
      if (geometry_msgs.mesh.file_path.empty())
      {
        geometry_msgs.mesh.scale[0] = 1;
        geometry_msgs.mesh.scale[1] = 1;
        geometry_msgs.mesh.scale[2] = 1;
      }
      else
      {
        const Eigen::Vector3f& scale = mesh.getScale().cast<float>();
        geometry_msgs.mesh.scale[0] = scale.x();
        geometry_msgs.mesh.scale[1] = scale.y();
        geometry_msgs.mesh.scale[2] = scale.z();
      }

      break;
    }
    default:
    {
      ROS_ERROR("Unable to construct primitive shape message for shape of type %d",
                static_cast<int>(geometry.getType()));
      return false;
    }
  }

  return true;
}

bool fromMsg(tesseract_geometry::Geometry::Ptr& geometry, const tesseract_msgs::Geometry& geometry_msg)
{
  geometry = nullptr;
  if (geometry_msg.type == tesseract_msgs::Geometry::SPHERE)
  {
    geometry = std::make_shared<tesseract_geometry::Sphere>(geometry_msg.sphere_radius);
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::BOX)
  {
    geometry = std::make_shared<tesseract_geometry::Box>(
        geometry_msg.box_dimensions[0], geometry_msg.box_dimensions[1], geometry_msg.box_dimensions[2]);
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::CYLINDER)
  {
    geometry = std::make_shared<tesseract_geometry::Cylinder>(geometry_msg.cylinder_dimensions[0],
                                                              geometry_msg.cylinder_dimensions[1]);
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::CONE)
  {
    geometry =
        std::make_shared<tesseract_geometry::Cone>(geometry_msg.cone_dimensions[0], geometry_msg.cone_dimensions[1]);
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::PLANE)
  {
    geometry = std::make_shared<tesseract_geometry::Plane>(geometry_msg.plane_coeff[0],
                                                           geometry_msg.plane_coeff[1],
                                                           geometry_msg.plane_coeff[2],
                                                           geometry_msg.plane_coeff[3]);
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::MESH)
  {
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>(geometry_msg.mesh.vertices.size());
    auto faces = std::make_shared<Eigen::VectorXi>(geometry_msg.mesh.faces.size());

    for (unsigned int i = 0; i < geometry_msg.mesh.vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(
          geometry_msg.mesh.vertices[i].x, geometry_msg.mesh.vertices[i].y, geometry_msg.mesh.vertices[i].z);

    for (unsigned int i = 0; i < geometry_msg.mesh.faces.size(); ++i)
      (*faces)[static_cast<int>(i)] = static_cast<int>(geometry_msg.mesh.faces[i]);

    if (!geometry_msg.mesh.file_path.empty())
      geometry = std::make_shared<tesseract_geometry::Mesh>(
          vertices,
          faces,
          std::make_shared<tesseract_scene_graph::SimpleLocatedResource>(geometry_msg.mesh.file_path,
                                                                         geometry_msg.mesh.file_path),
          Eigen::Vector3f(geometry_msg.mesh.scale[0], geometry_msg.mesh.scale[1], geometry_msg.mesh.scale[2])
              .cast<double>());
    else
      geometry = std::make_shared<tesseract_geometry::Mesh>(vertices, faces);
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::CONVEX_MESH)
  {
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>(geometry_msg.mesh.vertices.size());
    auto faces = std::make_shared<Eigen::VectorXi>(geometry_msg.mesh.faces.size());

    for (unsigned int i = 0; i < geometry_msg.mesh.vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(
          geometry_msg.mesh.vertices[i].x, geometry_msg.mesh.vertices[i].y, geometry_msg.mesh.vertices[i].z);

    for (unsigned int i = 0; i < geometry_msg.mesh.faces.size(); ++i)
      (*faces)[static_cast<int>(i)] = static_cast<int>(geometry_msg.mesh.faces[i]);

    if (!geometry_msg.mesh.file_path.empty())
      geometry = std::make_shared<tesseract_geometry::ConvexMesh>(
          vertices,
          faces,
          std::make_shared<tesseract_scene_graph::SimpleLocatedResource>(geometry_msg.mesh.file_path,
                                                                         geometry_msg.mesh.file_path),
          Eigen::Vector3f(geometry_msg.mesh.scale[0], geometry_msg.mesh.scale[1], geometry_msg.mesh.scale[2])
              .cast<double>());
    else
      geometry = std::make_shared<tesseract_geometry::ConvexMesh>(vertices, faces);
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::SDF_MESH)
  {
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>(geometry_msg.mesh.vertices.size());
    auto faces = std::make_shared<Eigen::VectorXi>(geometry_msg.mesh.faces.size());

    for (unsigned int i = 0; i < geometry_msg.mesh.vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(
          geometry_msg.mesh.vertices[i].x, geometry_msg.mesh.vertices[i].y, geometry_msg.mesh.vertices[i].z);

    for (unsigned int i = 0; i < geometry_msg.mesh.faces.size(); ++i)
      (*faces)[static_cast<int>(i)] = static_cast<int>(geometry_msg.mesh.faces[i]);

    if (!geometry_msg.mesh.file_path.empty())
      geometry = std::make_shared<tesseract_geometry::SDFMesh>(
          vertices,
          faces,
          std::make_shared<tesseract_scene_graph::SimpleLocatedResource>(geometry_msg.mesh.file_path,
                                                                         geometry_msg.mesh.file_path),
          Eigen::Vector3f(geometry_msg.mesh.scale[0], geometry_msg.mesh.scale[1], geometry_msg.mesh.scale[2])
              .cast<double>());
    else
      geometry = std::make_shared<tesseract_geometry::SDFMesh>(vertices, faces);
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::OCTREE)
  {
    std::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(geometry_msg.octomap)));
    auto sub_type = static_cast<tesseract_geometry::Octree::SubType>(geometry_msg.octomap_sub_type.type);
    geometry = std::make_shared<tesseract_geometry::Octree>(om, sub_type);
  }

  if (geometry == nullptr)
  {
    ROS_ERROR("Unable to construct shape corresponding to shape_msg of type %d", static_cast<int>(geometry_msg.type));
    return false;
  }

  return true;
}

bool toMsg(tesseract_msgs::Material& material_msg, const tesseract_scene_graph::Material::Ptr& material)
{
  if (material == nullptr)
  {
    material_msg.empty = true;
    return true;
  }

  material_msg.name = material->getName();
  material_msg.texture_filename = material->texture_filename;
  material_msg.color.r = static_cast<float>(material->color(0));
  material_msg.color.g = static_cast<float>(material->color(1));
  material_msg.color.b = static_cast<float>(material->color(2));
  material_msg.color.a = static_cast<float>(material->color(3));
  return true;
}

bool fromMsg(tesseract_scene_graph::Material::Ptr& material, const tesseract_msgs::Material& material_msg)
{
  if (material_msg.empty)
  {
    material = nullptr;
    return true;
  }

  material = std::make_shared<tesseract_scene_graph::Material>(material_msg.name);
  material->texture_filename = material_msg.texture_filename;
  material->color(0) = static_cast<double>(material_msg.color.r);
  material->color(1) = static_cast<double>(material_msg.color.g);
  material->color(2) = static_cast<double>(material_msg.color.b);
  material->color(3) = static_cast<double>(material_msg.color.a);
  return true;
}

bool toMsg(tesseract_msgs::Inertial& inertial_msg, const tesseract_scene_graph::Inertial::Ptr& inertial)
{
  if (inertial == nullptr)
  {
    inertial_msg.empty = true;
    return true;
  }

  tf::poseEigenToMsg(inertial->origin, inertial_msg.origin);

  inertial_msg.mass = inertial->mass;
  inertial_msg.ixx = inertial->ixx;
  inertial_msg.ixy = inertial->ixy;
  inertial_msg.ixz = inertial->ixz;
  inertial_msg.iyy = inertial->iyy;
  inertial_msg.iyz = inertial->iyz;
  inertial_msg.izz = inertial->izz;

  return true;
}

bool fromMsg(tesseract_scene_graph::Inertial::Ptr& inertial, const tesseract_msgs::Inertial& inertial_msg)
{
  if (inertial_msg.empty)
  {
    inertial = nullptr;
    return true;
  }

  inertial = std::make_shared<tesseract_scene_graph::Inertial>();

  tf::poseMsgToEigen(inertial_msg.origin, inertial->origin);

  inertial->mass = inertial_msg.mass;
  inertial->ixx = inertial_msg.ixx;
  inertial->ixy = inertial_msg.ixy;
  inertial->ixz = inertial_msg.ixz;
  inertial->iyy = inertial_msg.iyy;
  inertial->iyz = inertial_msg.iyz;
  inertial->izz = inertial_msg.izz;

  return true;
}

bool toMsg(tesseract_msgs::VisualGeometry& visual_msg, const tesseract_scene_graph::Visual& visual)
{
  visual_msg.name = visual.name;
  tf::poseEigenToMsg(visual.origin, visual_msg.origin);
  toMsg(visual_msg.geometry, *(visual.geometry));
  toMsg(visual_msg.material, visual.material);
  return true;
}

bool fromMsg(tesseract_scene_graph::Visual::Ptr& visual, const tesseract_msgs::VisualGeometry& visual_msg)
{
  visual = std::make_shared<tesseract_scene_graph::Visual>();
  visual->name = visual_msg.name;
  tf::poseMsgToEigen(visual_msg.origin, visual->origin);
  fromMsg(visual->geometry, visual_msg.geometry);
  fromMsg(visual->material, visual_msg.material);
  return true;
}

bool toMsg(tesseract_msgs::CollisionGeometry& collision_msg, const tesseract_scene_graph::Collision& collision)
{
  collision_msg.name = collision.name;
  tf::poseEigenToMsg(collision.origin, collision_msg.origin);
  toMsg(collision_msg.geometry, *(collision.geometry));
  return true;
}

bool fromMsg(tesseract_scene_graph::Collision::Ptr& collision, const tesseract_msgs::CollisionGeometry& collision_msg)
{
  collision = std::make_shared<tesseract_scene_graph::Collision>();
  collision->name = collision_msg.name;
  tf::poseMsgToEigen(collision_msg.origin, collision->origin);
  fromMsg(collision->geometry, collision_msg.geometry);
  return true;
}

bool toMsg(tesseract_msgs::Link& link_msg, const tesseract_scene_graph::Link& link)
{
  link_msg.name = link.getName();

  toMsg(link_msg.inertial, link.inertial);

  link_msg.collision.resize(link.collision.size());
  for (size_t i = 0; i < link.collision.size(); ++i)
    toMsg(link_msg.collision[i], *(link.collision[i]));

  link_msg.visual.resize(link.visual.size());
  for (size_t i = 0; i < link.visual.size(); ++i)
    toMsg(link_msg.visual[i], *(link.visual[i]));

  return true;
}

tesseract_scene_graph::Link fromMsg(const tesseract_msgs::Link& link_msg)
{
  tesseract_scene_graph::Link link(link_msg.name);

  fromMsg(link.inertial, link_msg.inertial);

  link.collision.resize(link_msg.collision.size());
  for (size_t i = 0; i < link_msg.collision.size(); ++i)
    fromMsg(link.collision[i], link_msg.collision[i]);

  link.visual.resize(link_msg.visual.size());
  for (size_t i = 0; i < link_msg.visual.size(); ++i)
    fromMsg(link.visual[i], link_msg.visual[i]);

  return link;
}

bool toMsg(tesseract_msgs::JointCalibration& joint_calibration_msg,
           const tesseract_scene_graph::JointCalibration::Ptr& joint_calibration)
{
  if (joint_calibration == nullptr)
  {
    joint_calibration_msg.empty = true;
    return true;
  }

  joint_calibration_msg.reference_position = joint_calibration->reference_position;

  joint_calibration_msg.rising = joint_calibration->rising;

  joint_calibration_msg.falling = joint_calibration->falling;

  return true;
}

bool fromMsg(tesseract_scene_graph::JointCalibration::Ptr& joint_calibration,
             const tesseract_msgs::JointCalibration& joint_calibration_msg)
{
  if (joint_calibration_msg.empty)
  {
    joint_calibration = nullptr;
    return true;
  }
  joint_calibration = std::make_shared<tesseract_scene_graph::JointCalibration>();

  joint_calibration->reference_position = joint_calibration_msg.reference_position;

  joint_calibration->rising = joint_calibration_msg.rising;

  joint_calibration->falling = joint_calibration_msg.falling;

  return true;
}

bool toMsg(tesseract_msgs::JointDynamics& joint_dynamics_msg,
           const tesseract_scene_graph::JointDynamics::Ptr& joint_dynamics)
{
  if (joint_dynamics == nullptr)
  {
    joint_dynamics_msg.empty = true;
    return true;
  }

  joint_dynamics_msg.damping = joint_dynamics->damping;

  joint_dynamics_msg.friction = joint_dynamics->friction;

  return true;
}

bool fromMsg(tesseract_scene_graph::JointDynamics::Ptr& joint_dynamics,
             const tesseract_msgs::JointDynamics& joint_dynamics_msg)
{
  if (joint_dynamics_msg.empty)
  {
    joint_dynamics = nullptr;
    return true;
  }

  joint_dynamics = std::make_shared<tesseract_scene_graph::JointDynamics>();

  joint_dynamics->damping = joint_dynamics_msg.damping;

  joint_dynamics->friction = joint_dynamics_msg.friction;

  return true;
}

bool toMsg(tesseract_msgs::JointLimits& joint_limits_msg, const tesseract_scene_graph::JointLimits::Ptr& joint_limits)
{
  if (joint_limits == nullptr)
  {
    joint_limits_msg.empty = true;
    return true;
  }

  joint_limits_msg.lower = joint_limits->lower;

  joint_limits_msg.upper = joint_limits->upper;

  joint_limits_msg.effort = joint_limits->effort;

  joint_limits_msg.velocity = joint_limits->velocity;

  return true;
}

bool fromMsg(tesseract_scene_graph::JointLimits::Ptr& joint_limits, const tesseract_msgs::JointLimits& joint_limits_msg)
{
  if (joint_limits_msg.empty)
  {
    joint_limits = nullptr;
    return true;
  }

  joint_limits = std::make_shared<tesseract_scene_graph::JointLimits>();

  joint_limits->lower = joint_limits_msg.lower;

  joint_limits->upper = joint_limits_msg.upper;

  joint_limits->effort = joint_limits_msg.effort;

  joint_limits->velocity = joint_limits_msg.velocity;

  return true;
}

bool toMsg(tesseract_msgs::JointMimic& joint_mimic_msg, const tesseract_scene_graph::JointMimic::Ptr& joint_mimic)
{
  if (joint_mimic == nullptr)
  {
    joint_mimic_msg.empty = true;
    return true;
  }

  joint_mimic_msg.offset = joint_mimic->offset;

  joint_mimic_msg.multiplier = joint_mimic->multiplier;

  joint_mimic_msg.joint_name = joint_mimic->joint_name;

  return true;
}

bool fromMsg(tesseract_scene_graph::JointMimic::Ptr& joint_mimic, const tesseract_msgs::JointMimic& joint_mimic_msg)
{
  if (joint_mimic_msg.empty)
  {
    joint_mimic = nullptr;
    return true;
  }

  joint_mimic = std::make_shared<tesseract_scene_graph::JointMimic>();

  joint_mimic->offset = joint_mimic_msg.offset;

  joint_mimic->multiplier = joint_mimic_msg.multiplier;

  joint_mimic->joint_name = joint_mimic_msg.joint_name;

  return true;
}

bool toMsg(tesseract_msgs::JointSafety& joint_safety_msg, const tesseract_scene_graph::JointSafety::Ptr& joint_safety)
{
  if (joint_safety == nullptr)
  {
    joint_safety_msg.empty = true;
    return true;
  }

  joint_safety_msg.soft_upper_limit = joint_safety->soft_upper_limit;

  joint_safety_msg.soft_lower_limit = joint_safety->soft_lower_limit;

  joint_safety_msg.k_position = joint_safety->k_position;

  joint_safety_msg.k_velocity = joint_safety->k_velocity;

  return true;
}

bool fromMsg(tesseract_scene_graph::JointSafety::Ptr& joint_safety, const tesseract_msgs::JointSafety& joint_safety_msg)
{
  if (joint_safety_msg.empty)
  {
    joint_safety = nullptr;
    return true;
  }

  joint_safety = std::make_shared<tesseract_scene_graph::JointSafety>();

  joint_safety->soft_upper_limit = joint_safety_msg.soft_upper_limit;

  joint_safety->soft_lower_limit = joint_safety_msg.soft_lower_limit;

  joint_safety->k_position = joint_safety_msg.k_position;

  joint_safety->k_velocity = joint_safety_msg.k_velocity;

  return true;
}

bool toMsg(tesseract_msgs::Joint& joint_msg, const tesseract_scene_graph::Joint& joint)
{
  joint_msg.name = joint.getName();
  joint_msg.type = static_cast<unsigned char>(joint.type);

  joint_msg.axis[0] = joint.axis[0];
  joint_msg.axis[1] = joint.axis[1];
  joint_msg.axis[2] = joint.axis[2];

  joint_msg.child_link_name = joint.child_link_name;
  joint_msg.parent_link_name = joint.parent_link_name;

  tf::poseEigenToMsg(joint.parent_to_joint_origin_transform, joint_msg.parent_to_joint_origin_transform);

  bool success = true;
  if (!toMsg(joint_msg.limits, joint.limits))
    success = false;

  if (!toMsg(joint_msg.dynamics, joint.dynamics))
    success = false;

  if (!toMsg(joint_msg.safety, joint.safety))
    success = false;

  if (!toMsg(joint_msg.calibration, joint.calibration))
    success = false;

  if (!toMsg(joint_msg.mimic, joint.mimic))
    success = false;

  return success;
}

tesseract_scene_graph::Joint fromMsg(const tesseract_msgs::Joint& joint_msg)
{
  tesseract_scene_graph::Joint joint(joint_msg.name);

  joint.type = static_cast<tesseract_scene_graph::JointType>(joint_msg.type);

  joint.axis[0] = joint_msg.axis[0];
  joint.axis[1] = joint_msg.axis[1];
  joint.axis[2] = joint_msg.axis[2];

  joint.child_link_name = joint_msg.child_link_name;
  joint.parent_link_name = joint_msg.parent_link_name;

  tf::poseMsgToEigen(joint_msg.parent_to_joint_origin_transform, joint.parent_to_joint_origin_transform);
  fromMsg(joint.limits, joint_msg.limits);
  fromMsg(joint.dynamics, joint_msg.dynamics);
  fromMsg(joint.safety, joint_msg.safety);
  fromMsg(joint.calibration, joint_msg.calibration);
  fromMsg(joint.mimic, joint_msg.mimic);

  return joint;
}

bool toMsg(std::vector<tesseract_msgs::AllowedCollisionEntry>& acm_msg,
           const tesseract_scene_graph::AllowedCollisionMatrix& acm)
{
  for (const auto& entry : acm.getAllAllowedCollisions())
  {
    tesseract_msgs::AllowedCollisionEntry entry_msg;
    entry_msg.link_1 = entry.first.first;
    entry_msg.link_2 = entry.first.second;
    entry_msg.reason = entry.second;
    acm_msg.push_back(entry_msg);
  }

  return true;
}

void toMsg(tesseract_msgs::SceneGraph& scene_graph_msg, const tesseract_scene_graph::SceneGraph& scene_graph)
{
  scene_graph_msg.id = scene_graph.getName();
  scene_graph_msg.root = scene_graph.getRoot();

  for (const auto& link : scene_graph.getLinks())
  {
    tesseract_msgs::Link link_msg;
    toMsg(link_msg, *link);
    scene_graph_msg.links.push_back(link_msg);
    if (!scene_graph.getLinkVisibility(link->getName()))
      scene_graph_msg.invisible_links.push_back(link->getName());

    if (!scene_graph.getLinkCollisionEnabled(link->getName()))
      scene_graph_msg.disabled_collision_links.push_back(link->getName());
  }

  for (const auto& joint : scene_graph.getJoints())
  {
    tesseract_msgs::Joint joint_msg;
    toMsg(joint_msg, *joint);
    scene_graph_msg.joints.push_back(joint_msg);
  }

  toMsg(scene_graph_msg.acm, *scene_graph.getAllowedCollisionMatrix());
}

tesseract_scene_graph::SceneGraph fromMsg(const tesseract_msgs::SceneGraph& scene_graph_msg)
{
  tesseract_scene_graph::SceneGraph g(scene_graph_msg.id);

  for (const auto& link_msg : scene_graph_msg.links)
    g.addLink(fromMsg(link_msg));

  for (const auto& joint_msg : scene_graph_msg.joints)
    g.addJoint(fromMsg(joint_msg));

  g.setRoot(scene_graph_msg.root);

  for (const auto& link_name : scene_graph_msg.invisible_links)
    g.setLinkVisibility(link_name, false);

  for (const auto& link_name : scene_graph_msg.disabled_collision_links)
    g.setLinkCollisionEnabled(link_name, false);

  for (const auto& entry : scene_graph_msg.acm)
    g.getAllowedCollisionMatrix()->addAllowedCollision(entry.link_1, entry.link_2, entry.reason);

  return g;
}

bool toMsg(tesseract_msgs::EnvironmentCommand& command_msg, const tesseract_environment::Command& command)
{
  switch (command.getType())
  {
    case tesseract_environment::CommandType::ADD:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::ADD;
      const auto& cmd = static_cast<const tesseract_environment::AddCommand&>(command);
      tesseract_rosutils::toMsg(command_msg.add_link, *(cmd.getLink()));
      tesseract_rosutils::toMsg(command_msg.add_joint, *(cmd.getJoint()));
      return true;
    }
    case tesseract_environment::CommandType::MOVE_LINK:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::MOVE_LINK;
      const auto& cmd = static_cast<const tesseract_environment::MoveLinkCommand&>(command);
      tesseract_rosutils::toMsg(command_msg.move_link_joint, *(cmd.getJoint()));
      return true;
    }
    case tesseract_environment::CommandType::MOVE_JOINT:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::MOVE_JOINT;
      const auto& cmd = static_cast<const tesseract_environment::MoveJointCommand&>(command);
      command_msg.move_joint_name = cmd.getJointName();
      command_msg.move_joint_parent_link = cmd.getParentLink();
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_LINK:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::REMOVE_LINK;
      const auto& cmd = static_cast<const tesseract_environment::RemoveLinkCommand&>(command);
      command_msg.remove_link = cmd.getLinkName();
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_JOINT:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::REMOVE_JOINT;
      const auto& cmd = static_cast<const tesseract_environment::RemoveJointCommand&>(command);
      command_msg.remove_joint = cmd.getJointName();
      return true;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_ORIGIN:
    {
      assert(false);
      return false;
    }
    case tesseract_environment::CommandType::CHANGE_JOINT_ORIGIN:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::CHANGE_JOINT_ORIGIN;
      const auto& cmd = static_cast<const tesseract_environment::ChangeJointOriginCommand&>(command);
      command_msg.change_joint_origin_name = cmd.getJointName();
      tf::poseEigenToMsg(cmd.getOrigin(), command_msg.change_joint_origin_pose);
      return true;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED;
      const auto& cmd = static_cast<const tesseract_environment::ChangeLinkCollisionEnabledCommand&>(command);
      command_msg.change_link_collision_enabled_name = cmd.getLinkName();
      command_msg.change_link_collision_enabled_value = cmd.getEnabled();
      return true;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_VISIBILITY:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::CHANGE_LINK_VISIBILITY;
      const auto& cmd = static_cast<const tesseract_environment::ChangeLinkVisibilityCommand&>(command);
      command_msg.change_link_visibility_name = cmd.getLinkName();
      command_msg.change_link_visibility_value = cmd.getEnabled();
      return true;
    }
    case tesseract_environment::CommandType::ADD_ALLOWED_COLLISION:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::ADD_ALLOWED_COLLISION;
      const auto& cmd = static_cast<const tesseract_environment::AddAllowedCollisionCommand&>(command);
      command_msg.add_allowed_collision.link_1 = cmd.getLinkName1();
      command_msg.add_allowed_collision.link_2 = cmd.getLinkName2();
      command_msg.add_allowed_collision.reason = cmd.getReason();
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION;
      const auto& cmd = static_cast<const tesseract_environment::RemoveAllowedCollisionCommand&>(command);
      command_msg.remove_allowed_collision.link_1 = cmd.getLinkName1();
      command_msg.remove_allowed_collision.link_2 = cmd.getLinkName2();
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK;
      const auto& cmd = static_cast<const tesseract_environment::RemoveAllowedCollisionLinkCommand&>(command);
      command_msg.remove_allowed_collision_link = cmd.getLinkName();
      return true;
    }
    case tesseract_environment::CommandType::ADD_SCENE_GRAPH:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::ADD_SCENE_GRAPH;
      const auto& cmd = static_cast<const tesseract_environment::AddSceneGraphCommand&>(command);

      toMsg(command_msg.scene_graph, *cmd.getSceneGraph());
      if (cmd.getJoint() != nullptr)
        toMsg(command_msg.scene_graph_joint, *cmd.getJoint());

      command_msg.scene_graph_prefix = cmd.getPrefix();
      return true;
    }
  }

  return false;
}

bool toMsg(std::vector<tesseract_msgs::EnvironmentCommand>& commands_msg,
           const tesseract_environment::Commands& commands,
           unsigned long past_revision)
{
  for (unsigned long i = past_revision; i < commands.size(); ++i)
  {
    tesseract_msgs::EnvironmentCommand command_msg;
    if (!tesseract_rosutils::toMsg(command_msg, *(commands[i])))
      return false;

    commands_msg.push_back(command_msg);
  }

  return true;
}

tesseract_environment::Commands fromMsg(const std::vector<tesseract_msgs::EnvironmentCommand>& commands_msg)
{
  tesseract_environment::Commands commands;
  commands.reserve(commands_msg.size());
  for (const auto& command : commands_msg)
    commands.push_back(fromMsg(command));

  return commands;
}

tesseract_environment::Command::Ptr fromMsg(const tesseract_msgs::EnvironmentCommand& command_msg)
{
  switch (command_msg.command)
  {
    case tesseract_msgs::EnvironmentCommand::ADD:
    {
      auto l = std::make_shared<tesseract_scene_graph::Link>(fromMsg(command_msg.add_link));
      auto j = std::make_shared<tesseract_scene_graph::Joint>(fromMsg(command_msg.add_joint));
      return std::make_shared<tesseract_environment::AddCommand>(l, j);
    }
    case tesseract_msgs::EnvironmentCommand::MOVE_LINK:
    {
      auto j = std::make_shared<tesseract_scene_graph::Joint>(fromMsg(command_msg.move_link_joint));
      return std::make_shared<tesseract_environment::MoveLinkCommand>(j);
    }
    case tesseract_msgs::EnvironmentCommand::MOVE_JOINT:
    {
      return std::make_shared<tesseract_environment::MoveJointCommand>(command_msg.move_joint_name,
                                                                       command_msg.move_joint_parent_link);
    }
    case tesseract_msgs::EnvironmentCommand::REMOVE_LINK:
    {
      return std::make_shared<tesseract_environment::RemoveLinkCommand>(command_msg.remove_link);
    }
    case tesseract_msgs::EnvironmentCommand::REMOVE_JOINT:
    {
      return std::make_shared<tesseract_environment::RemoveJointCommand>(command_msg.remove_joint);
    }
    case tesseract_msgs::EnvironmentCommand::CHANGE_JOINT_ORIGIN:
    {
      Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
      if (!fromMsg(pose, command_msg.change_joint_origin_pose))
        throw std::runtime_error("Failed to convert pose message to eigen");

      return std::make_shared<tesseract_environment::ChangeJointOriginCommand>(command_msg.change_joint_origin_name,
                                                                               pose);
    }
    case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED:
    {
      return std::make_shared<tesseract_environment::ChangeLinkCollisionEnabledCommand>(
          command_msg.change_link_collision_enabled_name, command_msg.change_link_collision_enabled_value);
    }
    case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_VISIBILITY:
    {
      return std::make_shared<tesseract_environment::ChangeLinkCollisionEnabledCommand>(
          command_msg.change_link_visibility_name, command_msg.change_link_visibility_value);
    }
    case tesseract_msgs::EnvironmentCommand::ADD_ALLOWED_COLLISION:
    {
      return std::make_shared<tesseract_environment::AddAllowedCollisionCommand>(
          command_msg.add_allowed_collision.link_1,
          command_msg.add_allowed_collision.link_2,
          command_msg.add_allowed_collision.reason);
    }
    case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION:
    {
      return std::make_shared<tesseract_environment::RemoveAllowedCollisionCommand>(
          command_msg.remove_allowed_collision.link_1, command_msg.remove_allowed_collision.link_2);
    }
    case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK:
    {
      return std::make_shared<tesseract_environment::RemoveAllowedCollisionLinkCommand>(
          command_msg.remove_allowed_collision_link);
    }
    case tesseract_msgs::EnvironmentCommand::ADD_SCENE_GRAPH:
    {
      auto j = std::make_shared<tesseract_scene_graph::Joint>(fromMsg(command_msg.scene_graph_joint));
      return std::make_shared<tesseract_environment::AddSceneGraphCommand>(
          fromMsg(command_msg.scene_graph), j, command_msg.scene_graph_prefix);
    }
    default:
    {
      throw std::runtime_error("Unsupported command type!");
    }
  }
}

void toMsg(tesseract_msgs::TesseractState& state_msg, const tesseract_environment::Environment& env)
{
  state_msg.id = env.getName();
  state_msg.revision = static_cast<unsigned long>(env.getRevision());
  toMsg(state_msg.commands, env.getCommandHistory(), 0);

  toMsg(state_msg.joint_state, env.getCurrentState()->joints);
}

void toMsg(const tesseract_msgs::TesseractStatePtr& state_msg, const tesseract_environment::Environment& env)
{
  toMsg(*state_msg, env);
}

void toMsg(trajectory_msgs::JointTrajectory& traj_msg,
           const tesseract_environment::EnvState& start_state,
           const std::vector<std::string>& joint_names,
           const Eigen::Ref<const tesseract_common::TrajArray>& traj)
{
  assert(joint_names.size() == static_cast<unsigned>(traj.cols()));

  // Initialze the whole traject with the current state.
  std::map<std::string, int> jn_to_index;
  traj_msg.joint_names.resize(start_state.joints.size());
  traj_msg.points.resize(static_cast<size_t>(traj.rows()));
  for (int i = 0; i < traj.rows(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.resize(start_state.joints.size());

    int j = 0;
    for (const auto& joint : start_state.joints)
    {
      if (i == 0)
      {
        traj_msg.joint_names[static_cast<size_t>(j)] = joint.first;
        jn_to_index[joint.first] = j;
      }
      jtp.positions[static_cast<size_t>(j)] = joint.second;

      ++j;
    }
    jtp.time_from_start = ros::Duration(i);
    traj_msg.points[static_cast<size_t>(i)] = jtp;
  }
  std::size_t jn_to_index_size = jn_to_index.size();

  // Update only the joints which were provided.
  for (int i = 0; i < traj.rows(); ++i)
  {
    for (int j = 0; j < traj.cols(); ++j)
    {
      traj_msg.points[static_cast<size_t>(i)]
          .positions[static_cast<size_t>(jn_to_index[joint_names[static_cast<size_t>(j)]])] = traj(i, j);
    }
  }
  // This will be the case if joint_names[static_cast<size_t>(j)] was not already in the map
  if (jn_to_index_size != jn_to_index.size())
    ROS_WARN("Trying to set joints that are not in the environment. Check that the joint_names are correct");
}

void toMsg(const trajectory_msgs::JointTrajectoryPtr& traj_msg,
           const tesseract_environment::EnvState& start_state,
           const std::vector<std::string>& joint_names,
           const Eigen::Ref<const tesseract_common::TrajArray>& traj)
{
  toMsg(*traj_msg, start_state, joint_names, traj);
}

void toMsg(trajectory_msgs::JointTrajectory& traj_msg, const tesseract_common::JointTrajectory& traj)
{
  assert(traj.joint_names.size() == static_cast<unsigned>(traj.trajectory.cols()));

  // Initialze the whole traject with the current state.
  std::map<std::string, int> jn_to_index;
  traj_msg.joint_names = traj.joint_names;
  traj_msg.points.resize(static_cast<size_t>(traj.trajectory.rows()));

  for (int i = 0; i < traj.trajectory.rows(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.resize(static_cast<size_t>(traj.trajectory.cols()));

    for (int j = 0; j < traj.trajectory.cols(); ++j)
      jtp.positions[static_cast<size_t>(j)] = traj.trajectory(i, j);

    jtp.time_from_start = ros::Duration(i);
    traj_msg.points[static_cast<size_t>(i)] = jtp;
  }
}

void toMsg(const trajectory_msgs::JointTrajectoryPtr& traj_msg, const tesseract_common::JointTrajectory& traj)
{
  toMsg(*traj_msg, traj);
}

bool processMsg(tesseract_environment::Environment& env, const sensor_msgs::JointState& joint_state_msg)
{
  if (!isMsgEmpty(joint_state_msg))
  {
    std::unordered_map<std::string, double> joints;
    for (auto i = 0u; i < joint_state_msg.name.size(); ++i)
    {
      joints[joint_state_msg.name[i]] = joint_state_msg.position[i];
    }
    env.setState(joints);
    return true;
  }
  return false;
}

bool processMsg(tesseract_environment::Environment& env,
                const std::vector<tesseract_msgs::EnvironmentCommand>& env_command_msg)
{
  bool success = true;
  for (const auto& command : env_command_msg)
  {
    switch (command.command)
    {
      case tesseract_msgs::EnvironmentCommand::ADD:
      {
        tesseract_scene_graph::Link link = tesseract_rosutils::fromMsg(command.add_link);
        tesseract_scene_graph::Joint joint = tesseract_rosutils::fromMsg(command.add_joint);
        success &= env.addLink(std::move(link), std::move(joint));
        break;
      }
      case tesseract_msgs::EnvironmentCommand::MOVE_LINK:
      {
        tesseract_scene_graph::Joint joint = tesseract_rosutils::fromMsg(command.move_link_joint);
        success &= env.moveLink(std::move(joint));
        break;
      }
      case tesseract_msgs::EnvironmentCommand::MOVE_JOINT:
      {
        success &= env.moveJoint(command.move_joint_name, command.move_joint_parent_link);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_LINK:
      {
        success &= env.removeLink(command.remove_link);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_JOINT:
      {
        success &= env.removeJoint(command.remove_joint);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_ORIGIN:
      {
        assert(false);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_JOINT_ORIGIN:
      {
        Eigen::Isometry3d pose;
        tf::poseMsgToEigen(command.change_joint_origin_pose, pose);
        success &= env.changeJointOrigin(command.change_joint_origin_name, pose);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED:
      {
        env.setLinkCollisionEnabled(command.change_link_collision_enabled_name,
                                    command.change_link_collision_enabled_value);
        success &= true;
        break;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_VISIBILITY:
      {
        env.setLinkVisibility(command.change_link_visibility_name, command.change_link_visibility_value);
        success &= true;
        break;
      }
      case tesseract_msgs::EnvironmentCommand::ADD_ALLOWED_COLLISION:
      {
        env.addAllowedCollision(command.add_allowed_collision.link_1,
                                command.add_allowed_collision.link_2,
                                command.add_allowed_collision.reason);
        success &= true;
        break;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION:
      {
        env.removeAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2);
        success &= true;
        break;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK:
      {
        env.removeAllowedCollision(command.remove_allowed_collision_link);
        success &= true;
        break;
      }
      case tesseract_msgs::EnvironmentCommand::UPDATE_JOINT_STATE:
      {
        success &= processMsg(env, command.joint_state);
        break;
      }
      case tesseract_msgs::EnvironmentCommand::ADD_SCENE_GRAPH:
      {
        tesseract_scene_graph::SceneGraph g = fromMsg(command.scene_graph);
        if (command.scene_graph_joint.type == tesseract_msgs::Joint::UNKNOWN)
          success &= env.addSceneGraph(g, command.scene_graph_prefix);
        else
          success &= env.addSceneGraph(g, fromMsg(command.scene_graph_joint), command.scene_graph_prefix);

        break;
      }
    }
  }

  return success;
}

bool processMsg(const tesseract_environment::Environment::Ptr& env, const sensor_msgs::JointState& joint_state_msg)
{
  return processMsg(*env, joint_state_msg);
}

bool processMsg(tesseract_environment::Environment& env, const tesseract_msgs::TesseractState& state_msg)
{
  if (state_msg.id != env.getName() || static_cast<unsigned long>(env.getRevision()) > state_msg.revision)
    return false;

  // Only add new commands to environment
  if (env.getRevision() < static_cast<int>(state_msg.revision))
  {
    std::vector<tesseract_msgs::EnvironmentCommand> new_commands;
    new_commands.insert(new_commands.end(), state_msg.commands.begin() + env.getRevision(), state_msg.commands.end());

    if (!processMsg(env, new_commands))
      return false;
  }

  if (!processMsg(env, state_msg.joint_state))
    return false;

  return true;
}

bool processMsg(const tesseract_environment::Environment::Ptr& env, const tesseract_msgs::TesseractState& state_msg)
{
  return processMsg(*env, state_msg);
}

void toMsg(tesseract_msgs::ContactResult& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result,
           const ros::Time& stamp)
{
  contact_result_msg.stamp = stamp;
  contact_result_msg.distance = contact_result.distance;
  contact_result_msg.type_id[0] = static_cast<unsigned char>(contact_result.type_id[0]);
  contact_result_msg.type_id[1] = static_cast<unsigned char>(contact_result.type_id[1]);
  contact_result_msg.link_names[0] = contact_result.link_names[0];
  contact_result_msg.link_names[1] = contact_result.link_names[1];
  contact_result_msg.shape_id[0] = static_cast<size_t>(contact_result.shape_id[0]);
  contact_result_msg.shape_id[1] = static_cast<size_t>(contact_result.shape_id[1]);
  contact_result_msg.subshape_id[0] = static_cast<size_t>(contact_result.subshape_id[0]);
  contact_result_msg.subshape_id[1] = static_cast<size_t>(contact_result.subshape_id[1]);
  contact_result_msg.normal.x = contact_result.normal[0];
  contact_result_msg.normal.y = contact_result.normal[1];
  contact_result_msg.normal.z = contact_result.normal[2];
  contact_result_msg.nearest_points[0].x = contact_result.nearest_points[0][0];
  contact_result_msg.nearest_points[0].y = contact_result.nearest_points[0][1];
  contact_result_msg.nearest_points[0].z = contact_result.nearest_points[0][2];
  contact_result_msg.nearest_points[1].x = contact_result.nearest_points[1][0];
  contact_result_msg.nearest_points[1].y = contact_result.nearest_points[1][1];
  contact_result_msg.nearest_points[1].z = contact_result.nearest_points[1][2];
  contact_result_msg.nearest_points_local[0].x = contact_result.nearest_points_local[0][0];
  contact_result_msg.nearest_points_local[0].y = contact_result.nearest_points_local[0][1];
  contact_result_msg.nearest_points_local[0].z = contact_result.nearest_points_local[0][2];
  contact_result_msg.nearest_points_local[1].x = contact_result.nearest_points_local[1][0];
  contact_result_msg.nearest_points_local[1].y = contact_result.nearest_points_local[1][1];
  contact_result_msg.nearest_points_local[1].z = contact_result.nearest_points_local[1][2];
  toMsg(contact_result_msg.transform[0], contact_result.transform[0]);
  toMsg(contact_result_msg.transform[1], contact_result.transform[1]);
  contact_result_msg.cc_time[0] = contact_result.cc_time[0];
  contact_result_msg.cc_time[1] = contact_result.cc_time[1];
  toMsg(contact_result_msg.cc_transform[0], contact_result.cc_transform[0]);
  toMsg(contact_result_msg.cc_transform[1], contact_result.cc_transform[1]);

  if (contact_result.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
    contact_result_msg.cc_type[0] = tesseract_msgs::ContactResult::CCType_Time0;
  else if (contact_result.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
    contact_result_msg.cc_type[0] = tesseract_msgs::ContactResult::CCType_Time1;
  else if (contact_result.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Between)
    contact_result_msg.cc_type[0] = tesseract_msgs::ContactResult::CCType_Between;
  else
    contact_result_msg.cc_type[0] = tesseract_msgs::ContactResult::CCType_None;

  if (contact_result.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
    contact_result_msg.cc_type[1] = tesseract_msgs::ContactResult::CCType_Time0;
  else if (contact_result.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
    contact_result_msg.cc_type[1] = tesseract_msgs::ContactResult::CCType_Time1;
  else if (contact_result.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Between)
    contact_result_msg.cc_type[1] = tesseract_msgs::ContactResult::CCType_Between;
  else
    contact_result_msg.cc_type[1] = tesseract_msgs::ContactResult::CCType_None;
}

void toMsg(const tesseract_msgs::ContactResultPtr& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result,
           const ros::Time& stamp)
{
  toMsg(*contact_result_msg, contact_result, stamp);
}

bool toMsg(geometry_msgs::PoseArray& pose_array, const tesseract_common::VectorIsometry3d& transforms)
{
  for (const auto& transform : transforms)
  {
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(transform, pose);
    pose_array.poses.push_back(pose);
  }

  return true;
}

tesseract_msgs::ChainGroup toMsg(tesseract_scene_graph::ChainGroups::const_reference group)
{
  tesseract_msgs::ChainGroup g;
  g.name = group.first;
  g.chains.reserve(group.second.size());
  for (const auto& pair : group.second)
  {
    tesseract_msgs::StringPair chain;
    chain.first = pair.first;
    chain.second = pair.second;
    g.chains.push_back(chain);
  }
  return g;
}

tesseract_msgs::GroupsROPKinematics toMsg(tesseract_scene_graph::GroupROPKinematics::const_reference group)
{
  tesseract_msgs::GroupsROPKinematics g;
  g.name = group.first;
  g.manipulator_group = group.second.manipulator_group;
  g.manipulator_ik_solver = group.second.manipulator_ik_solver;
  g.manipulator_reach = group.second.manipulator_reach;
  g.positioner_group = group.second.positioner_group;
  g.positioner_fk_solver = group.second.positioner_fk_solver;

  g.positioner_sample_resolution.reserve(group.second.positioner_sample_resolution.size());
  for (const auto& js : group.second.positioner_sample_resolution)
  {
    tesseract_msgs::StringDoublePair jp;
    jp.first = js.first;
    jp.second = js.second;
    g.positioner_sample_resolution.push_back(jp);
  }
  return g;
}

tesseract_msgs::GroupsREPKinematics toMsg(tesseract_scene_graph::GroupREPKinematics::const_reference group)
{
  tesseract_msgs::GroupsREPKinematics g;
  g.name = group.first;
  g.manipulator_group = group.second.manipulator_group;
  g.manipulator_ik_solver = group.second.manipulator_ik_solver;
  g.manipulator_reach = group.second.manipulator_reach;
  g.positioner_group = group.second.positioner_group;
  g.positioner_fk_solver = group.second.positioner_fk_solver;

  g.positioner_sample_resolution.reserve(group.second.positioner_sample_resolution.size());
  for (const auto& js : group.second.positioner_sample_resolution)
  {
    tesseract_msgs::StringDoublePair jp;
    jp.first = js.first;
    jp.second = js.second;
    g.positioner_sample_resolution.push_back(jp);
  }
  return g;
}

tesseract_msgs::GroupsOPWKinematics toMsg(tesseract_scene_graph::GroupOPWKinematics::const_reference group)
{
  tesseract_msgs::GroupsOPWKinematics g;
  g.name = group.first;
  g.a1 = group.second.a1;
  g.a2 = group.second.a2;
  g.b = group.second.b;
  g.c1 = group.second.c1;
  g.c2 = group.second.c2;
  g.c3 = group.second.c3;
  g.c4 = group.second.c4;

  for (std::size_t i = 0; i < 6; ++i)
  {
    g.offsets[i] = group.second.offsets[i];
    g.sign_corrections[i] = group.second.sign_corrections[i];
  }

  return g;
}

tesseract_msgs::GroupsJointStates toMsg(tesseract_scene_graph::GroupJointStates::const_reference group)
{
  tesseract_msgs::GroupsJointStates g;
  g.name = group.first;

  g.joint_states.reserve(group.second.size());
  for (const auto& gs : group.second)
  {
    tesseract_msgs::GroupsJointState gjs;
    gjs.name = gs.first;
    gjs.joint_state.reserve(gs.second.size());
    for (const auto& s : gs.second)
    {
      tesseract_msgs::StringDoublePair js;
      js.first = s.first;
      js.second = s.second;
      gjs.joint_state.push_back(js);
    }
    g.joint_states.push_back(gjs);
  }

  return g;
}

tesseract_msgs::GroupsTCPs toMsg(tesseract_scene_graph::GroupTCPs::const_reference group)
{
  tesseract_msgs::GroupsTCPs g;
  g.name = group.first;

  g.tcps.reserve(group.second.size());
  for (const auto& gs : group.second)
  {
    tesseract_msgs::GroupsTCP gtcp;
    gtcp.name = gs.first;
    toMsg(gtcp.tcp, gs.second);
  }
  return g;
}

bool toMsg(tesseract_msgs::KinematicsInformation& kin_info, const tesseract::ManipulatorManager& manager)
{
  for (const auto& group_name : kin_info.group_names)
  {
    auto fk_solver = manager.getFwdKinematicSolver(group_name);
    if (fk_solver != nullptr)
    {
      tesseract_msgs::StringPair pair;
      pair.first = group_name;
      pair.second = fk_solver->getSolverName();
      kin_info.default_fwd_kin.push_back(pair);
    }

    auto ik_solver = manager.getInvKinematicSolver(group_name);
    if (ik_solver != nullptr)
    {
      tesseract_msgs::StringPair pair;
      pair.first = group_name;
      pair.second = ik_solver->getSolverName();
      kin_info.default_inv_kin.push_back(pair);
    }
  }

  kin_info.chain_groups.reserve(manager.getChainGroups().size());
  for (const auto& group : manager.getChainGroups())
    kin_info.chain_groups.push_back(toMsg(group));

  kin_info.joint_groups.reserve(manager.getJointGroups().size());
  for (const auto& group : manager.getJointGroups())
  {
    tesseract_msgs::JointGroup g;
    g.name = group.first;
    g.joints.reserve(group.second.size());
    for (const auto& joint_name : group.second)
      g.joints.push_back(joint_name);

    kin_info.joint_groups.push_back(g);
  }

  kin_info.link_groups.reserve(manager.getLinkGroups().size());
  for (const auto& group : manager.getLinkGroups())
  {
    tesseract_msgs::LinkGroup g;
    g.name = group.first;
    g.links.reserve(group.second.size());
    for (const auto& link_name : group.second)
      g.links.push_back(link_name);

    kin_info.link_groups.push_back(g);
  }

  kin_info.group_rop.reserve(manager.getROPKinematicsSolvers().size());
  for (const auto& group : manager.getROPKinematicsSolvers())
    kin_info.group_rop.push_back(toMsg(group));

  kin_info.group_rep.reserve(manager.getREPKinematicsSolvers().size());
  for (const auto& group : manager.getREPKinematicsSolvers())
    kin_info.group_rep.push_back(toMsg(group));

  kin_info.group_opw.reserve(manager.getOPWKinematicsSolvers().size());
  for (const auto& group : manager.getOPWKinematicsSolvers())
    kin_info.group_opw.push_back(toMsg(group));

  kin_info.group_joint_states.reserve(manager.getGroupJointStates().size());
  for (const auto& group : manager.getGroupJointStates())
    kin_info.group_joint_states.push_back(toMsg(group));

  kin_info.group_tcps.reserve(manager.getGroupTCPs().size());
  for (const auto& group : manager.getGroupTCPs())
    kin_info.group_tcps.push_back(toMsg(group));

  return true;
}

bool fromMsg(tesseract::ManipulatorManager& manager, const tesseract_msgs::KinematicsInformation& kin_info)
{
  for (const auto& group : kin_info.chain_groups)
  {
    tesseract_scene_graph::ChainGroup chain_group;
    for (const auto& pair : group.chains)
      chain_group.emplace_back(pair.first, pair.second);
    manager.addChainGroup(group.name, chain_group);
  }

  for (const auto& group : kin_info.joint_groups)
    manager.addJointGroup(group.name, group.joints);

  for (const auto& group : kin_info.link_groups)
    manager.addLinkGroup(group.name, group.links);

  for (const auto& group : kin_info.group_rop)
  {
    tesseract_scene_graph::ROPKinematicParameters rop_group;
    rop_group.manipulator_group = group.manipulator_group;
    rop_group.manipulator_ik_solver = group.manipulator_ik_solver;
    rop_group.manipulator_reach = group.manipulator_reach;
    rop_group.positioner_group = group.positioner_group;
    rop_group.positioner_fk_solver = group.positioner_fk_solver;
    for (const auto& pair : group.positioner_sample_resolution)
      rop_group.positioner_sample_resolution[pair.first] = pair.second;
    manager.addROPKinematicsSolver(group.name, rop_group);
  }

  for (const auto& group : kin_info.group_rep)
  {
    tesseract_scene_graph::REPKinematicParameters rep_group;
    rep_group.manipulator_group = group.manipulator_group;
    rep_group.manipulator_ik_solver = group.manipulator_ik_solver;
    rep_group.manipulator_reach = group.manipulator_reach;
    rep_group.positioner_group = group.positioner_group;
    rep_group.positioner_fk_solver = group.positioner_fk_solver;
    for (const auto& pair : group.positioner_sample_resolution)
      rep_group.positioner_sample_resolution[pair.first] = pair.second;
    manager.addREPKinematicsSolver(group.name, rep_group);
  }

  for (const auto& group : kin_info.group_opw)
  {
    tesseract_scene_graph::OPWKinematicParameters opw_group;
    opw_group.a1 = group.a1;
    opw_group.a2 = group.a2;
    opw_group.b = group.b;
    opw_group.c1 = group.c1;
    opw_group.c2 = group.c2;
    opw_group.c3 = group.c3;
    opw_group.c4 = group.c4;
    for (std::size_t i = 0; i < 6; ++i)
    {
      opw_group.offsets[i] = group.offsets[i];
      opw_group.sign_corrections[i] = group.sign_corrections[i];
    }

    manager.addOPWKinematicsSolver(group.name, opw_group);
  }

  for (const auto& group : kin_info.group_joint_states)
  {
    for (const auto& state : group.joint_states)
    {
      tesseract_scene_graph::GroupsJointState joint_state;
      joint_state.reserve(state.joint_state.size());
      for (const auto& js : state.joint_state)
        joint_state[js.first] = js.second;

      manager.addGroupJointState(group.name, state.name, joint_state);
    }
  }

  for (const auto& group : kin_info.group_tcps)
  {
    for (const auto& pose : group.tcps)
    {
      Eigen::Isometry3d tcp{ Eigen::Isometry3d::Identity() };
      fromMsg(tcp, pose.tcp);

      manager.addGroupTCP(group.name, pose.name, tcp);
    }
  }

  for (const auto& d : kin_info.default_fwd_kin)
    manager.setDefaultFwdKinematicSolver(d.first, d.second);

  for (const auto& d : kin_info.default_inv_kin)
    manager.setDefaultInvKinematicSolver(d.first, d.second);

  return true;
}

bool toMsg(tesseract_msgs::TransformMap& transform_map_msg, const tesseract_common::TransformMap& transform_map)
{
  transform_map_msg.names.reserve(transform_map.size());
  transform_map_msg.transforms.reserve(transform_map.size());
  for (const auto& pair : transform_map)
  {
    transform_map_msg.names.push_back(pair.first);
    geometry_msgs::Pose pose;
    if (!toMsg(pose, pair.second))
      return false;

    transform_map_msg.transforms.push_back(pose);
  }
  return true;
}

bool fromMsg(tesseract_common::TransformMap& transform_map, const tesseract_msgs::TransformMap& transform_map_msg)
{
  if (transform_map_msg.names.size() != transform_map_msg.transforms.size())
    return false;

  for (std::size_t i = 0; i < transform_map_msg.names.size(); ++i)
  {
    Eigen::Isometry3d pose;
    if (fromMsg(pose, transform_map_msg.transforms.at(i)))
      return false;

    transform_map[transform_map_msg.names.at(i)] = pose;
  }

  return true;
}

bool toMsg(sensor_msgs::JointState& joint_state_msg, const std::unordered_map<std::string, double>& joint_state)
{
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.name.reserve(joint_state.size());
  joint_state_msg.position.reserve(joint_state.size());
  for (const auto& pair : joint_state)
  {
    joint_state_msg.name.push_back(pair.first);
    joint_state_msg.position.push_back(pair.second);
  }
  return true;
}

bool fromMsg(std::unordered_map<std::string, double>& joint_state, const sensor_msgs::JointState& joint_state_msg)
{
  if (joint_state_msg.name.size() != joint_state_msg.position.size())
    return false;

  for (std::size_t i = 0; i < joint_state_msg.name.size(); ++i)
    joint_state[joint_state_msg.name.at(i)] = joint_state_msg.position.at(i);

  return true;
}

bool toMsg(tesseract_msgs::Tesseract& tesseract_msg, const tesseract::Tesseract& tesseract)
{
  if (!tesseract_rosutils::toMsg(tesseract_msg.command_history, tesseract.getEnvironment()->getCommandHistory(), 0))
  {
    return false;
  }

  auto manipulator_manager = tesseract.getManipulatorManager();
  if (!tesseract_rosutils::toMsg(tesseract_msg.kinematics_information, *manipulator_manager))
  {
    return false;
  }

  if (!tesseract_rosutils::toMsg(tesseract_msg.joint_states, tesseract.getEnvironment()->getCurrentState()->joints))
  {
    return false;
  }

  return true;
}

tesseract::Tesseract::Ptr fromMsg(const tesseract_msgs::Tesseract& tesseract_msg)
{
  tesseract_environment::Commands commands;
  try
  {
    commands = tesseract_rosutils::fromMsg(tesseract_msg.command_history);
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Failed to convert command history message!");
    return nullptr;
  }

  auto env = std::make_shared<tesseract_environment::Environment>();
  if (!env->init<tesseract_environment::OFKTStateSolver>(commands))  // TODO: Get state solver
  {
    ROS_ERROR_STREAM("Failed to initialize environment!");
    return nullptr;
  }

  auto env_state = std::make_shared<tesseract_environment::EnvState>();
  if (!tesseract_rosutils::fromMsg(env_state->joints, tesseract_msg.joint_states))
  {
    ROS_ERROR_STREAM("Failed to get joint states");
    return nullptr;
  }
  env->setState(env_state->joints);

  auto manip_manager = std::make_shared<tesseract::ManipulatorManager>();
  auto srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  manip_manager->init(env, srdf);

  if (!tesseract_rosutils::fromMsg(*manip_manager, tesseract_msg.kinematics_information))
  {
    ROS_ERROR_STREAM("Failed to populate manipulator manager from kinematics information!");
    return nullptr;
  }

  auto thor = std::make_shared<tesseract::Tesseract>();
  if (!thor->init(*env, *manip_manager))
  {
    ROS_ERROR_STREAM("Failed to initialize tesseract!");
    return nullptr;
  }

  return thor;
}

}  // namespace tesseract_rosutils