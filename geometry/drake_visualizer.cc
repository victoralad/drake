#include "drake/geometry/drake_visualizer.h"

#include <utility>
#include <vector>

#include "drake/common/value.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {

using Eigen::Quaterniond;
using lcm::DrakeLcm;
using math::RigidTransformd;
using std::make_unique;
using std::vector;
using systems::Context;

namespace {

// Simple class for converting shape specifications into LCM-compatible shapes.
class ShapeToLcm : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ShapeToLcm)

  ShapeToLcm() = default;
  ~ShapeToLcm() override = default;

  lcmt_viewer_geometry_data Convert(const Shape& shape,
                                    const RigidTransformd& X_PG,
                                    const Rgba& in_color) {
    X_PG_ = X_PG;
    // NOTE: Reify *may* change X_PG_ based on the shape. For example, the
    // half-space requires an additional offset to shift the box representing
    // the plane *to* the plane.
    shape.Reify(this);

    // Saves the location and orientation of the visualization geometry in the
    // `lcmt_viewer_geometry_data` object. The location and orientation are
    // specified in the body's frame.
    Eigen::Map<Eigen::Vector3f> position(geometry_data_.position);
    position = X_PG_.translation().cast<float>();
    // LCM quaternion must be w, x, y, z.
    Quaterniond q(X_PG_.rotation().ToQuaternion());
    geometry_data_.quaternion[0] = q.w();
    geometry_data_.quaternion[1] = q.x();
    geometry_data_.quaternion[2] = q.y();
    geometry_data_.quaternion[3] = q.z();

    Eigen::Map<Eigen::Vector4f> color(geometry_data_.color);
    Eigen::Vector4d color_vec(in_color.r(), in_color.g(), in_color.b(),
                              in_color.a());
    color = color_vec.cast<float>();
    return geometry_data_;
  }

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere& sphere, void*) override {
    geometry_data_.type = geometry_data_.SPHERE;
    geometry_data_.num_float_data = 1;
    geometry_data_.float_data.push_back(static_cast<float>(sphere.radius()));
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void*) override {
    geometry_data_.type = geometry_data_.ELLIPSOID;
    geometry_data_.num_float_data = 3;
    geometry_data_.float_data.push_back(static_cast<float>(ellipsoid.a()));
    geometry_data_.float_data.push_back(static_cast<float>(ellipsoid.b()));
    geometry_data_.float_data.push_back(static_cast<float>(ellipsoid.c()));
  }

  void ImplementGeometry(const Cylinder& cylinder, void*) override {
    geometry_data_.type = geometry_data_.CYLINDER;
    geometry_data_.num_float_data = 2;
    geometry_data_.float_data.push_back(static_cast<float>(cylinder.radius()));
    geometry_data_.float_data.push_back(static_cast<float>(cylinder.length()));
  }

  void ImplementGeometry(const HalfSpace&, void*) override {
    // Currently representing a half space as a big box. This assumes that the
    // underlying box representation is centered on the origin.
    geometry_data_.type = geometry_data_.BOX;
    geometry_data_.num_float_data = 3;
    // Box width, height, and thickness.
    geometry_data_.float_data.push_back(50);
    geometry_data_.float_data.push_back(50);
    const float thickness = 1;
    geometry_data_.float_data.push_back(thickness);

    // The final pose of the box is the half-space's pose pre-multiplied by
    // an offset sufficient to move the box down so it's top face lies on the
    // z = 0 plane.
    // Shift it down so that the origin lies on the top surface.
    RigidTransformd box_xform{Eigen::Vector3d{0, 0, -thickness / 2}};
    X_PG_ = X_PG_ * box_xform;
  }

  void ImplementGeometry(const Box& box, void*) override {
    geometry_data_.type = geometry_data_.BOX;
    geometry_data_.num_float_data = 3;
    // Box width, depth, and height.
    geometry_data_.float_data.push_back(static_cast<float>(box.width()));
    geometry_data_.float_data.push_back(static_cast<float>(box.depth()));
    geometry_data_.float_data.push_back(static_cast<float>(box.height()));
  }

  void ImplementGeometry(const Capsule& capsule, void*) override {
    geometry_data_.type = geometry_data_.CAPSULE;
    geometry_data_.num_float_data = 2;
    geometry_data_.float_data.push_back(static_cast<float>(capsule.radius()));
    geometry_data_.float_data.push_back(static_cast<float>(capsule.length()));
  }

  void ImplementGeometry(const Mesh& mesh, void*) override {
    geometry_data_.type = geometry_data_.MESH;
    geometry_data_.num_float_data = 3;
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.string_data = mesh.filename();
  }

  // For visualization, Convex is the same as Mesh.
  void ImplementGeometry(const Convex& mesh, void*) override {
    geometry_data_.type = geometry_data_.MESH;
    geometry_data_.num_float_data = 3;
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.string_data = mesh.filename();
  }

 private:
  lcmt_viewer_geometry_data geometry_data_{};
  // The transform from the geometry frame to its parent frame.
  RigidTransformd X_PG_;
};

}  // namespace

DrakeVisualizer::DrakeVisualizer(lcm::DrakeLcmInterface* lcm,
                                 DrakeVisualizerParams params)
    : LeafSystem<double>(),
      owned_lcm_(lcm ? nullptr : new DrakeLcm()),
      lcm_(lcm ? lcm : owned_lcm_.get()),
      params_(std::move(params)) {
  if (params_.publish_period <= 0) {
    throw std::runtime_error(fmt::format(
        "DrakeVisualizer requires a positive publish period; {} was given",
        params_.publish_period));
  }
  if (params_.role == Role::kUnassigned) {
    throw std::runtime_error(
        "DrakeVisualizer cannot be be used for geometries with the "
        "Role::kUnassigned value. Please choose proximity, perception, or "
        "illustration");
  }

  DeclarePeriodicPublishEvent(params_.publish_period, 0.0,
                              &DrakeVisualizer::SendGeometryMessage);

  query_object_input_port_ =
      this->DeclareAbstractInputPort("query_object",
                                     Value<QueryObject<double>>())
          .get_index();

  // This cache entry depends on *nothing*.
  dynamic_data_cache_index_ =
      DeclareCacheEntry("dynamic_frames", vector<DynamicFrameData>(),
                        &DrakeVisualizer::CalcDynamicFrameData,
                        {nothing_ticket()})
          .cache_index();
}

void DrakeVisualizer::SendGeometryMessage(
    const Context<double>& context) const {
  const auto& query_object =
      query_object_input_port().Eval<QueryObject<double>>(context);
  const GeometryVersion& current_version =
      query_object.inspector().geometry_version();

  bool send_load_message = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!version_.IsSameAs(current_version, params_.role)) {
      send_load_message = true;
      version_ = current_version;
    }
  }
  if (send_load_message) {
    SendLoadMessage(context);
  }
  SendPoseMessage(context);
}

void DrakeVisualizer::SendLoadMessage(const Context<double>& context) const {
  const auto& query_object =
      query_object_input_port().Eval<QueryObject<double>>(context);
  const SceneGraphInspector<double>& inspector = query_object.inspector();

  lcmt_viewer_load_robot message{};

  // We'll need to make sure our knowledge of dynamic frames can get updated.
  get_cache_entry(dynamic_data_cache_index_)
      .get_mutable_cache_entry_value(context)
      .mark_out_of_date();
  const vector<DynamicFrameData>& dynamic_frames =
      get_cache_entry(dynamic_data_cache_index_)
          .Eval<vector<DynamicFrameData>>(context);
  // Add the world frame if it has geometries with the specified role.
  const int anchored_count = inspector.NumGeometriesForFrameWithRole(
      inspector.world_frame_id(), params_.role);
  const int frame_count =
      static_cast<int>(dynamic_frames.size()) + (anchored_count > 0 ? 1 : 0);

  message.num_links = frame_count;
  message.link.resize(frame_count);

  // Helper utility to create lcm geometry description from geometry id.
  auto make_geometry = [this, &inspector](GeometryId g_id) {
    const GeometryProperties* props =
        inspector.GetProperties(g_id, params_.role);
    // We assume that the g_id was obtained by asking for geometries with the
    // indicated role. So, by definition, the properties should be non-null.
    DRAKE_ASSERT(props != nullptr);
    const Shape& shape = inspector.GetShape(g_id);
    Rgba default_color = params_.default_color;
    if (params_.role != Role::kIllustration) {
      const GeometryProperties* illus_props =
          inspector.GetIllustrationProperties(g_id);
      if (illus_props) {
        default_color = illus_props->GetPropertyOrDefault("phong", "diffuse",
                                                          default_color);
      }
    }
    const Rgba& color =
        props->GetPropertyOrDefault("phong", "diffuse", default_color);
    return ShapeToLcm().Convert(shape, inspector.GetPoseInFrame(g_id), color);
  };

  int link_index = 0;
  // Load anchored geometry into the world frame.
  if (anchored_count) {
    message.link[0].name = "world";
    message.link[0].robot_num = 0;
    message.link[0].num_geom = anchored_count;
    message.link[0].geom.resize(anchored_count);
    int geom_index = -1;  // We'll pre-increment before using.
    for (const GeometryId& g_id :
         inspector.GetGeometries(inspector.world_frame_id(), params_.role)) {
      message.link[0].geom[++geom_index] = make_geometry(g_id);
    }
    link_index = 1;
  }

  // Load dynamic geometry into their own frames.
  for (const auto& [frame_id, geometry_count, name] : dynamic_frames) {
    message.link[link_index].name = name;
    message.link[link_index].robot_num = inspector.GetFrameGroup(frame_id);
    message.link[link_index].num_geom = geometry_count;
    message.link[link_index].geom.resize(geometry_count);
    int geom_index = -1;  // We'll pre-increment before using.
    for (const GeometryId& g_id :
         inspector.GetGeometries(frame_id, params_.role)) {
      message.link[link_index].geom[++geom_index] = make_geometry(g_id);
    }
    ++link_index;
  }

  lcm::Publish(lcm_, "DRAKE_VIEWER_LOAD_ROBOT", message, context.get_time());
}

void DrakeVisualizer::SendPoseMessage(const Context<double>& context) const {
  const auto& query_object =
      query_object_input_port().Eval<QueryObject<double>>(context);
  const SceneGraphInspector<double>& inspector = query_object.inspector();

  lcmt_viewer_draw message{};

  const vector<DynamicFrameData>& dynamic_frames =
      get_cache_entry(dynamic_data_cache_index_)
          .Eval<vector<DynamicFrameData>>(context);
  const int frame_count = static_cast<int>(dynamic_frames.size());

  message.timestamp = static_cast<int64_t>(context.get_time() * 1000.0);
  message.num_links = frame_count;
  message.link_name.resize(frame_count);
  message.robot_num.resize(frame_count);
  message.position.resize(frame_count);
  message.quaternion.resize(frame_count);

  for (int i = 0; i < frame_count; ++i) {
    const FrameId frame_id = dynamic_frames[i].frame_id;
    message.robot_num[i] = inspector.GetFrameGroup(frame_id);
    message.link_name[i] = dynamic_frames[i].name;

    const RigidTransformd& X_WF = query_object.GetPoseInWorld(frame_id);
    message.position[i].resize(3);
    message.position[i][0] = X_WF.translation()[0];
    message.position[i][1] = X_WF.translation()[1];
    message.position[i][2] = X_WF.translation()[2];

    const Eigen::Quaternion<double> q = X_WF.rotation().ToQuaternion();
    message.quaternion[i].resize(4);
    message.quaternion[i][0] = q.w();
    message.quaternion[i][1] = q.x();
    message.quaternion[i][2] = q.y();
    message.quaternion[i][3] = q.z();
  }

  lcm::Publish(lcm_, "DRAKE_VIEWER_DRAW", message, context.get_time());
}

void DrakeVisualizer::CalcDynamicFrameData(
    const Context<double>& context,
    vector<DynamicFrameData>* frame_data) const {
  const auto& query_object =
      query_object_input_port().Eval<QueryObject<double>>(context);
  const SceneGraphInspector<double>& inspector = query_object.inspector();
  // Collect the dynamic frames that actually have geometries of the
  // specified role. These are the frames broadcast in a draw message and are
  // also part of the load message (plus possibly the world frame).
  vector<DynamicFrameData>& dynamic_frames = *frame_data;
  dynamic_frames.clear();

  for (const FrameId& frame_id : inspector.all_frame_ids()) {
    // We'll handle the world frame special.
    if (frame_id == inspector.world_frame_id()) continue;
    const int count =
        inspector.NumGeometriesForFrameWithRole(frame_id, params_.role);
    if (count > 0) {
      dynamic_frames.push_back({frame_id, count,
                                inspector.GetOwningSourceName(frame_id) +
                                    "::" + inspector.GetName(frame_id)});
    }
  }
}

}  // namespace geometry
}  // namespace drake
