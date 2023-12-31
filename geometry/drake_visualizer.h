#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_version.h"
#include "drake/geometry/query_object.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/input_port.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {

/* TODO(SeanCurtis-TRI): Here's the stuff that needs to happen:

  - Static method that adds this to a diagram (including connections) so that
    it can be done in a single call.
*/

/** The set of parameters for configuring DrakeVisualizer.  */
struct DrakeVisualizerParams {
  /** The duration (in seconds) between published LCM messages that update the
   poses of the scene's geometry.  */
  double publish_period{1 / 60.0};

  /* The role of the geometries to be sent to the visualizer.  */
  Role role{Role::kIllustration};

  /** The color to apply to any geometry that hasn't defined one.  */
  Rgba default_color{0.9, 0.9, 0.9, 1.0};
};

/** A system that publishes LCM messages compatible with the `drake_visualizer`
 application representing the current state of a SceneGraph instance (whose
 QueryObject-valued output port is connected to this system's input port).

 @system
 name: DrakeVisualizer
 input_ports:
 - query_object
 @endsystem

 The %DrakeVisualizer system broadcasts two kinds of LCM messages:

   - a message that defines the geometry in the world on the lcm channel named
     "DRAKE_VIEWER_DRAW",
   - a message that updates the poses of those geometries on the lcm channel
     named "DRAKE_VIEWER_LOAD_ROBOT"

 The system uses the versioning mechanism provided by SceneGraph to detect
 changes to the geometry so that a change in SceneGraph's data will propagate
 to `drake_visualizer`.

 @anchor drake_visualizer_role_consumer
 <h3>Visualization by Role</h3>

 By default, %DrakeVisualizer visualizes geometries with the illustration role
 (see @ref geometry_roles for more details). It can be configured to visualize
 geometries with other roles (see DrakeVisualizerParams). Only one role can be
 specified.

 The appearance of the geometry in the visualizer is typically defined by the
 the geometry's properties for the visualized role.

   - For the visualized role, if a geometry has the ("phong", "diffuse")
     property described in the table below, that value is used.
   - Otherwise, if the geometry *also* has the illustration properties, those
     properties are likewise tested for the ("phong", "diffuse") property. This
     rule only has significance is the visualized role is *not* the illustration
     role.
   - Otherwise, the configured default color will be applied (see
     DrakeVisualizerParams).

 | Group name | Required | Property Name |  Property Type  | Property Description |
 | :--------: | :------: | :-----------: | :-------------: | :------------------- |
 |    phong   | no       | diffuse       |     Rgba        | The rgba value of the object surface. |

 <h4>Appearance of OBJ files</h4>

 Meshes represented by OBJ are special. The OBJ file can reference a material
 file (.mtl). If found by `drake_visualizer`, the values in the .mtl will take
 precedence over the ("phong", "diffuse") geometry property.

 It's worth emphasizing that these rules permits control over the appearance of
 collision geometry on a per-geometry basis by assigning an explicit Rgba value
 to the ("phong", "diffuse") property in the geometry's ProximityProperties.

 @note If collision geometries are added to SceneGraph by parsing URDF/SDF
 files, they will not have diffuse values. Even if elements were added to the
 specification files, they would not be parsed. They must be added to the
 geometries after parsing.

 <h3>Effective visualization</h3>

 The best visualization is when draw messages have been preceded by a compatible
 load message (i.e., a "coherent" message sequence). While LCM doesn't guarantee
 that messages will be received/processed in the same order as they are
 broadcast, your results will be best if %DrakeVisualizer is allowed to
 broadcast coherent messages. Practices that interfere with that will likely
 produce undesirable results. E.g.,

   - Evaluating a single instance of %DrakeVisualizer across several threads,
     such that the data in the per-thread systems::Context varies.
   - Evaluating multiple instances of %DrakeVisualizer in a single thread that
     share the same lcm::DrakeLcmInterface.
*/
class DrakeVisualizer : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeVisualizer)

  /** Creates an instance of %DrakeVisualizer.

   @param lcm     An optional LCM interface. If none is provided, this system
                  will allocate its own instance. If one is provided it must
                  remain valid for the lifetime of this object.
   @param params  The set of parameters to control this system's behavior.
   @throws std::exception if `params.publish_period <= 0`.
   @throws std::exception if `params.role == Role::kUnassigned`.
   */
  DrakeVisualizer(lcm::DrakeLcmInterface* lcm = nullptr,
                  DrakeVisualizerParams params = {});

  /** Returns the QueryObject-valued input port. It should be connected to
   SceneGraph's QueryObject-valued output port. Failure to do so will cause a
   runtime error when attempting to broadcast messages.  */
  const systems::InputPort<double>& query_object_input_port() const {
    return this->get_input_port(query_object_input_port_);
  }

 private:
  friend class DrakeVisualizerTest;

  /* The periodic event handler. It tests to see if the last scene description
   is valid (if not, updates it) and then broadcasts poses.  */
  void SendGeometryMessage(const systems::Context<double>& context) const;

  /* Dispatches a "load geometry" message; replacing the whole scene with the
   current scene.  */
  void SendLoadMessage(const systems::Context<double>& context) const;

  /* Dispatches a "update pose" message for geometry that is known to have been
   loaded.  */
  void SendPoseMessage(const systems::Context<double>& context) const;

  /* Data stored in the cache; populated when we transmit a load message and
   read from for a pose message.  */
  struct DynamicFrameData {
    FrameId frame_id;
    int num_geometry{};
    std::string name;
  };

  /* Identifies all of the frames with dynamic data and stores them (with
   additional data) in the given vector `frame_data`. */
  void CalcDynamicFrameData(const systems::Context<double>& context,
                            std::vector<DynamicFrameData>* frame_data) const;

  /* DrakeVisualizer stores a "model" of what it thinks is registered in the
   drake_visualizer application. Because drake_visualizer is not part of the
   Drake state, this model is likewise not part of the Drake state. It is a
   property of the system. This allows arbitrary changes to the context but
   DrakeVisualizer can still make its *best effort* to ensure that
   drake_visualizer state is consistent with the messages it is about to send.
   Because of the nature of lcm messages, it cannot make guarantees; lcm
   messages can arrive in a different order than they were broadcast.

   To this end, DrakeVisualizer has the model (GeometryVersion) and a
   mutex that will allow updating that model safely. Beyond that, there are
   no guarantees about order of operations when the publish callback is
   invoked across multiple threads.  */

  /* The version of the geometry that was last loaded (i.e., had a load message
   sent). If the version found on the input port differs from this value, a
   new load message will be sent prior to the "draw" message.  */
  mutable GeometryVersion version_;
  mutable std::mutex mutex_;

  /* The index of this System's QueryObject-valued input port.  */
  int query_object_input_port_{};

  /* The LCM interface: the owned (if such exists) and the active interface
   (whether owned or not). The active interface is mutable because we non-const
   access to the LCM interface in const System methods.  */
  std::unique_ptr<lcm::DrakeLcmInterface> owned_lcm_{};
  mutable lcm::DrakeLcmInterface* lcm_{};

  /* The index of the cache entry that stores the dynamic frame data.  */
  systems::CacheIndex dynamic_data_cache_index_{};

  /* The parameters for the visualizer.  */
  DrakeVisualizerParams params_;
};

}  // namespace geometry
}  // namespace drake
