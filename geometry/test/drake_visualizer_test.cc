#include "drake/geometry/drake_visualizer.h"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcm_messages.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {

using Eigen::Vector3d;
using math::RigidTransformd;
using std::make_unique;
using std::map;
using std::move;
using std::string;
using std::unique_ptr;
using std::vector;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;

namespace {

/* Serves as a source of pose values for SceneGraph input ports. */
class PoseSource : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSource)
  PoseSource() {
    this->DeclareAbstractOutputPort(FramePoseVector<double>(),
                                    &PoseSource::ReadPoses);
  }

  void SetPoses(FramePoseVector<double> poses) { poses_ = move(poses); }

 private:
  void ReadPoses(const Context<double>&, FramePoseVector<double>* poses) const {
    *poses = poses_;
  }

  FramePoseVector<double> poses_;
};

}  // namespace

// TODO(SeanCurtis-TRI): These unit tests aren't complete. Much of the DUT has
//  code from the old `geometry_visualizer.{h|cc}. That code wasn't particularly
//  tested either, but has been run thousands of times. That code lives on in
//  DrakeVisualizer and the tests still leave out the following aspects:
//
//    1. Mapping from shape specification to LCM equivalent (i.e., spheres are
//       visualized as spheres, boxes as boxes, etc.)
//    2. The poses of geometries (both relative to their parent frame and the
//       frame in the world).
//
// The assumption is that if these were wrong, it would immediately apparent to
// every user. If this ever changes, it might be worthwhile to test these
// things.


/* Note: We're not using an anonymous namespace so the DrakeVisualizerTest
 satisfies the friend declaration in DrakeVisualizer.  */

/* Infrastructure for testing the DrakeVisualizer. */
class DrakeVisualizerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_EQ(lcm_.get_lcm_url(), "memq://");
  }

  /* Returns the pointer to the `visualizer`'s owned lcm interface (may be
   nullptr). */
  static const lcm::DrakeLcmInterface* get_owned_interface(
      const DrakeVisualizer& visualizer) {
    return visualizer.owned_lcm_.get();
  }

  /* Returns the pointer to the `visualizer`'s active lcm interface -- the one
   being invoked to do work (can't be nullptr). */
  static const lcm::DrakeLcmInterface* get_active_interface(
      const DrakeVisualizer& visualizer) {
    return visualizer.lcm_;
  }

  /* Configures the diagram (and raw pointers) with a DrakeVisualizer configured
   by the given parameters.  */
  void ConfigureDiagram(double period = kPublishPeriod,
                        Role role = Role::kIllustration,
                        const Rgba& default_color = Rgba{0.1, 0.2, 0.3, 0.4}) {
    DrakeVisualizerParams params{period, role, default_color};
    DiagramBuilder<double> builder;
    scene_graph_ = builder.AddSystem<SceneGraph<double>>();
    visualizer_ = builder.AddSystem<DrakeVisualizer>(&lcm_, move(params));
    builder.Connect(scene_graph_->get_query_output_port(),
                    visualizer_->query_object_input_port());
    source_id_ = scene_graph_->RegisterSource(kSourceName);
    pose_source_ = builder.AddSystem<PoseSource>();
    builder.Connect(pose_source_->get_output_port(0),
                    scene_graph_->get_source_pose_port(source_id_));
    diagram_ = builder.Build();
  }

  /* Populates SceneGraph with various objects. This function's defining
   characteristic is that there are three different geometries, each with a
   different role, such that there will always be one geometry included,
   regardless of role.  */
  void PopulateScene() {
    // A box has perception properties with a green color.
    const FrameId f0_id =
        scene_graph_->RegisterFrame(source_id_, GeometryFrame("perception", 0));
    const GeometryId g0_id = scene_graph_->RegisterGeometry(
        source_id_, f0_id,
        make_unique<GeometryInstance>(RigidTransformd{},
                                      make_unique<Box>(1, 1, 1), "perception"));
    PerceptionProperties percep_prop;
    percep_prop.AddProperty("phong", "diffuse", Rgba(0, 1, 0, 1));
    scene_graph_->AssignRole(source_id_, g0_id, percep_prop);

    // A cylinder has illustration properties with a blue color.
    const FrameId f1_id = scene_graph_->RegisterFrame(
        source_id_, GeometryFrame("illustration", 1));
    const GeometryId g1_id = scene_graph_->RegisterGeometry(
        source_id_, f1_id,
        make_unique<GeometryInstance>(
            RigidTransformd{}, make_unique<Cylinder>(1, 1), "illustration"));
    IllustrationProperties illus_prop;
    illus_prop.AddProperty("phong", "diffuse", Rgba(0, 1, 0, 1));
    scene_graph_->AssignRole(source_id_, g1_id, illus_prop);

    // A sphere has proximity properties with no color.
    const FrameId f2_id =
        scene_graph_->RegisterFrame(source_id_, GeometryFrame("proximity", 2));
    const GeometryId g2_id = scene_graph_->RegisterGeometry(
        source_id_, f2_id,
        make_unique<GeometryInstance>(RigidTransformd{}, make_unique<Sphere>(1),
                                      "proximity"));
    scene_graph_->AssignRole(source_id_, g2_id, ProximityProperties());

    pose_source_->SetPoses({{f0_id, RigidTransformd{}},
                            {f1_id, RigidTransformd{}},
                            {f2_id, RigidTransformd{}}});
  }

  /* Collection of data for reporting what is waiting in the LCM queue.  */
  struct MessageResults {
    int num_load{};
    // Zeroed out unless num_load > 0.
    lcmt_viewer_load_robot load_message{};
    int num_draw{};
    // Zeroed out unless num_draw > 0.
    lcmt_viewer_draw draw_message{};
  };

  /* Processes the message queue, reporting the number of load and draw messges
   and, if that number is non-zero, the last message of each type received.  */
  MessageResults ProcessMessages() {
    lcm_.HandleSubscriptions(0);
    const int num_draw = draw_subscriber_.count();
    lcmt_viewer_draw draw_message = draw_subscriber_.message();
    const int num_load = load_subscriber_.count();
    lcmt_viewer_load_robot load_message = load_subscriber_.message();
    draw_subscriber_.clear();
    load_subscriber_.clear();
    return {num_load, load_message, num_draw, draw_message};
  }

  /* Handles all subscribers and confirms the number of load and draw messages
   are as expected. Clears the subscribers after looking. */
  ::testing::AssertionResult ExpectedMessageCount(int num_load, int num_draw) {
    MessageResults results = ProcessMessages();
    if (results.num_draw != num_draw || results.num_load != num_load) {
      ::testing::AssertionFailure()
          << "Expected " << num_load << " load messages and " << num_draw
          << " draw messages"
          << "\nFound   " << results.num_load << " load messages and "
          << results.num_draw << " draw messages";
    }
    return ::testing::AssertionSuccess();
  }

  static constexpr double kPublishPeriod = 1 / 60.0;
  /* The LCM channel names. We are implicitly confirming that DrakeVisualizer
   broadcasts on the right channels via the subscribers. The reception of
   expected messages on those subscribers is proof.  */
  static constexpr char kLoadChannel[] = "DRAKE_VIEWER_LOAD_ROBOT";
  static constexpr char kDrawChannel[] = "DRAKE_VIEWER_DRAW";

  /* The name of the source registered with the SceneGraph.  */
  static constexpr char kSourceName[] = "DrakeVisualizerTest";

  /* The LCM visualizer broadcasts messages on.  */
  lcm::DrakeMockLcm lcm_;
  /* The subscribers for draw and load messages.  */
  lcm::Subscriber<lcmt_viewer_draw> draw_subscriber_{&lcm_, kDrawChannel};
  lcm::Subscriber<lcmt_viewer_load_robot> load_subscriber_{&lcm_, kLoadChannel};

  /* Raw pointer into the diagram's scene graph.  */
  SceneGraph<double>* scene_graph_{};
  /* Raw pointer into the diagram's visualizer.  */
  DrakeVisualizer* visualizer_{};
  /* Raw pointer to the pose data.  */
  PoseSource* pose_source_{};
  SourceId source_id_;
  /* A diagram containing scene graph and connected visualizer.  */
  unique_ptr<Diagram<double>> diagram_;
};

/* Confirms that the visualizer publishes at the specified period.  */
TEST_F(DrakeVisualizerTest, PublishPeriod) {
  for (double period : {1 / 30.0, 1 / 10.0}) {
    ConfigureDiagram(period);
    Simulator<double> simulator(*diagram_);
    ASSERT_EQ(simulator.get_context().get_time(), 0.0);

    SCOPED_TRACE(fmt::format("Period = {:.4}", period));

    // When we start up, we should get a load and a draw at time 0.
    simulator.AdvanceTo(0.0);
    EXPECT_TRUE(ExpectedMessageCount(1, 1));

    // Advancing past five period boundaries --> five draw messages.
    simulator.AdvanceTo(period * 5.1);
    EXPECT_TRUE(ExpectedMessageCount(0, 5));
  }
}

/* Confirms messages are sent, even if there is nothing. This matters because
 a no-op would *not* clear drake_visualizer leading to confusing results (if
 the visualizer already contained geometry from a previous session).  */
TEST_F(DrakeVisualizerTest, EmptyScene) {
  ConfigureDiagram();
  Simulator<double> simulator(*diagram_);
  simulator.AdvanceTo(0.0);
  MessageResults results = ProcessMessages();
  ASSERT_EQ(results.num_load, 1);
  EXPECT_EQ(results.load_message.num_links, 0);

  ASSERT_EQ(results.num_draw, 1);
  EXPECT_EQ(results.draw_message.num_links, 0);
  EXPECT_EQ(results.draw_message.timestamp, 0);
}

/* DrakeVisualizer can accept an lcm interface from the user or instantiate its
 own. This tests that logic. However, it does *not* do any work on the owned
 lcm interface because we don't want this unit test to spew network traffic.  */
TEST_F(DrakeVisualizerTest, OwnedLcm) {
  DrakeVisualizer visualizer_external(&lcm_);
  EXPECT_EQ(get_owned_interface(visualizer_external), nullptr);
  EXPECT_EQ(get_active_interface(visualizer_external), &lcm_);

  /* Note: do not do any work with this instance that would cause LCM messages
   to be spewed!  */
  DrakeVisualizer visualizer_owning;
  EXPECT_NE(get_owned_interface(visualizer_owning), nullptr);
  EXPECT_NE(get_owned_interface(visualizer_owning), &lcm_);
  EXPECT_EQ(get_active_interface(visualizer_owning),
            get_owned_interface(visualizer_owning));
}

/* DrakeVisualizer uses the cache to facilitate coordination between what got
 broadcast in the load message and what needs to be broadcast in the draw
 message. This confirms we get the same results with cache enabled and disabled.
 In this case, we use the number and names of the dynamic frames and the draw
 messages as evidence. */
TEST_F(DrakeVisualizerTest, CacheInsensitive) {
  lcmt_viewer_draw messages[2];

  for (bool cache_enabled : {true, false}) {
    ConfigureDiagram();
    PopulateScene();

    Simulator<double> simulator(*diagram_);
    if (cache_enabled) {
      simulator.get_mutable_context().EnableCaching();
    } else {
      simulator.get_mutable_context().DisableCaching();
    }
    simulator.AdvanceTo(0.0);
    MessageResults results = ProcessMessages();
    /* Confirm that messages were sent.  */
    ASSERT_EQ(results.num_load, 1);
    ASSERT_EQ(results.num_draw, 1);

    /* Stash the draw message for comparison.  */
    messages[cache_enabled ? 0 : 1] = results.draw_message;
  }
  /* Confirm that *something* was visualized.  */
  ASSERT_GT(messages[0].num_links, 0);

  /* Confirm the two draw messages are identical.  */
  const vector<uint8_t> encoding0 = lcm::EncodeLcmMessage(messages[0]);
  const vector<uint8_t> encoding1 = lcm::EncodeLcmMessage(messages[1]);
  EXPECT_EQ(encoding0, encoding1);
}

/* Confirm that configuring the default diffuse color affects the resulting
 color for geometries with no specified diffuse values.  */
TEST_F(DrakeVisualizerTest, ConfigureDefaultDiffuse) {
  /* Apply to arbitrary default diffuse and confirm the geometry (with no)
   diffuse color defined, inherits it.  */
  for (const auto& expected_rgba :
       {Rgba{0.75, 0.25, 0.125, 1.0}, Rgba{0.5, 0.75, 0.875, 0.5}}) {
    ConfigureDiagram(kPublishPeriod, Role::kProximity, expected_rgba);
    const FrameId f_id =
        scene_graph_->RegisterFrame(source_id_, GeometryFrame("frame", 0));
    const GeometryId g_id = scene_graph_->RegisterGeometry(
        source_id_, f_id,
        make_unique<GeometryInstance>(RigidTransformd{}, make_unique<Sphere>(1),
                                      "proximity_sphere"));
    // ProximityProperties will make use of the default diffuse value.
    scene_graph_->AssignRole(source_id_, g_id, ProximityProperties());
    pose_source_->SetPoses({{f_id, RigidTransformd{}}});
    Simulator<double> simulator(*diagram_);
    simulator.AdvanceTo(0.0);
    MessageResults results = ProcessMessages();
    ASSERT_EQ(results.num_load, 1);
    ASSERT_EQ(results.load_message.num_links, 1);
    ASSERT_EQ(results.load_message.link[0].num_geom, 1);
    const auto& color = results.load_message.link[0].geom[0].color;
    const Rgba rgba{color[0], color[1], color[2], color[3]};
    EXPECT_EQ(rgba, expected_rgba);
  }
}

/* Confirm that the anchored and dynamic geometries are handled correctly. All
 geometries are loaded (with the "world" frame holding the anchored geometries
 and all dynamic geometries on other frames). The draw message should only
 provide data for the dynamic frames (i.e., "world" will not be included).  */
TEST_F(DrakeVisualizerTest, AnchoredAndDynamicGeometry) {
  ConfigureDiagram(kPublishPeriod, Role::kProximity);
  const FrameId f_id =
      scene_graph_->RegisterFrame(source_id_, GeometryFrame("frame", 0));
  const GeometryId g0_id = scene_graph_->RegisterGeometry(
      source_id_, f_id,
      make_unique<GeometryInstance>(RigidTransformd{}, make_unique<Sphere>(1),
                                    "sphere0"));
  scene_graph_->AssignRole(source_id_, g0_id, ProximityProperties());
  const GeometryId g1_id = scene_graph_->RegisterGeometry(
      source_id_, f_id,
      make_unique<GeometryInstance>(RigidTransformd{Vector3d{10, 0, 0}},
                                    make_unique<Sphere>(1), "sphere1"));
  scene_graph_->AssignRole(source_id_, g1_id, ProximityProperties());

  const GeometryId g2_id = scene_graph_->RegisterAnchoredGeometry(
      source_id_,
      make_unique<GeometryInstance>(RigidTransformd{Vector3d{5, 0, 0}},
                                    make_unique<Sphere>(1), "sphere3"));
  scene_graph_->AssignRole(source_id_, g2_id, ProximityProperties());

  pose_source_->SetPoses({{f_id, RigidTransformd{}}});

  Simulator<double> simulator(*diagram_);
  simulator.AdvanceTo(0.0);
  MessageResults results = ProcessMessages();

  // Confirm load message; three geometries on two links.
  ASSERT_EQ(results.num_load, 1);
  ASSERT_EQ(results.load_message.num_links, 2);
  // We exploit the fact that we always know the world is first.
  ASSERT_EQ(results.load_message.link[0].name, "world");
  ASSERT_EQ(results.load_message.link[0].num_geom, 1);

  // Now test for the dynamic frame (with its two geometries).
  ASSERT_EQ(results.load_message.link[1].name, string(kSourceName) + "::frame");
  ASSERT_EQ(results.load_message.link[1].num_geom, 2);

  // Confirm draw message; a single link.
  ASSERT_EQ(results.num_draw, 1);
  ASSERT_EQ(results.draw_message.num_links, 1);
  ASSERT_EQ(results.draw_message.link_name[0], string(kSourceName) + "::frame");
}

/* Confirms that the role parameter leads to the correct geometry being
 selected.  */
TEST_F(DrakeVisualizerTest, TargetRole) {
  const string source_name(kSourceName);
  /* For a given role, the name of the *unique* frame we expect to load. The
   frame should have a single geometry affixed to it.  */
  const map<Role, string> expected{
      {Role::kIllustration, source_name + "::illustration"},
      {Role::kPerception, source_name + "::perception"},
      {Role::kProximity, source_name + "::proximity"}};
  for (const auto& [role, name] : expected) {
    ConfigureDiagram(kPublishPeriod, role);
    PopulateScene();
    Simulator<double> simulator(*diagram_);
    simulator.AdvanceTo(0.0);
    MessageResults results = ProcessMessages();

    /* Confirm that messages were sent.  */
    ASSERT_EQ(results.num_load, 1);
    ASSERT_EQ(results.load_message.num_links, 1);
    EXPECT_EQ(results.load_message.link[0].name, name);
    EXPECT_EQ(results.load_message.link[0].num_geom, 1);

    ASSERT_EQ(results.num_draw, 1);
    ASSERT_EQ(results.draw_message.num_links, 1);
    EXPECT_EQ(results.draw_message.link_name[0], name);
  }
}

/* When targeting a non-illustration role, if that same geometry *has* an
 illustration role with color, that value is used instead of the default.  */
TEST_F(DrakeVisualizerTest, GeometryWithIllustrationFallback) {
  ConfigureDiagram(kPublishPeriod, Role::kProximity);
  const GeometryId g_id = scene_graph_->RegisterAnchoredGeometry(
      source_id_, make_unique<GeometryInstance>(
                      RigidTransformd{}, make_unique<Sphere>(1), "sphere0"));
  const Rgba expected_rgba{0.25, 0.125, 0.75, 0.5};
  ASSERT_NE(expected_rgba, DrakeVisualizerParams().default_color);
  IllustrationProperties illus_props;
  illus_props.AddProperty("phong", "diffuse", expected_rgba);
  scene_graph_->AssignRole(source_id_, g_id, ProximityProperties());
  scene_graph_->AssignRole(source_id_, g_id, illus_props);

  Simulator<double> simulator(*diagram_);
  simulator.AdvanceTo(0.0);

  // We're just checking the load message for the right color.
  MessageResults results = ProcessMessages();
  ASSERT_EQ(results.num_load, 1);
  ASSERT_EQ(results.load_message.num_links, 1);
  ASSERT_EQ(results.load_message.link[0].num_geom, 1);
  const auto& color = results.load_message.link[0].geom[0].color;
  const Rgba rgba(color[0], color[1], color[2], color[3]);
  EXPECT_EQ(rgba, expected_rgba);
}

/* For *any* role type, if the properties include ("phong", "diffuse"), *that*
 color will be used.  */
TEST_F(DrakeVisualizerTest, AllRolesCanDefineDiffuse) {
  for (const Role role : {Role::kProximity, Role::kPerception}) {
    ConfigureDiagram(kPublishPeriod, role);
    const GeometryId g_id = scene_graph_->RegisterAnchoredGeometry(
        source_id_, make_unique<GeometryInstance>(
                        RigidTransformd{}, make_unique<Sphere>(1), "sphere0"));
    const Rgba expected_rgba{0.25, 0.125, 0.75, 0.5};
    ASSERT_NE(expected_rgba, DrakeVisualizerParams().default_color);
    if (role == Role::kProximity) {
      ProximityProperties props;
      props.AddProperty("phong", "diffuse", expected_rgba);
      scene_graph_->AssignRole(source_id_, g_id, props);
    } else if (role == Role::kPerception) {
      PerceptionProperties props;
      props.AddProperty("phong", "diffuse", expected_rgba);
      scene_graph_->AssignRole(source_id_, g_id, props);
    }

    Simulator<double> simulator(*diagram_);
    simulator.AdvanceTo(0.0);

    // We're just checking the load message for the right color.
    MessageResults results = ProcessMessages();
    ASSERT_EQ(results.num_load, 1);
    ASSERT_EQ(results.load_message.num_links, 1);
    ASSERT_EQ(results.load_message.link[0].num_geom, 1);
    const auto& color = results.load_message.link[0].geom[0].color;
    const Rgba rgba(color[0], color[1], color[2], color[3]);
    EXPECT_EQ(rgba, expected_rgba);
  }
}

/* Confirms that the documented prerequisites do bad things.  */
TEST_F(DrakeVisualizerTest, BadParameters) {
  // Zero publish period.
  EXPECT_THROW(
      DrakeVisualizer(&lcm_, DrakeVisualizerParams{0, Role::kIllustration,
                                                   Rgba{1, 1, 1, 1}}),
      std::exception);

  // Negative publish period.
  EXPECT_THROW(
      DrakeVisualizer(&lcm_, DrakeVisualizerParams{-0.1, Role::kIllustration,
                                                   Rgba{1, 1, 1, 1}}),
      std::exception);

  // Unassigned role.
  EXPECT_THROW(
      DrakeVisualizer(&lcm_, DrakeVisualizerParams{0.1, Role::kUnassigned,
                                                   Rgba{1, 1, 1, 1}}),
      std::exception);
}

/* This confirms that DrakeVisualizer will dispatch a new load message when it
 recognizes a change in the visualized role's version. We'll confirm both the
 positive case (change to expected role triggers load) and the negative case
 (change to other roles does *not* trigger load).  */
TEST_F(DrakeVisualizerTest, ChangesInVersion) {
  for (const Role role :
       {Role::kProximity, Role::kPerception, Role::kIllustration}) {
    ConfigureDiagram(kPublishPeriod, role);
    const GeometryId g_id = scene_graph_->RegisterAnchoredGeometry(
        source_id_, make_unique<GeometryInstance>(
                        RigidTransformd{}, make_unique<Sphere>(1), "sphere0"));

    Simulator<double> simulator(*diagram_);
    Context<double>& diagram_context = simulator.get_mutable_context();
    Context<double>& sg_context =
        diagram_->GetMutableSubsystemContext(*scene_graph_, &diagram_context);
    double t = 0.0;
    simulator.AdvanceTo(t);
    // Confirm the initial load/draw message pair.
    MessageResults results = ProcessMessages();
    ASSERT_EQ(results.num_load, 1);
    ASSERT_EQ(results.num_draw, 1);

    t += kPublishPeriod;
    simulator.AdvanceTo(t);
    // Confirm draw only.
    results = ProcessMessages();
    ASSERT_EQ(results.num_load, 0);
    ASSERT_EQ(results.num_draw, 1);

    // Confirm that modifying a role has the expected outcome. If the modified
    // role matches the visualized `role`, we expect a load message. Otherwise
    // expect no load message.
    // We don't exhaustively compare all three roles. The only way to get
    // perception version changes is to have an actual render engine that
    // accepts geometry. We'll avoid the overhead and assume that successful
    // interactions of other role-role matchups suggest that it's likewise true
    // for a modified perception role.
    for (const Role modified_role : {Role::kProximity, Role::kIllustration}) {
      if (modified_role == Role::kProximity) {
        scene_graph_->AssignRole(&sg_context, source_id_, g_id,
                                 ProximityProperties());
      } else if (modified_role == Role::kIllustration) {
        scene_graph_->AssignRole(&sg_context, source_id_, g_id,
                                 IllustrationProperties());
      }
      t += kPublishPeriod;
      simulator.AdvanceTo(t);
      results = ProcessMessages();
      EXPECT_EQ(results.num_load, modified_role == role ? 1 : 0)
          << "For visualized role '" << role << "' and modified role '" << role
          << "'\n";
      EXPECT_EQ(results.num_draw, 1);
    }
  }
}

}  // namespace geometry
}  // namespace drake
