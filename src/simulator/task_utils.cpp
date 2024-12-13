// Copyright (c) Facebook, Inc. and its affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "task_utils.h"
#include "task_validation.h"
#include "thrift_box2d_conversion.h"

#include "gen-cpp/scene_types.h"
#include "gen-cpp/shared_constants.h"
#include "geometry.h"

#include <iostream>

using scene::Body;

namespace {
struct SimulationRequest {
  int maxSteps;
  int stride;
};

float m2p(float meters) { return meters * PIXELS_IN_METER; }

// Relationship enum values from ttypes
enum RelationshipType {
  NONE = 0,
  ABOVE = 1,
  BELOW = 2,
  LEFT_OF = 3,
  RIGHT_OF = 4,
  TOUCHING_BRIEFLY = 5,
  TOUCHING = 6,
  INSIDE = 7,
  NOT_TOUCHING = 8,
  NOT_INSIDE = 9
};

const std::vector<::task::SpatialRelationship::type> touching_vector = {::task::SpatialRelationship::type::TOUCHING};
// const std::vector<::task::SpatialRelationship::type> relation_vector = {::task::SpatialRelationship::type::TOUCHING};

const Box2dData* getBodyUserData(const b2Body& body) {
  if (body.GetUserData() == nullptr) {
    throw std::runtime_error("Found a Box2d body without userdata");
  }
  const Box2dData* box2d_data = static_cast<Box2dData*>(body.GetUserData());
  if (box2d_data == nullptr) {
    throw std::runtime_error(
        "Found a Box2d body with userdata that is not Box2dData");
  }
  return box2d_data;
}

size_t getBodyId(const b2Body& body) {
  const Box2dData* box2d_data = getBodyUserData(body);
  return box2d_data->object_id;
}

Box2dData::ObjectType getBodyType(const b2Body& body) {
  const Box2dData* box2d_data = getBodyUserData(body);
  return box2d_data->object_type;
}

bool isTwoBallTouchingCase(
    const ::scene::Body thriftBody1, const ::scene::Body& thriftBody2,
    const std::vector<::task::SpatialRelationship::type>& relationships) {
  if (thriftBody1.shapes.size() != 1) return false;
  if (thriftBody2.shapes.size() != 1) return false;
  if (relationships.size() != 1) return false;
  if (!thriftBody1.shapes[0].__isset.circle) return false;
  if (!thriftBody2.shapes[0].__isset.circle) return false;
  if (relationships[0] != ::task::SpatialRelationship::TOUCHING) return false;
  return true;
}

// Need to change into b2ContactListener (https://box2d.org/doc_version_2_4/classb2_contact_listener.html)
// GetContactList is not accurate.
bool isTouching(const b2Body& body1, const b2Body& body2) {
  size_t body2Id = getBodyId(body2);
  for (const b2ContactEdge* ce = body1.GetContactList(); ce; ce = ce->next) {
    if (body2Id == getBodyId(*ce->other) &&
        ce->contact->IsTouching()) {
      return true;
    }
  }
  return false;
}

void calculateAllRelationships(const ::task::Task& task, const b2WorldWithData& world, RelationshipData& result) {
  // std::vector<int> object_output_ids;
  int object_output_id1;
  int object_output_id2;
  const auto& bodies = task.scene.bodies;
  const auto& user_input_bodies = task.scene.user_input_bodies;
  result.num_general_objects = bodies.size();
  result.num_user_input_objects = user_input_bodies.size();

  std::map<int, std::map<int, bool>> timestep_relationships_t;
  std::map<int, std::vector<float>> timestep_positions_angles_t;

  const ::scene::Body* body1;
  const ::scene::Body* body2;

  const b2Body* box2dBody1 = world.GetBodyList();
  for (; box2dBody1 != nullptr; box2dBody1 = box2dBody1->GetNext()) {
    if (box2dBody1->GetUserData() == nullptr) {
      throw std::runtime_error("Found a Box2d body without userdata");
    }
    const Box2dData* box2d_data1 = static_cast<Box2dData*>(box2dBody1->GetUserData());
    if (box2d_data1 == nullptr) {
      throw std::runtime_error("Found a Box2d body with userdata that is not Box2dData");
    }

    // Get output_id1 and body1 (general idx => 0, 1, 2, ...; user_input idx => num_general_objects + 0, num_general_objects + 1, ...;)
    if (box2d_data1->object_type == Box2dData::GENERAL) {
      object_output_id1 = box2d_data1->object_id;
      body1 = &bodies.at(box2d_data1->object_id);
    } else {
      object_output_id1 = result.num_general_objects + box2d_data1->object_id;
      body1 = &user_input_bodies.at(box2d_data1->object_id);
    }

    // Push back positions and angles
    float x = m2p(box2dBody1->GetPosition().x);
    float y = m2p(box2dBody1->GetPosition().y);
    float angle = box2dBody1->GetAngle();
    timestep_positions_angles_t[object_output_id1] = {x, y, angle};

    // Get second body
    const b2Body* box2dBody2 = world.GetBodyList();
    for (; box2dBody2 != nullptr; box2dBody2 = box2dBody2->GetNext()) {
      if (box2dBody2->GetUserData() == nullptr) {
        throw std::runtime_error("Found a Box2d body without userdata");
      }
      const Box2dData* box2d_data2 = static_cast<Box2dData*>(box2dBody2->GetUserData());
      if (box2d_data2 == nullptr) {
        throw std::runtime_error("Found a Box2d body with userdatbox2dBody2a that is not Box2dData");
      } 
      // Get output_id2 and body2
      if (box2d_data2->object_type == Box2dData::GENERAL) {
        object_output_id2 = box2d_data2->object_id;
        body2 = &bodies.at(box2d_data2->object_id);
      } else {
        object_output_id2 = result.num_general_objects + box2d_data2->object_id;
        body2 = &user_input_bodies.at(box2d_data2->object_id);
      }

      // Check relationships
      if (object_output_id1 == object_output_id2){
        timestep_relationships_t[object_output_id1][object_output_id2] = false; // No self-relationships
      }
      else if (isTwoBallTouchingCase(*body1, *body2, touching_vector)) {
        // Check if two balls are touching
        const auto r1 = box2dBody1->GetFixtureList()->GetShape()->m_radius;
        const auto r2 = box2dBody2->GetFixtureList()->GetShape()->m_radius;
        const float distance = sqrt(geometry::squareDistance(box2dBody1->GetPosition(), box2dBody2->GetPosition()));
        if (distance < r1 + r2 + kBallTouchingThreshold) {
          timestep_relationships_t[object_output_id1][object_output_id2] = true;
        }
        else {
          timestep_relationships_t[object_output_id1][object_output_id2] = false;
        }
      }
      else {
        // Check other relationships
        if (isTouching(*box2dBody1, *box2dBody2)) {
          timestep_relationships_t[object_output_id1][object_output_id2] = true;
        }
        else {
          timestep_relationships_t[object_output_id1][object_output_id2] = false;
        }
      }
    }
  }

  // Push back timestep relationships and positions/angles
  result.timestep_relationships.push_back(timestep_relationships_t);
  result.timestep_positions_angles.push_back(timestep_positions_angles_t);
}

// Runs simulation for the scene. If task is not nullptr, is-task-solved checks
// are performed.
std::tuple<::task::TaskSimulation, RelationshipData> simulateTaskInternal(const ::scene::Scene &scene,
                                                                          const SimulationRequest &request,
                                                                          const ::task::Task *task,
                                                                          const bool is_calculate_relationships) {
  std::unique_ptr<b2WorldWithData> world = convertSceneToBox2dWorld(scene);

  unsigned int continuousSolvedCount = 0;
  std::vector<::scene::Scene> scenes;
  std::vector<bool> solveStateList;
  bool solved = false;
  int step = 0;
  
  RelationshipData relationshipData;
  
  // Debug
  if (false) { 
    int num_objects = scene.bodies.size();
    int num_user_input_objects = scene.user_input_bodies.size();

    printf("num_objects: %d\n", num_objects); // env objects except agents 
    printf("num_user_input_objects: %d\n", num_user_input_objects); // agents (red balls)

    for (const auto& body : scene.bodies) {
      printf("body.color: %d\n", body.color); // WHITE = 0, BLACK = 6, GRAY = 5, GREEN = 2, BLUE = 3, PURPLE = 4, RED = 1, LIGHT_RED = 7
      printf("body.shapeType: %d\n", body.shapeType); // UNDEFINED = 0, BALL = 1, BAR = 2, JAR = 3, STANDINGSTICKS = 4
    }
    for (const auto& body : scene.user_input_bodies) {
      printf("body.color: %d\n", body.color); // WHITE = 0, BLACK = 6, GRAY = 5, GREEN = 2, BLUE = 3, PURPLE = 4, RED = 1, LIGHT_RED = 7
      printf("body.shapeType: %d\n", body.shapeType); // UNDEFINED = 0, BALL = 1, BAR = 2, JAR = 3, STANDINGSTICKS = 4
    }

    printf("bodies:\n");
    for (const b2Body* box2dBody = world->GetBodyList(); box2dBody != nullptr;
        box2dBody = box2dBody->GetNext()) {
      size_t body2Id = getBodyId(*box2dBody); // subindex per body type
      Box2dData::ObjectType body2Type = getBodyType(*box2dBody); // 0: general (= env objects), 1: user (= agents), 2: bounding box
      printf("body2Id: %zu, body2Type: %d\n", body2Id, body2Type);
      }
    exit(0);
  }

  // For different relations number of steps the condition should hold varies.
  // For NOT_TOUCHING relation one of three should be true:
  //   1. Objects are touching at the beginning and then not touching for
  //   kStepsForSolution steps.
  //   2. Objects are not touching at the beginning, touching at some point of
  //   simulation and then not touching for kStepsForSolution steps.
  //   3. Objects are not touching whole sumulation.
  // For TOUCHING_BRIEFLY a single touching is allowed.
  // For all other relations the condition must hold for kStepsForSolution
  // consequent steps.
  bool lookingForSolution =
      (task == nullptr || !isTaskInSolvedState(*task, *world) ||
       task->relationships.size() != 1 ||
       task->relationships[0] != ::task::SpatialRelationship::NOT_TOUCHING);
  const bool allowInstantSolution =
      (task != nullptr && task->relationships.size() == 1 &&
       task->relationships[0] == ::task::SpatialRelationship::TOUCHING_BRIEFLY);
  for (; step < request.maxSteps; step++) {
    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.

    // printf("step/maxSteps: %d/%d\n", step, request.maxSteps);
    // printf("stride: %d\n", request.stride);
    // printf("step %% stride: %d\n", step % request.stride);

    world->Step(kTimeStep, kVelocityIterations, kPositionIterations);
    if (request.stride > 0 && step % request.stride == 0) { // default stride = FPS = 60
      scenes.push_back(updateSceneFromWorld(scene, *world));
      if (is_calculate_relationships) {
        calculateAllRelationships(*task, *world, relationshipData);
      }
    }
    if (task == nullptr) {
      solveStateList.push_back(false);
    } else {
      solveStateList.push_back(isTaskInSolvedState(*task, *world));
      if (solveStateList.back()) {
        continuousSolvedCount++;
        if (lookingForSolution) {
          if (continuousSolvedCount >= kStepsForSolution ||
              allowInstantSolution) {
            solved = true;
            // break; // keep running after solution is found
          }
        }
      } else {
        lookingForSolution = true;  // Task passed through non-solved state.
        continuousSolvedCount = 0;
      }
    }
  }

  if (!lookingForSolution && continuousSolvedCount == solveStateList.size()) {
    // See condition 3) for NOT_TOUCHING relation above.
    solved = true;
  }

  {
    std::vector<bool> stridedSolveStateList;
    if (request.stride > 0) {
      for (size_t i = 0; i < solveStateList.size(); i += request.stride) {
        stridedSolveStateList.push_back(solveStateList[i]);
      }
    }
    stridedSolveStateList.swap(solveStateList);
  }

  ::task::TaskSimulation taskSimulation;
  taskSimulation.__set_sceneList(scenes);
  taskSimulation.__set_stepsSimulated(step);
  if (task != nullptr) {
    taskSimulation.__set_solvedStateList(solveStateList);
    taskSimulation.__set_isSolution(solved);
  }

  return std::make_tuple(taskSimulation, relationshipData);
}
}  // namespace

std::vector<::scene::Scene> simulateScene(const ::scene::Scene &scene,
                                          const int num_steps) {
  const SimulationRequest request{num_steps, 1};
  const auto simulation = simulateTaskInternal(scene, request, /*task=*/nullptr, /*is_calculate_relationships=*/false);
  return std::get<0>(simulation).sceneList; 
}

::task::TaskSimulation simulateTask(const ::task::Task &task,
                                                      const int num_steps, const int stride) {
  const SimulationRequest request{num_steps, stride};
  return std::get<0>(simulateTaskInternal(task.scene, request, &task, /*is_calculate_relationships=*/false));
}

std::tuple<::task::TaskSimulation, RelationshipData> simulateTaskRelationships(const ::task::Task &task,
                                                      const int num_steps, const int stride) {
  const SimulationRequest request{num_steps, stride};
  return simulateTaskInternal(task.scene, request, &task, /*is_calculate_relationships=*/true);
}
