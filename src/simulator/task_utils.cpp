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

struct RelationshipData {
  // timestep: 1/60 second for default (check task_utils.h)
  // timestep -> object_i -> object_j -> vector of relationship numbers (touch or not)
  std::vector<std::map<int, std::map<int, std::vector<bool>>>> timestep_relationships;
  // timestep -> map(object_i -> {x, y, angle} in pixels)
  std::vector<std::map<int, std::vector<float>>> timestep_positions_angles;
  int num_general_objects;
  int num_user_input_objects;
};

std::vector<::task::SpatialRelationship::type> touching_vector = {::task::SpatialRelationship::type::TOUCHING};

constexpr float kBallTouchingThreshold = 0.1 / PIXELS_IN_METER;

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

void calculateAllRelationships(const ::task::Task& task, const b2WorldWithData& world, RelationshipData& result) {
  // std::vector<int> object_output_ids;
  int object_output_id1;
  int object_output_id2;
  const auto& bodies = task.scene.bodies;
  const auto& user_input_bodies = task.scene.user_input_bodies;
  result.num_general_objects = bodies.size();
  result.num_user_input_objects = user_input_bodies.size();

  // std::vector<std::vector<std::vector<int>>> current_timestep;
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
    result.timestep_positions_angles.push_back(std::map<int, std::vector<float>>{{object_output_id1, {x, y, angle}}});

    /// Get second body
    const b2Body* box2dBody2 = world.GetBodyList();
    for (; box2dBody2 != nullptr; box2dBody2 = box2dBody2->GetNext()) {
      if (box2dBody2->GetUserData() == nullptr) {
        throw std::runtime_error("Found a Box2d body without userdata");
      }
      const Box2dData* box2d_data2 = static_cast<Box2dData*>(box2dBody2->GetUserData());
      if (box2d_data2 == nullptr) {
        throw std::runtime_error("Found a Box2d body with userdata that is not Box2dData");
      } 
      // Get output_id2 and body2
      if (box2d_data2->object_type == Box2dData::GENERAL) {
        object_output_id2 = box2d_data2->object_id;
        body2 = &bodies.at(box2d_data2->object_id);
      } else {
        object_output_id2 = result.num_general_objects + box2d_data2->object_id;
        body2 = &user_input_bodies.at(box2d_data2->object_id);
      }

      // Check if two balls are touching
      if (isTwoBallTouchingCase(*body1, *body2, touching_vector)) {
        const auto r1 = body1->shapes[0].circle.radius;
        const auto r2 = body2->shapes[0].circle.radius;
        const float distance = sqrt(
            geometry::squareDistance(body1->position, body2->position));
        if (distance < r1 + r2 + kBallTouchingThreshold) {
          std::map<int, std::map<int, std::vector<bool>>> timestep_map;
          timestep_map[object_output_id1][object_output_id2] = {true};
          result.timestep_relationships.push_back(timestep_map);
        }
      }
      else {
        // Check other relationships
      }
    }


  }






  
  // A custom check for a pair of touching balls to improve stability.
  

  ::scene::Shape scaledPhantomShape;
  if (task.__isset.phantomShape && task.phantomShape.__isset.polygon) {
    scaledPhantomShape = p2mShape(task.phantomShape);
  }
  
  // For each pair of objects
  for (size_t i = 0; i < bodies.size(); i++) {
    std::vector<std::vector<int>> object_i_relationships;
    for (size_t j = 0; j < bodies.size(); j++) {
      if (i == j) {
        object_i_relationships.push_back(std::vector<int>());
        continue;
      }
      
      const auto& body1 = bodies[i];
      const auto& body2 = bodies[j];
      std::vector<int> relationships;

      // 특별한 경우 처리 - 두 공이 닿는 경우
      bool isTwoBallCase = false;
      if (body1.shapes.size() == 1 && body2.shapes.size() == 1 &&
        body1.shapes[0].__isset.circle && body2.shapes[0].__isset.circle) {
        isTwoBallCase = true;
        
        // Box2D world에서 현재 위치와 속성을 가져옴
        const b2Body* b2body1 = world.bodies[i];
        const b2Body* b2body2 = world.bodies[j];
        
        b2Vec2 pos1 = b2body1->GetPosition();
        b2Vec2 pos2 = b2body2->GetPosition();
        
        float r1 = body1.shapes[0].circle.radius;
        float r2 = body2.shapes[0].circle.radius;
        float distance = (pos2 - pos1).Length();
        
        // TOUCHING 관계 확인
        if (distance < r1 + r2 + kBallTouchingThreshold) {
            relationships.push_back(TOUCHING);
        }
        // NOT_TOUCHING 관계 확인
        else {
            relationships.push_back(NOT_TOUCHING);
        }
      }
      
      // 일반적인 경우 처리
      if (!isTwoBallCase) {


        for (int rel = 1; rel <= 9; ++rel) {
          auto relationship = static_cast<::task::SpatialRelationship::type>(rel);
          
          // INSIDE/NOT_INSIDE 관계는 phantomShape가 필요
          if ((relationship == ::task::SpatialRelationship::INSIDE || 
                relationship == ::task::SpatialRelationship::NOT_INSIDE) && 
            !task.__isset.phantomShape) {
            continue;
          }
          
          // Box2D world의 현재 상태를 사용하여 관계 확인
          if (isValidRelationship(world.bodies[i], world.bodies[j], relationship, task.phantomShape)) {
            relationships.push_back(rel);
          }
        }
      }
      
      object_i_relationships.push_back(relationships);
    }
    current_timestep.push_back(object_i_relationships);
  }
  result.timestep_relationships.push_back(current_timestep);
  
  return result;
}

// Runs simulation for the scene. If task is not nullptr, is-task-solved checks
// are performed.
::task::TaskSimulation simulateTask(const ::scene::Scene &scene,
                                    const SimulationRequest &request,
                                    const ::task::Task *task) {
  std::unique_ptr<b2WorldWithData> world = convertSceneToBox2dWorld(scene);

  unsigned int continuousSolvedCount = 0;
  std::vector<::scene::Scene> scenes;
  std::vector<bool> solveStateList;
  bool solved = false;
  int step = 0;
  
  RelationshipData relationshipData;
  
  if (false) { // debug
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
    world->Step(kTimeStep, kVelocityIterations, kPositionIterations);
    if (request.stride > 0 && step % request.stride == 0) {
      scenes.push_back(updateSceneFromWorld(scene, *world));
      calculateAllRelationships(*task, *world, relationshipData);
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
            break;
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

  return taskSimulation;
}
}  // namespace

std::vector<::scene::Scene> simulateScene(const ::scene::Scene &scene,
                                          const int num_steps) {
  const SimulationRequest request{num_steps, 1};
  const auto simulation = simulateTask(scene, request, /*task=*/nullptr);
  return simulation.sceneList;
}

::task::TaskSimulation simulateTask(const ::task::Task &task,
                                    const int num_steps, const int stride) {
  const SimulationRequest request{num_steps, stride};
  return simulateTask(task.scene, request, &task);
}
