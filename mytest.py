import math
import random

import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm_notebook

import phyre
random.seed(0)

# from src.python.phyre.data.task_scripts.main.task00000 import build_task

print('All eval setups:', *phyre.MAIN_EVAL_SETUPS)
# For now, let's select cross template for ball tier.
eval_setup = 'ball_cross_template'

fold_id = 0  # For simplicity, we will just use one fold for evaluation.
train_tasks, dev_tasks, test_tasks = phyre.get_fold(eval_setup, fold_id)
print('Size of resulting splits:\n train:', len(train_tasks), '\n dev:',
      len(dev_tasks), '\n test:', len(test_tasks))

print(*dev_tasks[:10], sep=', ')

action_tier = phyre.eval_setup_to_action_tier(eval_setup)
print('Action tier for', eval_setup, 'is', action_tier)

tasks = dev_tasks[:1]

# Create the simulator from the tasks and tier.
simulator = phyre.initialize_simulator(tasks, action_tier)

task_index = 0  # Note, this is a integer index of task within simulator.task_ids.
task_id = simulator.task_ids[task_index]
initial_scene = simulator.initial_scenes[task_index]
print('Initial scene shape=%s dtype=%s' % (initial_scene.shape, initial_scene.dtype))
# plt.imshow(phyre.observations_to_float_rgb(initial_scene))
# plt.title(f'Task {task_id}');

# Let's see the features of the initial featurized objects for the scene visualized above
initial_featurized_objects = simulator.initial_featurized_objects[task_index]
print('Initial featurized objects shape=%s dtype=%s' % (initial_featurized_objects.features.shape, initial_featurized_objects.features.dtype))
np.set_printoptions(precision=3)
print(initial_featurized_objects.features)

print('Dimension of the action space:', simulator.action_space_dim)

actions = simulator.build_discrete_action_space(max_actions=100)
print('A random action:', actions[0])

task_index = 0  # The simulator takes an index into simulator.task_ids.
action = random.choice(actions)
# Set need_images=False and need_featurized_objects=False to speed up simulation, when only statuses are needed.
simulation = simulator.simulate_action(task_index, action, need_images=True, need_featurized_objects=True)

### task_index = 0: 4 bounding boxes, 1 agent, 6 env objects (??)

# Visualize
if False:
    img_start = phyre.vis.observations_to_float_rgb(simulation.images[0])
    img_end = phyre.vis.observations_to_float_rgb(simulation.images[-1])

    fig, axs = plt.subplots(1, 2, figsize=(7, 7))
    fig.tight_layout()
    plt.subplots_adjust(hspace=0.4, wspace=0.3)
    axs[0].imshow(img_start)
    axs[0].title.set_text(f'Start state\nAction solves task: {simulation.status.is_solved()}')
    axs[0].get_xaxis().set_ticks([])
    axs[0].get_yaxis().set_ticks([])

    axs[1].imshow(img_end)
    axs[1].title.set_text(f'End state\nAction solves task: {simulation.status.is_solved()}')
    axs[1].get_xaxis().set_ticks([])
    axs[1].get_yaxis().set_ticks([]);

    plt.show()

    exit()

# Three statuses could be returned.
print('Action solves task:', phyre.SimulationStatus.SOLVED)
print('Action does not solve task:', phyre.SimulationStatus.NOT_SOLVED)
print('Action is an invalid input on task (e.g., occludes a task object):',
      phyre.SimulationStatus.INVALID_INPUT)
# May call is_* methods on the status to check the status.
print()
print('Result of taking action', action, 'on task', tasks[task_index], 'is:',
      simulation.status)
print('Does', action, 'solve task', tasks[task_index], '?', simulation.status.is_solved())
print('Is', action, 'an invalid action on task', tasks[task_index], '?',
      simulation.status.is_invalid())