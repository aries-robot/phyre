import math
import random

import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import gridspec
import numpy as np
import networkx as nx

import phyre
random.seed(0)

# print('All eval setups:', *phyre.MAIN_EVAL_SETUPS)
# For now, let's select cross template for ball tier.
eval_setup = 'ball_within_template'

fold_id = 0 # For simplicity, we will just use one fold for evaluation. # this is seed for the split
train_tasks, dev_tasks, test_tasks = phyre.get_fold(eval_setup, fold_id) # train_tasks, valid_tasks, test_tasks 
# print('Size of resulting splits:\n train:', len(train_tasks), '\n dev:',
#       len(dev_tasks), '\n test:', len(test_tasks))

all_tasks = sorted(list(train_tasks + dev_tasks + test_tasks))
id = '00000'
new_tasks = []
for task in all_tasks:
    if id+':' in task:
        new_tasks.append(task)

action_tier = phyre.eval_setup_to_action_tier(eval_setup)
print('Action tier for', eval_setup, 'is', action_tier) # ball

# Create the simulator from the tasks and tier.
simulator = phyre.initialize_simulator(new_tasks, action_tier)

# task_index = 0  # task 00000:000
# task_id = simulator.task_ids[task_index]
# initial_scene = simulator.initial_scenes[task_index]
# print('Initial scene shape=%s dtype=%s' % (initial_scene.shape, initial_scene.dtype))

# Let's see the features of the initial featurized objects for the scene visualized above
# initial_featurized_objects = simulator.initial_featurized_objects[task_index]
# print('Initial featurized objects shape=%s dtype=%s' % (initial_featurized_objects.features.shape, initial_featurized_objects.features.dtype))
# np.set_printoptions(precision=3)
# print(initial_featurized_objects.features)

# print('Dimension of the action space:', simulator.action_space_dim)
actions = simulator.build_discrete_action_space(max_actions=100)
print('A random action:', actions[0])

task_index = 0  # The simulator takes an index into simulator.task_ids. (task 00000:000)
action = random.choice(actions)
simulation = simulator.simulate_action(task_index, action, need_images=True, need_featurized_objects=True, stride=60) # default stride is 60

def reshape_images_ids(images_ids, timestep):
    new_images_ids = images_ids[timestep][::-1]
    return new_images_ids

if False:
    timestep = -1
    plt.figure(figsize=(10, 10))
    plt.imshow(reshape_images_ids(simulation.images_ids, timestep), cmap='tab10', interpolation='nearest')
    plt.colorbar(label="Index Value")
    plt.title('Object IDs in Simulation\n(-1: background, 0-3: walls, 4+: objects)')
    plt.axis('on')
    plt.grid(True)
    plt.show()
    exit()

total_timesteps = len(simulation.relationships['timestep_relationships'])
num_objects = simulation.relationships['num_general_objects'] + simulation.relationships['num_user_input_objects']

relationships_array = np.zeros((total_timesteps, num_objects, num_objects), dtype=np.bool)
for t, relationship in enumerate(simulation.relationships['timestep_relationships']):
    for i in relationship.keys():
        for j in relationship[i].keys():
            relationships_array[t, i, j] = relationship[i][j]

relationships_changed = [relationships_array[0]]
relationships_changed_timesteps_idx = [0]
for t in range(1, total_timesteps):
    if not np.all(relationships_changed[-1] == relationships_array[t]):
        relationships_changed.append(relationships_array[t])
        relationships_changed_timesteps_idx.append(t)

### Filter the relationships changed timesteps
if False:
    relationships_changed_timesteps_filtered_idx = []
    relationships_changed_filtered = []

    idx = 0
    while idx < len(relationships_changed_timesteps_idx) - 1:
        current_timestep = relationships_changed_timesteps_idx[idx]
        next_timestep = relationships_changed_timesteps_idx[idx + 1]

        ### Check if this is the start of a consecutive sequence
        if next_timestep - current_timestep == 1:

            ### Find the end of the consecutive sequence
            end_idx = idx + 1
            while end_idx < len(relationships_changed_timesteps_idx) - 1:
                current_timestep_end_idx = relationships_changed_timesteps_idx[end_idx]
                next_timestep_end_idx = relationships_changed_timesteps_idx[end_idx + 1]
                if next_timestep_end_idx - current_timestep_end_idx != 1:
                    break
                end_idx += 1

            ### Take only the first timestep of the sequence
            relationships_changed_timesteps_filtered_idx.append(relationships_changed_timesteps_idx[idx])

            ### Perform logical OR on the sequence of relationships
            combined_relationship = relationships_changed[idx]
            # seq_idx_list = [idx] # debug
            # overlap_list = [relationships_changed_timesteps_idx[idx]] # debug
            for seq_idx in range(idx + 1, end_idx + 1):
                # seq_idx_list.append(seq_idx) # debug
                # overlap_list.append(relationships_changed_timesteps_idx[seq_idx]) # debug
                combined_relationship = np.logical_or(combined_relationship, relationships_changed[seq_idx])
            relationships_changed_filtered.append(combined_relationship)
            # print('seq_idx_list:', seq_idx_list)
            # print('overlap_list:', overlap_list)

            ### Skip to the end of the sequence
            idx = end_idx + 1

        else:
            ### If not consecutive, add the current timestep
            relationships_changed_timesteps_filtered_idx.append(relationships_changed_timesteps_idx[idx])
            relationships_changed_filtered.append(relationships_changed[idx])
            idx += 1

    ### Add the last timestep if we haven't processed it
    if idx == len(relationships_changed_timesteps_idx) - 1:
        relationships_changed_timesteps_filtered_idx.append(relationships_changed_timesteps_idx[idx])
        relationships_changed_filtered.append(relationships_changed[idx])

### Animate the relationship graph
if False:
    ### Generate the images and graph animated for each timestep
    # Create figure with two subplots side by side
    fig = plt.figure(figsize=(20, 8))
    ax1 = plt.subplot(121)  # For simulation images
    ax2 = plt.subplot(122)  # For relationship graphs

    def update(frame):
        # Clear previous plots
        ax1.clear()
        ax2.clear()
        
        # Plot simulation image
        ax1.imshow(phyre.vis.observations_to_float_rgb(simulation.images[frame]))
        ax1.set_title(f'Simulation Frame {frame}')
        
        # Add walls/boundaries
        ax1.axhline(y=0, color='black', linewidth=2)  # Top wall
        ax1.axhline(y=255, color='black', linewidth=2)  # Bottom wall
        ax1.axvline(x=0, color='black', linewidth=2)  # Left wall
        ax1.axvline(x=255, color='black', linewidth=2)  # Right wall
        ax1.axis('on')  # Show the axes
        
        # Create and plot relationship graph
        G = nx.Graph()
        num_objects = relationships_array.shape[1]
        G.add_nodes_from(range(num_objects))
        
        # Add labels for nodes to distinguish walls
        labels = {}
        for i in range(num_objects):
            if i < 4:  # First 4 objects are typically walls in PHYRE
                labels[i] = f'Wall {i}'
            else:
                labels[i] = f'Obj {i}'
        
        for i in range(num_objects):
            for j in range(i+1, num_objects):
                if relationships_array[frame, i, j]:
                    G.add_edge(i, j)
        
        pos = nx.kamada_kawai_layout(G)
        nx.draw(G, pos,
                ax=ax2,
                labels=labels,
                node_color=['lightgray' if i < 4 else 'lightblue' for i in range(num_objects)],
                node_size=500,
                font_size=10,
                font_weight='bold',
                edge_color='gray',
                width=2)
        ax2.set_title(f'Object Relationships at Frame {frame}')

    # Create animation
    anim = animation.FuncAnimation(fig, update, frames=len(simulation.images), 
                                interval=5, repeat=False)

    plt.tight_layout()
    plt.show()

    exit()

### Plot the relationship graph for all the changed timesteps
elif True:
    timesteps_idx = relationships_changed_timesteps_idx
    relationships = relationships_changed

    print(f"Plotting graphs for {len(timesteps_idx)} timesteps where relationships changed")

    ### Determine the number of rows needed for the grid
    num_plots = len(timesteps_idx)
    cols = 6  # Six columns for each section
    rows = (num_plots + cols - 1) // cols

    ### Create figure with nested gridspec for layout
    fig = plt.figure(figsize=(30, 1 * (rows + 1)))  # Increased figure width

    ### Create outer grid with 3 columns now
    outer_grid = gridspec.GridSpec(1, 3, figure=fig)

    ### Create three inner grids, each with rows x 6 layout
    left_inner_grid = gridspec.GridSpecFromSubplotSpec(rows, 6, subplot_spec=outer_grid[0])
    center_inner_grid = gridspec.GridSpecFromSubplotSpec(rows, 6, subplot_spec=outer_grid[1])
    right_inner_grid = gridspec.GridSpecFromSubplotSpec(rows, 6, subplot_spec=outer_grid[2])

    ### Plot all visualizations
    for idx, (timestep, relationship_t) in enumerate(zip(timesteps_idx, relationships)):
        row = idx // 6
        col = idx % 6
        
        ### Plot simulation image (left)
        ax_img = fig.add_subplot(left_inner_grid[row, col], aspect='equal')
        ax_img.imshow(phyre.vis.observations_to_float_rgb(simulation.images[timestep]))
        ax_img.text(0.5, 0.9, f't={timestep}', 
                   horizontalalignment='center',
                   transform=ax_img.transAxes)
        ax_img.set_xticks([])
        ax_img.set_yticks([])
        ax_img.set_title('')

        ### Plot object IDs (center)
        ax_ids = fig.add_subplot(center_inner_grid[row, col])
        im = ax_ids.imshow(reshape_images_ids(simulation.images_ids, timestep), # wall: bottom0, left1, top2, right3
                           cmap='tab10', 
                           interpolation='nearest')
        ax_ids.text(0.5, 0.9, f't={timestep}', 
                   horizontalalignment='center',
                   transform=ax_ids.transAxes)
        ax_ids.set_xticks([])
        ax_ids.set_yticks([])
        ax_ids.set_title('')
        
        ### Plot relationship graph (right)
        ax_graph = fig.add_subplot(right_inner_grid[row, col], aspect='equal')  # Force equal aspect ratio
        G = nx.Graph()
        num_objects = relationship_t.shape[1]
        G.add_nodes_from(range(num_objects))
        
        labels = {i: f'{i}' for i in range(num_objects)}
        
        for i in range(num_objects):
            for j in range(i+1, num_objects):
                if relationship_t[i, j]:
                    G.add_edge(i, j)
        
        pos = nx.kamada_kawai_layout(G)

        nx.draw(G, pos,
                ax=ax_graph,
                labels=labels,
                node_color=['lightgray' if i < 4 else 'lightblue' for i in range(num_objects)],
                node_size=100,
                font_size=10,
                font_weight='bold',
                edge_color='gray',
                width=2)
        
        # Set equal limits and force square shape
        ax_graph.set_xlim(-1.2, 1.2)
        ax_graph.set_ylim(-1.2, 1.2)
        ax_graph.set_aspect('equal', adjustable='box')
        
        ax_graph.set_title('')
        ax_graph.text(0.15, 0.0, f't={timestep}',
                     horizontalalignment='center',
                     transform=ax_graph.transAxes)
        ax_graph.axis('on')
        
    ### Hide unused subplots
    for idx in range(num_plots, rows * 6):
        row = idx // 6
        col = idx % 6
        fig.add_subplot(left_inner_grid[row, col]).axis('off')
        fig.add_subplot(center_inner_grid[row, col]).axis('off')
        fig.add_subplot(right_inner_grid[row, col]).axis('off')

    plt.tight_layout()
    plt.show()

    exit()


###########################################################


def plot_relationship_graph_custom(relationships_array, timestep):
    fig, ax = plt.subplots(figsize=(12, 8))
    
    G = nx.Graph()
    num_objects = relationships_array.shape[1]
    
    # Add nodes
    G.add_nodes_from(range(num_objects))
    
    # Add edges
    for i in range(num_objects):
        for j in range(i+1, num_objects):  # avoid duplicate edges
            if relationships_array[timestep, i, j]:
                G.add_edge(i, j)
    
    pos = nx.kamada_kawai_layout(G)
    
    # Draw the network with uniform node size
    nx.draw(G, pos,
            ax=ax,
            with_labels=True,
            node_color='lightblue',
            node_size=500,  # uniform size for all nodes
            font_size=12,
            font_weight='bold',
            edge_color='gray',
            width=2)
    
    ax.set_title(f'Object Relationships at Timestep {timestep}')
    plt.tight_layout()
    plt.show()

# Plot with custom visualization
plot_relationship_graph_custom(relationships_array, -1)
exit()

# print(simulation.relationships['timestep_relationships'][0])
# exit()

print(simulation.relationships.keys())
print('num_general_objects:', simulation.relationships['num_general_objects'])
print('num_user_input_objects:', simulation.relationships['num_user_input_objects'])
print('num_timesteps:', len(simulation.relationships['timestep_relationships'])) # 17 if fail the task (1000/60FPS)
print('timestep_relationships[0]:', simulation.relationships['timestep_relationships'][0])
print('timestep_positions_angles[0]:', simulation.relationships['timestep_positions_angles'][0])
print('timestep_relationships[-1]:', simulation.relationships['timestep_relationships'][-1])
print('timestep_positions_angles[-1]:', simulation.relationships['timestep_positions_angles'][-1])

### task_index = 0: 4 bounding boxes, 1 agent, 2 env objects => total 7 objects

# Visualize
if False:
    print('images:', len(simulation.images))
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


