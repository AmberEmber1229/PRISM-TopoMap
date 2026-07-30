import json
import heapq
import numpy as np
import time
import os
from collections import deque
from typing import Dict

import faiss
import torch
from scipy.spatial.transform import Rotation
from skimage.io import imread, imsave
from opr.pipelines.registration import PointcloudRegistrationPipeline, RansacGlobalRegistrationPipeline, Feature2DGlobalRegistrationPipeline
from opr.datasets.augmentations import DefaultHM3DImageTransform
from local_grid import load_local_grid
from utils import get_rel_pose

from memory_profiler import profile

class TopologicalGraph():
    def __init__(self,
                 place_recognition_index,
                 inline_registration_model,
                 inline_registration_score_threshold=0.5,
                 grid_resolution=0.1,
                 grid_radius=18.0,
                 max_grid_range=8.0,
                 trace=None):
        self.vertices = []
        self.adj_lists = []
        self.index = place_recognition_index
        self.inline_registration_pipeline = inline_registration_model
        self.inline_registration_score_threshold = inline_registration_score_threshold
        self.grid_resolution = grid_resolution
        self.grid_radius = grid_radius
        self.max_grid_range = max_grid_range
        self.device = torch.device('cuda:0')
        self.trace = trace
        self.trace_frame = None
        self.trace_stamp = None

    def set_trace_context(self, frame, stamp):
        self.trace_frame = frame
        self.trace_stamp = stamp

    def normalize(self, angle):
        while angle < -np.pi:
            angle += 2 * np.pi
        while angle > np.pi:
            angle -= 2 * np.pi
        return angle

    def load_from_json(self, input_path):
        fin = open(os.path.join(input_path, 'graph.json'), 'r')
        j = json.load(fin)
        fin.close()
        self.vertices = j['vertices']
        self.adj_lists = j['edges']
        for i in range(len(self.vertices)):
            #grid = np.load(os.path.join(input_path, '{}_grid.npz'.format(i)))['arr_0']
            self.vertices[i]['grid'] = load_local_grid(os.path.join(input_path, str(i)))
            #img_front = imread(os.path.join(input_path, '{}_img_front.png'.format(i)))
            #self.vertices[i]['img_front'] = img_front
            #img_back = imread(os.path.join(input_path, '{}_img_back.png'.format(i)))
            #self.vertices[i]['img_back'] = img_back
            self.index.add(np.array(self.vertices[i]['descriptor'])[np.newaxis, :])

    #@profile
    def add_vertex(self, global_pose_for_visualization, descriptor=None, grid=None):
        x, y, theta = global_pose_for_visualization
        vertex_count_before = len(self.vertices)
        index_size_before = int(self.index.ntotal)
        # Replaced by the unified, opt-in [FLOW] VERTEX event below.
        # print('\n\n\n Add new vertex ({}, {}, {}) with idx {} \n\n\n'.format(x, y, theta, len(self.vertices)))
        if grid is not None:
            self.adj_lists.append([])
            vertex_dict = {
                'pose_for_visualization': [x, y, theta],
                'grid': grid.copy(),
                'descriptor': descriptor
            }
            self.vertices.append(vertex_dict)
            #print('Descriptor shape:', descriptor.shape)
            self.index.add(descriptor)
        new_vertex_id = len(self.vertices) - 1
        if self.trace is not None and self.trace.enabled:
            occupancy = grid.layers['occupancy'] if grid is not None else None
            descriptor_array = np.asarray(descriptor) if descriptor is not None else None
            self.trace.log(
                'VERTEX', frame=self.trace_frame, stamp=self.trace_stamp, force=True,
                RESULT='ADDED' if grid is not None else 'SKIPPED_NO_GRID',
                vertex_count_before=vertex_count_before,
                new_vertex_id=new_vertex_id,
                global_pose=[x, y, theta],
                descriptor_dim=int(descriptor_array.shape[-1]) if descriptor_array is not None else None,
                occupancy_unknown=int((occupancy == 0).sum()) if occupancy is not None else None,
                occupancy_free=int((occupancy == 1).sum()) if occupancy is not None else None,
                occupancy_occupied=int((occupancy >= 2).sum()) if occupancy is not None else None,
                index_size_before=index_size_before,
                index_size_after=int(self.index.ntotal))
        return new_vertex_id

    def get_transform_to_vertex(self, vertex_id, grid):
        trace_registration = (self.trace is not None and self.trace.enabled and
                              self.trace.registration_candidates)
        registration_start = time.perf_counter() if trace_registration else None
        # Replaced by the unified, opt-in [FLOW] REGISTRATION record below.
        # print('Trying to match to vertex {}'.format(vertex_id))
        #return get_rel_pose(*self.global_pose_for_visualization, *self.vertices[vertex_id]['pose_for_visualization'])
        cand_grid = self.vertices[vertex_id]['grid']
        cand_grid_tensor = torch.Tensor(cand_grid.layers['occupancy']).to(self.device)
        ref_grid_tensor = torch.Tensor(grid.layers['occupancy']).to(self.device)
        #print('                Ref grid:', grid.max())
        #print('                Cand grid:', cand_grid.max())

        try:
            transform, score = self.inline_registration_pipeline.infer(
                ref_grid_tensor, cand_grid_tensor, verbose=False)
        except Exception as exc:
            if self.trace is not None and self.trace.enabled:
                self.trace.log(
                    'REGISTRATION', frame=self.trace_frame, stamp=self.trace_stamp,
                    force=True, RESULT='REGISTRATION_FAILED',
                    candidate_id=vertex_id, registration_type='inline',
                    exception_type=type(exc).__name__,
                    exception_message=str(exc))
            raise
        # print('TRANSFORM:', transform)
        # print('Score:', score)
        if score > self.inline_registration_score_threshold:
            tf_matrix = cand_grid.get_tf_matrix_xy(*transform)
            x = tf_matrix[0, 3]
            y = tf_matrix[1, 3]
            _, __, theta = Rotation.from_matrix(tf_matrix[:3, :3]).as_rotvec()
            if trace_registration:
                self.trace.log(
                    'REGISTRATION', frame=self.trace_frame, stamp=self.trace_stamp,
                    candidate_id=vertex_id, registration_type='inline',
                    ref_shape=list(grid.layers['occupancy'].shape),
                    candidate_shape=list(cand_grid.layers['occupancy'].shape),
                    ref_nonzero=int(np.count_nonzero(grid.layers['occupancy'])),
                    candidate_nonzero=int(np.count_nonzero(cand_grid.layers['occupancy'])),
                    score=float(score), threshold=self.inline_registration_score_threshold,
                    transform=list(transform), relative_pose_m=[x, y, theta],
                    matched=True,
                    elapsed_ms=(time.perf_counter() - registration_start) * 1000.0)
            # print('X Y THETA:', x, y, theta)
            return x, y, theta
        if trace_registration:
            self.trace.log(
                'REGISTRATION', frame=self.trace_frame, stamp=self.trace_stamp,
                candidate_id=vertex_id, registration_type='inline',
                ref_shape=list(grid.layers['occupancy'].shape),
                candidate_shape=list(cand_grid.layers['occupancy'].shape),
                ref_nonzero=int(np.count_nonzero(grid.layers['occupancy'])),
                candidate_nonzero=int(np.count_nonzero(cand_grid.layers['occupancy'])),
                score=float(score), threshold=self.inline_registration_score_threshold,
                transform=list(transform), relative_pose_m=None, matched=False,
                elapsed_ms=(time.perf_counter() - registration_start) * 1000.0)
        return None, None, None

    def inverse_transform(self, x, y, theta):
        x_inv = -x * np.cos(theta) - y * np.sin(theta)
        y_inv = x * np.sin(theta) - y * np.cos(theta)
        theta_inv = -theta
        return [x_inv, y_inv, theta_inv]
    
    def add_edge(self, i, j, x, y, theta, edge_type='UNSPECIFIED'):
        if i == j:
            return
        if j in [x[0] for x in self.adj_lists[i]]:
            return
        xi, yi, _ = self.vertices[i]['pose_for_visualization']
        xj, yj, _ = self.vertices[j]['pose_for_visualization']
        # Replaced by the unified, opt-in [FLOW] EDGE event below.
        # print('\nAdd edge from ({}, {}) to ({}, {}) with rel pose ({}, {}, {})\n'.format(xi, yi, xj, yj, x, y, theta))
        self.adj_lists[i].append((int(j), [x, y, theta]))
        self.adj_lists[j].append((int(i), self.inverse_transform(x, y, theta)))
        if self.trace is not None and self.trace.enabled:
            self.trace.log(
                'EDGE', frame=self.trace_frame, stamp=self.trace_stamp, force=True,
                RESULT='ADDED', source=i, target=j,
                relative_pose=[x, y, theta],
                length=float(np.sqrt(x ** 2 + y ** 2)),
                edge_type=edge_type,
                length_validation='NOT_PRESENT_IN_PYTHON_ORIGINAL')

    def get_vertex(self, vertex_id):
        return self.vertices[vertex_id]

    def has_edge(self, u, v):
        for x, rel_pose in self.adj_lists[u]:
            if x == v:
                return True
        return False

    def get_edge(self, u, v):
        for x, rel_pose in self.adj_lists[u]:
            if x == v:
                return rel_pose
        return None

    def get_edges_from(self, u):
        return self.adj_lists[u]

    def get_path_with_length(self, u, v):
        # Initialize distances and previous nodes dictionaries
        start_time = time.time()
        distances = [float('inf')] * len(self.adj_lists)
        prev_nodes = [None] * len(self.adj_lists)
        # Set distance to start node as 0
        distances[u] = 0
        # Create priority queue with initial element (distance to start node, start node)
        heap = [(0, u)]
        # Run Dijkstra's algorithm
        while heap:
            # Pop node with lowest distance from heap
            current_distance, current_node = heapq.heappop(heap)
            if current_node == v:
                path = [current_node]
                cur = current_node
                while cur != u:
                    cur = prev_nodes[cur]
                    path.append(cur)
                path = path[::-1]
                #print('Path planning time:', time.time() - start_time)
                return path, distances[v]
            # If current node has already been visited, skip it
            if current_distance > distances[current_node]:
                continue
            # For each neighbour of current node
            for neighbour, pose in self.adj_lists[current_node]:
                weight = np.sqrt(pose[0] ** 2 + pose[1] ** 2)
                # Calculate tentative distance to neighbour through current node
                tentative_distance = current_distance + weight
                # Update distance and previous node if tentative distance is better than current distance
                if tentative_distance < distances[neighbour]:
                    distances[neighbour] = tentative_distance
                    prev_nodes[neighbour] = current_node
                    # Add neighbour to heap with updated distance
                    heapq.heappush(heap, (tentative_distance, neighbour))
        #print('Path planning time:', time.time() - start_time)
        return None, float('inf')

    def save_to_json(self, output_path):
        if not os.path.exists(output_path):
            os.mkdir(output_path)
        self.vertices = list(self.vertices)
        vertices_to_save = []
        for i in range(len(self.vertices)):
            vertex_dict = self.vertices[i]
            vertex_dict['grid'].save(os.path.join(output_path, str(i)))
            x, y, theta = vertex_dict['pose_for_visualization']
            descriptor = vertex_dict['descriptor']
            descriptor = np.array(descriptor)
            if descriptor.ndim == 2:
                descriptor = descriptor[0]
            vertices_to_save.append({'pose_for_visualization': (x, y, theta), 'descriptor': [float(x) for x in list(descriptor)]})
        j = {'vertices': vertices_to_save, 'edges': self.adj_lists}
        fout = open(os.path.join(output_path, 'graph.json'), 'w')
        json.dump(j, fout)
        fout.close()
