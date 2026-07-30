import numpy as np
import os
import cv2
import sys
import time
import yaml
import torch
from localization import Localizer
from utils import *
from topo_graph import TopologicalGraph
from local_grid import LocalGrid
from models import get_place_recognition_model, get_registration_model
from skimage.io import imsave
from threading import Lock
from typing import Dict
from torch import Tensor
import MinkowskiEngine as ME

class TopoSLAMModel():
    def __init__(self, config,
                 path_to_load_graph=None,
                 path_to_save_graph=None,
                 path_to_save_logs=None,
                 trace=None):
        print('Intializing...')
        print('File:', __file__)
        self.path_to_load_graph = path_to_load_graph
        self.path_to_save_graph = path_to_save_graph
        self.path_to_save_logs = path_to_save_logs
        self.trace = trace
        self.current_frame = None
        self.frame_start_perf = None
        if self.path_to_save_logs is not None and not os.path.exists(self.path_to_save_logs):
            os.mkdir(self.path_to_save_logs)
        if self.path_to_save_logs is not None:
            self.path_to_save_iou_results = os.path.join(self.path_to_save_logs, 'test_iou')
        else:
            self.path_to_save_iou_results = None
        self.init_params_from_config(config)

        self.last_vertex = None
        self.last_vertex_id = None
        self.prev_img_front = None
        self.prev_img_back = None
        self.prev_cloud = None
        self.prev_pose_for_visualization = None
        self.prev_rel_pose = None
        self.odom_pose = None
        self.in_sight_response = None
        self.cur_global_pose = None
        self.pose_pairs = []
        layer_names = ['occupancy', 'density_map', 'height_map']
        if config['input']['pointcloud']['subscribe_to_curbs']:
            layer_names.append('curbs')
        self.cur_grid = LocalGrid(resolution=self.grid_resolution, 
                                  radius=self.grid_radius, 
                                  max_range=self.max_grid_range,
                                  floor_height=self.floor_height, ceil_height=self.ceil_height,
                                  layer_names=layer_names,
                                  save_dir=self.path_to_save_iou_results,
                                  trace=self.trace)
        self.rel_poses_stamped = []
        self.rel_pose_of_vcur = None
        self.rel_pose_vcur_to_loc = None
        self.found_loop_closure = False
        self.need_to_change_vcur = False
        self.path = []
        self.device = torch.device('cuda:0')

        self.graph = TopologicalGraph(place_recognition_index=self.place_recognition_index,
                                      inline_registration_model=self.inline_registration_model,
                                      inline_registration_score_threshold=self.inline_registration_score_threshold,
                                      grid_resolution=self.grid_resolution,
                                      grid_radius=self.grid_radius,
                                      max_grid_range=self.max_grid_range,
                                      trace=self.trace)
        if self.path_to_load_graph is not None:
            print('Loading graph from {}'.format(self.path_to_load_graph))
            self.graph.load_from_json(self.path_to_load_graph)
            print('Done!')
            # print('Grid max of graph vertex 0:', self.graph.vertices[0]['grid'].grid.max())
        if self.path_to_save_logs is not None:
            path_to_save_localization_results = os.path.join(self.path_to_save_logs, 'localization_results')
        else:
            path_to_save_localization_results = None
        self.localizer = Localizer(self.graph, 
                                   registration_model=self.registration_model,
                                   registration_score_threshold=self.registration_score_threshold,
                                   save_dir=path_to_save_localization_results,
                                   top_k=self.top_k,
                                   trace=self.trace)

        self.localization_time = 0
        self.last_successful_match_time = 0
        self.cur_iou = 0
        self.localization_results = ([], [])
        self.edge_reattach_cnt = 0
        self.rel_pose_cnt = 0
        self.iou_cnt = 0
        self.local_grid_cnt = 0
        self.current_stamp = None
        self.mutex = Lock()
        print('Initializing done!')

    def set_trace_context(self, frame, stamp):
        self.current_frame = frame
        self.current_stamp = stamp
        self.cur_grid.set_trace_context(frame, stamp)
        self.graph.set_trace_context(frame, stamp)

    def trace_log(self, stage, force=False, **fields):
        if self.trace is not None:
            self.trace.log(stage, frame=self.current_frame, stamp=self.current_stamp,
                           force=force, **fields)

    def trace_summary(self, vertex_before, decision, inside=None, iou=None,
                      rel_dist=None, localization_stamp=None):
        if self.trace is None or not self.trace.enabled:
            return
        edge_count = sum(len(edges) for edges in self.graph.adj_lists) // 2
        self.trace_log(
            'SUMMARY', force=True,
            vertex_before=vertex_before,
            vertex_after=self.last_vertex_id,
            decision=decision,
            inside=inside,
            iou=iou,
            iou_threshold=self.iou_threshold,
            rel_dist=rel_dist,
            max_edge_length=self.max_edge_length,
            LOC_STAMP=localization_stamp,
            localization_stamp=localization_stamp,
            vertices=len(self.graph.vertices),
            undirected_edges=edge_count,
            frame_elapsed_ms=(time.perf_counter() - self.frame_start_perf) * 1000.0
            if self.frame_start_perf is not None else None)

    def init_params_from_config(self, config):
        # TopoMap
        topomap_config = config['topomap']
        self.mode = topomap_config['mode']
        self.localization_timeout = topomap_config['localization_timeout']
        if self.mode not in ['localization', 'mapping']:
            print('Invalid mode {}. Mode can be only "mapping" or "localization"'.format(self.mode))
            exit(1)
        if 'start_location' in topomap_config:
            self.start_location = topomap_config['start_location']
            if 'start_local_pose' in topomap_config:
                self.start_local_pose = topomap_config['start_local_pose']
        else:
            self.start_location = None
        self.iou_threshold = topomap_config['iou_threshold']
        self.localization_frequency = topomap_config['localization_frequency']
        self.rel_pose_correction_frequency = topomap_config['rel_pose_correction_frequency']
        self.max_edge_length = topomap_config['max_edge_length']
        self.drift_coef = topomap_config['drift_coef']
        # Input
        pointcloud_config = config['input']['pointcloud']
        self.floor_height = pointcloud_config['floor_height']
        self.ceil_height = pointcloud_config['ceiling_height']
        # Place recognition
        place_recognition_config = config['place_recognition']
        self.place_recognition_model_type = place_recognition_config['model']
        self.pointcloud_quantization_size = place_recognition_config['pointcloud_quantization_size']
        self.place_recognition_model, self.place_recognition_index = get_place_recognition_model(place_recognition_config)
        self.top_k = place_recognition_config['top_k']
        # Local grid
        grid_config = config['local_occupancy_grid']
        self.grid_resolution = grid_config['resolution']
        self.grid_radius = grid_config['radius']
        self.max_grid_range = grid_config['max_range']
        # Registration
        registration_config = config['scan_matching']
        registration_config['voxel_downsample_size'] = grid_config['resolution']
        if self.path_to_save_logs is not None:
            path_to_save_registration_results = os.path.join(self.path_to_save_logs, 'test_registration')
        else:
            path_to_save_registration_results = None
        self.registration_model = get_registration_model(registration_config, \
                                                         save_dir=path_to_save_registration_results)
        self.registration_score_threshold = registration_config['score_threshold']
        inline_registration_config = config['scan_matching_along_edge']
        inline_registration_config['voxel_downsample_size'] = grid_config['resolution']
        if self.path_to_save_logs is not None:
            path_to_save_inline_registration_results = os.path.join(self.path_to_save_logs, 'test_inline_registration')
        else:
            path_to_save_inline_registration_results = None
        self.inline_registration_model = get_registration_model(inline_registration_config, \
                                                                save_dir=path_to_save_inline_registration_results)
        self.inline_registration_score_threshold = inline_registration_config['score_threshold']
        self.local_jump_threshold = inline_registration_config['jump_threshold']
        # Map frame
        visualization_config = config['visualization']
        self.map_frame = visualization_config['map_frame']

    def correct_rel_pose(self, event=None):
        if self.last_vertex_id is None:
            return
        if self.cur_grid is None:
            return
        rel_x_old, rel_y_old, rel_theta_old = self.get_rel_pose_from_stamp(self.current_stamp)[0]
        cur_grid_transformed = self.cur_grid.copy()
        cur_grid_transformed.transform(rel_x_old, rel_y_old, -rel_theta_old)
        x, y, theta = self.graph.get_transform_to_vertex(self.last_vertex_id, cur_grid_transformed)
        # true_rel_pose = get_rel_pose(*self.last_vertex['pose_for_visualization'], *self.cur_global_pose)
        # print('True rel pose:', true_rel_pose)
        # print('Old rel pose:', rel_x_old, rel_y_old, rel_theta_old)
        # print('Rel pose of vcur:', self.rel_pose_of_vcur)
        # print('True correction:', get_rel_pose(*self.rel_pose_of_vcur, *true_rel_pose))
        # print('Found pose:', x, y, theta)
        if x is not None and np.sqrt(x ** 2 + y ** 2) < 0.5:
            a_inv_v_inv_n = apply_pose_shift((x, y, theta), *self.rel_pose_of_vcur)
            corrected_pose = a_inv_v_inv_n#apply_pose_shift(self.rel_pose_of_vcur, est_x, est_y, theta)
            # print('Corrected pose:', corrected_pose)
            self.rel_pose_of_vcur = corrected_pose
        # else:
            # print('Failed to correct rel pose!')

    def check_path_condition(self, u, v, estimated_transform=None):
        # print('Checking path condition between {} and {}'.format(u, v))
        path, path_length = self.graph.get_path_with_length(u, v)
        if path is None:
            return True
        # print('Path:', path)
        # print('Path length:', path_length)
        # print('Node positions:')
        #for v in path:
        #    print(self.graph.get_vertex(v)['pose_for_visualization'])
        rel_pose_along_path = [0, 0, 0]
        for i in range(1, len(path)):
            rel_pose_along_path = apply_pose_shift(rel_pose_along_path, *self.graph.get_edge(path[i - 1], path[i]))
        # if path_length < 8:
        #     return True
        # print('Path length to vertex {} is {}'.format(v, path_length))
        straight_length = np.sqrt(rel_pose_along_path[0] ** 2 + rel_pose_along_path[1] ** 2)
        # print('Straight length:', straight_length)
        if path_length > 3 * straight_length or straight_length < 10:
            # if estimated_transform is not None:
            #     print('Rel pose along path:', rel_pose_along_path)
            #     print('Estimated transform:', estimated_transform)
            #     diff = np.array(estimated_transform) - np.array(rel_pose_along_path)
            #     error = np.sqrt(diff[0] ** 2 + diff[1] ** 2)
            #     print('Abs diff:', error)
            #     print('Rel diff:', error / straight_length)
            #     return error < 3 or error / straight_length < 0.6 + 0.5 - 1 / len(path)
            return True
        return False

    def get_path_to_metric_goal(self, x, y):
        min_length = np.inf
        best_path = None
        print('Goal coords:', x, y)
        while self.last_vertex is None:
            print('Waiting for localization to create path...')
            time.sleep(0.5)
        for v_id, v in enumerate(self.graph.vertices):
            rel_pose_in_v = get_rel_pose(*v['pose_for_visualization'], x, y, 0)
            if v['grid'].is_inside(*rel_pose_in_v):
                print('Goal is inside vertex {} with coords ({}, {})'.format(v_id, v['pose_for_visualization'][0], v['pose_for_visualization'][1]))
                path, length = self.graph.get_path_with_length(self.last_vertex_id, v_id)
                assert path is not None
                if length < min_length:
                    min_length = length
                    best_path = path
        if best_path is None:
            print('PATH TO METRIC GOAL NOT FOUND!!!')
        return best_path

    def find_loop_closure(self, vertex_ids, dists):
        self.found_loop_closure = False
        path = []
        for i in range(len(vertex_ids)):
            for j in range(len(vertex_ids)):
                u = vertex_ids[i]
                v = vertex_ids[j]
                if u < 0 or v < 0:
                    continue
                path, path_len = self.graph.get_path_with_length(u, v)
                if path is None:
                    self.trace_log('DECISION', candidate_u=u, candidate_v=v,
                                   decision_point='LOOP_CLOSURE',
                                   RESULT='NO_GRAPH_PATH')
                    continue
                dst_through_cur = dists[i] + dists[j]
                # print('Path len: {}, dst through cur: {}'.format(path_len, dst_through_cur))
                length_condition = path_len > 5 and path_len > 2 * dst_through_cur
                path_condition = None
                if length_condition:
                    path_condition = self.check_path_condition(u, v)
                triggered = length_condition and path_condition
                self.trace_log(
                    'DECISION', candidate_u=u, candidate_v=v,
                    decision_point='LOOP_CLOSURE',
                    graph_path=path, path_length=path_len,
                    distance_through_current=dst_through_cur,
                    length_condition=length_condition,
                    path_condition=path_condition,
                    triggered=triggered)
                if triggered:
                    # ux, uy, _ = self.graph.get_vertex(u)['pose_for_visualization']
                    # vx, vy, _ = self.graph.get_vertex(v)['pose_for_visualization']
                    # print('u:', ux, uy)
                    # print('v:', vx, vy)
                    # print('Path in graph:', path_len)
                    # print('Path through cur:', dst_through_cur)
                    print('Loop: connect {} and {} through current position'.format(u, v))
                    self.trace_log(
                        'DECISION', force=True, candidate_u=u, candidate_v=v,
                        decision_point='LOOP_CLOSURE', RESULT='TRIGGERED',
                        graph_path=path, path_length=path_len,
                        distance_through_current=dst_through_cur)
                    self.found_loop_closure = True
                    self.path = path
                    break
            if self.found_loop_closure:
                break
        return self.found_loop_closure

    def is_inside_vcur(self):
        return self.last_vertex['grid'].is_inside(*self.rel_pose_of_vcur)

    def get_rel_pose_from_stamp(self, timestamp, verbose=False):
        if len(self.rel_poses_stamped) == 0:
            self.rel_poses_stamped.append([timestamp] + self.rel_pose_of_vcur)
        j = 0
        while j < len(self.rel_poses_stamped) and self.rel_poses_stamped[j][0] < timestamp:
            j += 1
        if j == len(self.rel_poses_stamped):
            j -= 1
        return self.rel_poses_stamped[j][1:], get_rel_pose(*self.rel_poses_stamped[j][1:], *self.rel_pose_of_vcur)

    def get_rel_pose_since_localization(self):
        if len(self.rel_poses_stamped) == 0:
            return self.rel_pose_of_vcur
        j = 0
        while j < len(self.rel_poses_stamped) and self.rel_poses_stamped[j][0] < self.localizer.localized_stamp:
            j += 1
        if j == len(self.rel_poses_stamped):
            j -= 1
        #print('Rel pose stamped:', self.rel_poses_stamped[j])
        #print(self.rel_poses_stamped[j][1:])
        #print(self.rel_pose_of_vcur)
        rel_pose_after_localization = get_rel_pose(*self.rel_poses_stamped[j][1:], *self.rel_pose_of_vcur)
        #print('Rel pose after localization:', rel_pose_after_localization)
        return rel_pose_after_localization

    def reattach_by_edge(self, require_match=True):
        pose_diffs = []
        edge_poses = []
        neighbours = []
        # print('Vcur:', self.last_vertex_id)
        for vertex_id, pose_to_vertex in self.graph.adj_lists[self.last_vertex_id]:
            edge_poses.append(pose_to_vertex)
            pose_diff = np.sqrt((pose_to_vertex[0] - self.rel_pose_of_vcur[0]) ** 2 + (pose_to_vertex[1] - self.rel_pose_of_vcur[1]) ** 2)
            pose_diffs.append(pose_diff)
            neighbours.append(vertex_id)
            self.trace_log(
                'DECISION', decision_point='EDGE_REATTACH_CANDIDATE',
                current_vertex=self.last_vertex_id,
                neighbour_id=vertex_id,
                edge_pose=pose_to_vertex,
                distance_to_neighbour_prediction=pose_diff)
        dist_to_vcur = np.sqrt(self.rel_pose_of_vcur[0] ** 2 + self.rel_pose_of_vcur[1] ** 2)
        changed = False
        if len(pose_diffs) > 0 and min(pose_diffs) < dist_to_vcur and min(pose_diffs) < 5:
            nearest_vertex_id = neighbours[np.argmin(pose_diffs)]
            pose_on_edge = edge_poses[np.argmin(pose_diffs)]
        else:
            print('Could not find proper edge to change')
            self.trace_log(
                'DECISION', decision_point='EDGE_REATTACH',
                RESULT='NO_PROPER_EDGE',
                current_vertex=self.last_vertex_id,
                distance_to_current_center=dist_to_vcur,
                nearest_prediction_distance=min(pose_diffs) if pose_diffs else None,
                require_match=require_match)
            return False
        print('Nearest vertex id:', nearest_vertex_id)
        # print('\n\n\n                    Pose on edge:', pose_on_edge)
        old_rel_pose_of_vcur = self.rel_pose_of_vcur
        rel_pose_to_vertex = get_rel_pose(*self.rel_pose_of_vcur, *pose_on_edge)
        self.trace_log(
            'DECISION', decision_point='EDGE_REATTACH',
            RESULT='SELECTED_CANDIDATE',
            current_vertex=self.last_vertex_id,
            candidate_id=nearest_vertex_id,
            edge_pose=pose_on_edge,
            distance_to_current_center=dist_to_vcur,
            distance_to_neighbour_prediction=min(pose_diffs),
            relative_pose_to_candidate=rel_pose_to_vertex,
            require_match=require_match)
        print('Rel pose of vcur:', self.rel_pose_of_vcur)
        print('Pose on edge:', pose_on_edge)
        print('Rel pose to vertex:', rel_pose_to_vertex)
        cur_grid_transformed = self.cur_grid.copy()
        rel_pose_to_vertex_inv = self.graph.inverse_transform(*rel_pose_to_vertex)
        cur_grid_transformed.transform(rel_pose_to_vertex_inv[0], rel_pose_to_vertex_inv[1], -rel_pose_to_vertex_inv[2])
        corr_x, corr_y, corr_theta = self.graph.get_transform_to_vertex(nearest_vertex_id, cur_grid_transformed)
        print('Require match:', require_match)
        if corr_x is not None:
            print('Old transform:', rel_pose_to_vertex)
            corr_x_inv, corr_y_inv, corr_theta_inv = self.graph.inverse_transform(corr_x, corr_y, corr_theta)
            x, y, theta = apply_pose_shift(rel_pose_to_vertex, corr_x_inv, corr_y_inv, corr_theta_inv)
            #theta = -theta
            print('Scan matching transform:', x, y, theta)
            diff = np.sqrt((x - rel_pose_to_vertex[0]) ** 2 + (y - rel_pose_to_vertex[1]) ** 2)
            print('Diff:', diff)
            if diff < self.local_jump_threshold:
                print('\n\n\n Change to vertex {} by edge \n\n\n'.format(nearest_vertex_id))
                self.last_successful_match_time = self.current_stamp
                changed = True
                self.rel_pose_of_vcur = self.graph.inverse_transform(x, y, theta)
            else:
                print('Big jump! Ignore this match')
            self.trace_log(
                'DECISION', decision_point='EDGE_REATTACH_MATCH',
                candidate_id=nearest_vertex_id,
                registration_pose=[x, y, theta],
                jump=diff,
                jump_threshold=self.local_jump_threshold,
                accepted=diff < self.local_jump_threshold)
        if not changed and not require_match:
            changed = True
            print('\n\n\n Change to vertex {} by edge without matching\n\n\n'.format(nearest_vertex_id))
            self.rel_pose_of_vcur = self.graph.inverse_transform(*rel_pose_to_vertex)
        if changed:
            self.last_vertex_id = nearest_vertex_id
            self.need_to_change_vcur = False
            self.last_vertex = self.graph.get_vertex(self.last_vertex_id)
            if self.rel_pose_vcur_to_loc is not None:
                self.rel_pose_vcur_to_loc = apply_pose_shift(self.graph.inverse_transform(*pose_on_edge), *self.rel_pose_vcur_to_loc)
            print('Reset rel_poses_stamped to time', self.current_stamp)
            self.rel_poses_stamped = [[self.current_stamp] + self.rel_pose_of_vcur]
            self.trace_log(
                'DECISION', force=True, decision_point='EDGE_REATTACH',
                RESULT='SWITCHED', vertex_after=self.last_vertex_id,
                relative_pose_after=self.rel_pose_of_vcur,
                required_registration=require_match)
        else:
            print('Failed to match current cloud to vertex {}!'.format(nearest_vertex_id))
            self.trace_log(
                'DECISION', decision_point='EDGE_REATTACH',
                RESULT='REGISTRATION_REJECTED', candidate_id=nearest_vertex_id)

        return changed

    def reattach_by_localization(self, iou_threshold, localized_stamp):
        vertex_ids = self.localization_results['vertex_ids_matched']
        rel_poses = self.localization_results['rel_poses']
        print('Reattach by localization')
        if len(self.rel_poses_stamped) > 0 and localized_stamp < self.rel_poses_stamped[0][0]:
            print('Old localization 1! Ignore it')
            print((self.rel_poses_stamped[0][0] - localized_stamp))
            self.trace_log(
                'DECISION', decision_point='LOCALIZATION_REATTACH',
                RESULT='REJECTED_OLD_LOCALIZATION',
                LOC_STAMP=localized_stamp,
                oldest_motion_stamp=self.rel_poses_stamped[0][0],
                age=self.rel_poses_stamped[0][0] - localized_stamp)
            return False
        found_proper_vertex = False
        self.rel_pose_vcur_to_loc, _ = self.get_rel_pose_from_stamp(localized_stamp)
        # First try to pass the nearest edge
        for i, v in enumerate(vertex_ids):
            # if v == self.last_vertex_id:
            #     continue
            pred_rel_pose_vcur_to_v = apply_pose_shift(self.rel_pose_vcur_to_loc, *self.graph.inverse_transform(*rel_poses[i]))
            rel_pose_robot_to_loc = get_rel_pose(*self.get_rel_pose_since_localization(), *rel_poses[i])
            print('Rel pose robot to loc:', rel_pose_robot_to_loc)
            iou = self.cur_grid.get_iou(self.graph.get_vertex(v)['grid'], *rel_pose_robot_to_loc, save=False)
            vx, vy, vtheta = self.graph.get_vertex(v)['pose_for_visualization']
            x, y, theta = get_rel_pose(*self.last_vertex['pose_for_visualization'], 
                                       *self.graph.get_vertex(v)['pose_for_visualization'])
            dx, dy, dtheta = get_rel_pose(x, y, theta, *self.rel_pose_of_vcur)
            dst = np.sqrt(dx ** 2 + dy ** 2)
            drift_limit = self.drift_coef * (self.current_stamp - self.last_successful_match_time) + 10
            too_far = dst > drift_limit
            drift_ok = not too_far
            self.trace_log(
                'DECISION', decision_point='LOCALIZATION_REATTACH_CANDIDATE',
                LOC_STAMP=localized_stamp,
                candidate_id=v,
                localization_relative_pose=rel_poses[i],
                motion_compensated_pose=rel_pose_robot_to_loc,
                candidate_iou=iou,
                iou_threshold=iou_threshold,
                drift_distance=dst,
                drift_limit=drift_limit,
                drift_ok=drift_ok,
                need_to_change_vcur=self.need_to_change_vcur)
            if too_far:
                print('Vertex {} is too far to match'.format(v))
                continue
            print('v:', v)
            print('IoU between current state and ({}, {}) is {}'.format(vx, vy, iou))
            print('IoU threshold is {}'.format(iou_threshold))
            print('Need to change vcur:', self.need_to_change_vcur)
            if iou > iou_threshold or self.need_to_change_vcur:
                self.last_successful_match_time = localized_stamp
                found_proper_vertex = True
                print('\n\n\n Change to vertex {} with coords ({}, {})\n\n\n'.format(v, vx, vy))
                #last_x, last_y, last_theta = self.last_vertex['pose_for_visualization']
                if self.mode == 'mapping':
                    self.graph.add_edge(self.last_vertex_id, v, *pred_rel_pose_vcur_to_v,
                                        edge_type='LOCALIZATION')
                self.last_vertex_id = v
                self.need_to_change_vcur = False
                self.last_vertex = self.graph.get_vertex(v)
                _, rel_pose_after_localization = self.get_rel_pose_from_stamp(localized_stamp, verbose=True)
                pred_rel_pose = apply_pose_shift(rel_poses[i], *rel_pose_after_localization)
                self.rel_pose_cnt += 1
                self.rel_pose_of_vcur = pred_rel_pose
                self.rel_pose_vcur_to_loc = apply_pose_shift(self.graph.inverse_transform(*pred_rel_pose_vcur_to_v), *self.rel_pose_vcur_to_loc)
                self.rel_poses_stamped = [[self.current_stamp] + self.rel_pose_of_vcur]
                #self.localization_time = 0
                self.trace_log(
                    'DECISION', force=True,
                    decision_point='LOCALIZATION_REATTACH',
                    RESULT='SWITCHED',
                    LOC_STAMP=localized_stamp,
                    candidate_id=v,
                    relative_pose_after=self.rel_pose_of_vcur)
                return True
        self.trace_log(
            'DECISION', decision_point='LOCALIZATION_REATTACH',
            RESULT='NO_ACCEPTED_CANDIDATE', LOC_STAMP=localized_stamp)
        return False

    def add_new_vertex(self, vertex_ids, rel_poses):
        vertex_count_before = len(self.graph.vertices)
        new_vertex_id = self.graph.add_vertex(self.global_pose_for_visualization, self.cur_desc, self.cur_grid)
        new_vertex = self.graph.get_vertex(new_vertex_id)
        pose_stamped, new_rel_pose_of_vcur = self.get_rel_pose_from_stamp(self.current_stamp)
        if self.last_vertex is not None:
            #true_rel_pose = get_rel_pose(*self.last_vertex['pose_for_visualization'], *new_vertex['pose_for_visualization'])
            self.graph.add_edge(self.last_vertex_id, new_vertex_id, *pose_stamped,
                                edge_type='SEQUENTIAL')
        self.rel_pose_of_vcur = new_rel_pose_of_vcur
        if self.rel_pose_vcur_to_loc is not None:
            self.rel_pose_vcur_to_loc = get_rel_pose(*pose_stamped, *self.rel_pose_vcur_to_loc)
        # print('Vertex ids: {}, rel poses: {}'.format(vertex_ids, rel_poses))
        for v, rel_pose in zip(vertex_ids, rel_poses):
            # print('Rel pose vcur to loc:', self.rel_pose_vcur_to_loc)
            # print('Rel pose:', rel_pose)
            if self.rel_pose_vcur_to_loc is not None and rel_pose is not None:
                pred_rel_pose = apply_pose_shift(self.rel_pose_vcur_to_loc, *self.graph.inverse_transform(*rel_pose))
                #if np.sqrt(pred_rel_pose[0] ** 2 + pred_rel_pose[1] ** 2) < 5:
                edge_type = 'LOOP_CANDIDATE' if self.found_loop_closure else 'LOCALIZATION'
                self.graph.add_edge(new_vertex_id, v, *pred_rel_pose, edge_type=edge_type)
        self.rel_poses_stamped = [[self.current_stamp] + self.rel_pose_of_vcur]
        self.last_vertex_id = new_vertex_id
        self.need_to_change_vcur = False
        self.last_vertex = new_vertex
        self.trace_log(
            'VERTEX', force=True, RESULT='CURRENT_VERTEX_UPDATED',
            vertex_count_before=vertex_count_before,
            new_vertex_id=new_vertex_id,
            vertex_count_after=len(self.graph.vertices),
            relative_pose_after=self.rel_pose_of_vcur)

    def init_localization(self):
        if self.start_location is not None:
            self.last_vertex_id = self.start_location
            self.need_to_change_vcur = False
            self.last_vertex = self.graph.get_vertex(self.start_location)
            if self.start_local_pose is not None:
                self.rel_pose_of_vcur = self.start_local_pose
                self.rel_pose_vcur_to_loc = self.rel_pose_of_vcur
                self.rel_poses_stamped = [[self.current_stamp] + self.rel_pose_of_vcur]
        else:
            localized_state = self.localizer.get_localized_state()
            start_time = time.time()
            while localized_state['vertex_ids_matched'] is None or len(localized_state['vertex_ids_matched']) == 0:
                print('Still waiting for localization... Try to move forward-backward slightly')
                time.sleep(1.0)
                localized_state = self.localizer.get_localized_state()
                if self.mode == 'mapping' and time.time() - start_time > self.localization_timeout:
                    print('Localization timed out. Add new vertex at start')
                    self.add_new_vertex([], [])
                    break
            vertex_ids = localized_state['vertex_ids_matched']
            rel_poses = localized_state['rel_poses']
            if vertex_ids is not None and len(vertex_ids) > 0:
                print('Initially attached to vertex {} from localization'.format(vertex_ids[0]))
                self.last_vertex_id = vertex_ids[0]
                self.need_to_change_vcur = False
                self.last_vertex = self.graph.get_vertex(vertex_ids[0])
                self.rel_pose_of_vcur = self.graph.inverse_transform(*rel_poses[0])
                print('Rel pose of vcur is set to', self.rel_pose_of_vcur)
                self.rel_pose_vcur_to_loc = self.rel_pose_of_vcur
                self.current_stamp = self.localizer.localized_stamp
                self.rel_poses_stamped = [[self.current_stamp] + self.rel_pose_of_vcur]

    def _preprocess_input(self, input_data: Dict[str, Tensor]) -> Dict[str, Tensor]:
        """Preprocess input data."""
        out_dict: Dict[str, Tensor] = {}
        for key in input_data:
            if key.startswith("image_"):
                out_dict[f"images_{key[6:]}"] = input_data[key].unsqueeze(0).to(self.device)
            elif key.startswith("mask_"):
                out_dict[f"masks_{key[5:]}"] = input_data[key].unsqueeze(0).to(self.device)
            elif key == "pointcloud_lidar_coords":
                quantized_coords, quantized_feats = ME.utils.sparse_quantize(
                    coordinates=input_data["pointcloud_lidar_coords"],
                    features=input_data["pointcloud_lidar_feats"],
                    quantization_size=self.pointcloud_quantization_size,
                )
                self.last_sparse_voxel_count = len(quantized_coords)
                out_dict["pointclouds_lidar_coords"] = ME.utils.batched_coordinates([quantized_coords]).to(
                    self.device
                )
                out_dict["pointclouds_lidar_feats"] = quantized_feats.to(self.device)
        return out_dict

    def process_observations(self, img_front, img_back, cur_cloud, cur_curbs, x, y, theta):
        # Extract descriptor from cloud and images
        trace_enabled = self.trace is not None and self.trace.enabled
        descriptor_start = time.perf_counter() if trace_enabled else None
        finite_points = int(np.isfinite(cur_cloud[:, :3]).all(axis=1).sum()) if trace_enabled else None
        self.last_sparse_voxel_count = None
        input_data = {
                     'pointcloud_lidar_coords': torch.Tensor(cur_cloud[:, :3]).cuda(),
                     'pointcloud_lidar_feats': torch.ones((cur_cloud.shape[0], 1)).cuda(),
                     }
        if img_front is not None:
            img_front_tensor = torch.Tensor(img_front).cuda()
            img_front_tensor = torch.permute(img_front_tensor, (2, 0, 1))
            input_data['image_front'] = img_front_tensor
        if img_back is not None:
            img_back_tensor = torch.Tensor(img_back).cuda()
            img_back_tensor = torch.permute(img_back_tensor, (2, 0, 1))
            input_data['image_back'] = img_back_tensor
        batch = self._preprocess_input(input_data)
        try:
            self.cur_desc = self.place_recognition_model(batch)["final_descriptor"].detach().cpu().numpy()
        except Exception as exc:
            self.trace_log(
                'DESCRIPTOR', force=True, RESULT='DESCRIPTOR_FAILED',
                exception_type=type(exc).__name__,
                exception_message=str(exc))
            raise
        if len(self.cur_desc.shape) == 1:
                self.cur_desc = self.cur_desc[np.newaxis, :]
        if trace_enabled:
            descriptor_elapsed_ms = (time.perf_counter() - descriptor_start) * 1000.0
            descriptor_flat = self.cur_desc.reshape(-1)
            descriptor_head_size = self.trace.descriptor_head_size
            self.trace_log(
                'DESCRIPTOR',
                IMPLEMENTATION='LOCAL_MODEL',
                input_points=len(cur_cloud),
                finite_points=finite_points,
                sparse_voxels=self.last_sparse_voxel_count,
                quantization_size=self.pointcloud_quantization_size,
                front_image_used=img_front is not None,
                back_image_used=img_back is not None,
                front_image_shape=list(img_front.shape) if img_front is not None else None,
                back_image_shape=list(img_back.shape) if img_back is not None else None,
                model_type=self.place_recognition_model_type,
                descriptor_dim=int(descriptor_flat.size),
                descriptor_l2_norm=float(np.linalg.norm(descriptor_flat)),
                descriptor_head=descriptor_flat[:descriptor_head_size].tolist(),
                elapsed_ms=descriptor_elapsed_ms)
        # Project cloud into a grid
        self.cur_grid.update_from_cloud_and_transform(cur_cloud, x, y, -theta)
        if cur_curbs is not None:
            self.cur_grid.update_curbs_from_cloud(cur_curbs)
        # else:
        #     print('NO CURBS!')

    def update_rel_pose_of_vcur_by_odom(self, cur_odom_pose):
        x, y, theta = cur_odom_pose
        # print('Odom pose:', self.odom_pose)
        # print('Cur odom pose:', cur_odom_pose)
        previous_odom = list(self.odom_pose) if self.odom_pose is not None else None
        rel_pose_before = list(self.rel_pose_of_vcur) if self.rel_pose_of_vcur is not None else None
        first_odom = getattr(self, '_odom_first_frame', self.odom_pose is None)
        if self.odom_pose is not None:
            rel_x, rel_y, rel_theta = get_rel_pose(*self.odom_pose, x, y, theta)
        else:
            rel_x, rel_y, rel_theta = x, y, theta
        self.odom_pose = [x, y, theta]
        if self.rel_pose_of_vcur is None:
            print('Rel pose of vcur is None, initialize it as ({}, {}, {})'.format(rel_x, rel_y, rel_theta))
            self.rel_pose_of_vcur = [rel_x, rel_y, rel_theta]
        else:
            # print('Apply pose shift:', rel_x, rel_y, rel_theta)
            self.rel_pose_of_vcur = apply_pose_shift(self.rel_pose_of_vcur, rel_x, rel_y, rel_theta)
            # print('Update rel pose of vcur from odom:', self.rel_pose_of_vcur)
        self.rel_poses_stamped.append([self.current_stamp] + self.rel_pose_of_vcur)
        self.trace_log(
            'ODOM',
            prev_odom=previous_odom,
            cur_odom=cur_odom_pose,
            delta=[rel_x, rel_y, rel_theta],
            rel_pose_before=rel_pose_before,
            rel_pose_after=self.rel_pose_of_vcur,
            first_frame_initialization=first_odom)

    def update(self, global_pose_for_visualization, cur_odom_pose, img_front, img_back, cur_cloud, cur_curbs):
        self.frame_start_perf = time.perf_counter()
        vertex_before = self.last_vertex_id
        vertex_count_before = len(self.graph.vertices)
        decision = 'KEEP'
        was_unattached = self.last_vertex is None
        self._odom_first_frame = self.odom_pose is None
        if self.odom_pose is None:
            self.odom_pose = cur_odom_pose
        x, y, theta = get_rel_pose(*cur_odom_pose, *self.odom_pose)
        self.trace_log(
            'ODOM',
            grid_shift=[x, y, -theta],
            grid_shift_source='INVERSE_CURRENT_TO_PREVIOUS',
            first_frame_initialization=self._odom_first_frame)
        # Update rel_pose_of_vcur by odometry
        self.update_rel_pose_of_vcur_by_odom(cur_odom_pose)
        # Update localizer and localized state
        self.process_observations(img_front, img_back, cur_cloud, cur_curbs, x, y, theta)
        self.global_pose_for_visualization = global_pose_for_visualization
        self.localizer.update_current_state(self.global_pose_for_visualization, self.cur_desc,
                                            self.cur_grid, self.current_stamp,
                                            frame=self.current_frame)
        if self.last_vertex is None:
            self.init_localization()
            if self.last_vertex is not None:
                decision = 'FIRST_VERTEX' if vertex_count_before == 0 else 'LOCALIZATION_SWITCH'
        self.localization_results = self.localizer.get_localized_state()
        if self.localization_results is not None and self.localization_results['timestamp'] is not None:
            self.localization_time = self.localization_results['timestamp']
        localized_stamp = self.localization_results['timestamp']
        localization_is_fresh = (len(self.rel_poses_stamped) == 0 or localized_stamp is None or localized_stamp >= self.rel_poses_stamped[0][0] - 1e-3)
        localized_frame = self.localizer.last_localized_frame_read
        motion_compensation = None
        if (self.trace is not None and self.trace.enabled and
                localized_stamp is not None and len(self.rel_poses_stamped) > 0):
            motion_compensation = self.get_rel_pose_from_stamp(localized_stamp)[1]
        self.trace_log(
            'LOCALIZATION_RESULT',
            RESULT='CONSUMED',
            LOC_STAMP=localized_stamp,
            localization_frame=localized_frame,
            consumer_frame=self.current_frame,
            current_minus_localized_stamp=self.current_stamp - localized_stamp
            if localized_stamp is not None else None,
            fresh=localization_is_fresh,
            motion_compensation=motion_compensation,
            matched_vertices=self.localization_results['vertex_ids_matched'],
            unmatched_vertices=self.localization_results['vertex_ids_unmatched'])
        print('Localized in vertices: {}. Actual: {}'.format(self.localization_results['vertex_ids_matched'], 
                                                                          localization_is_fresh))
        if localization_is_fresh:
            self.rel_pose_vcur_to_loc = self.get_rel_pose_from_stamp(localized_stamp)[0]
        # print('Rel pose vcur to loc:', self.rel_pose_vcur_to_loc)
        # print('Rel poses:', self.localization_results['rel_poses'])

        if self.mode == 'mapping' and localization_is_fresh:
            vertex_ids = list(self.localization_results['vertex_ids_matched'])
            rel_poses = list(self.localization_results['rel_poses'])
            if vertex_ids is not None and rel_poses is not None:
                if self.last_vertex_id not in vertex_ids:
                    vertex_ids.append(self.last_vertex_id)
                    rel_poses.append(self.graph.inverse_transform(*self.rel_pose_vcur_to_loc))
                dists = [np.sqrt(x ** 2 + y ** 2) for x, y, theta in rel_poses]
                if self.find_loop_closure(vertex_ids, dists):
                    print('\n\n\nFound loop closure. Add new vertex to close loop\n\n\n')
                    self.add_new_vertex(vertex_ids, rel_poses)
                    decision = 'LOOP_NEW_VERTEX'
                    self.trace_summary(vertex_before, decision,
                                       localization_stamp=localized_stamp)
                    return
        else:
            print('Could not check loop closure - old localization')

        if cur_cloud is None:
            print('No point cloud received!')
            self.trace_log('DECISION', force=True, RESULT='EMPTY_CLOUD')
            self.trace_summary(vertex_before, 'EMPTY_CLOUD',
                               localization_stamp=localized_stamp)
            return
        changed = self.reattach_by_edge(require_match=True)
        if changed:
            decision = 'EDGE_SWITCH'
        #last_x, last_y, _ = self.last_vertex['pose_for_visualization']
        inside_vcur = self.is_inside_vcur()
        # print('Rel pose of vcur:', self.rel_pose_of_vcur)
        iou = self.cur_grid.get_iou(self.last_vertex['grid'], *self.graph.inverse_transform(*self.rel_pose_of_vcur), \
                               save=False, cnt=self.iou_cnt)
        self.iou_cnt += 1
        self.cur_iou = iou
        # print('IoU:', iou)
        # print('Vcur:', self.last_vertex_id)
        dst = np.sqrt(self.rel_pose_of_vcur[0] ** 2 + self.rel_pose_of_vcur[1] ** 2)
        self.trace_log(
            'DECISION',
            decision_point='CURRENT_VERTEX_HOLD',
            inside=inside_vcur,
            iou=iou,
            iou_threshold=self.iou_threshold,
            rel_dist=dst,
            max_edge_length=self.max_edge_length,
            keep=inside_vcur and iou >= self.iou_threshold and dst <= self.max_edge_length)
        if not inside_vcur or iou < self.iou_threshold or dst > self.max_edge_length:
            self.need_to_change_vcur = True
            if not inside_vcur:
                print('Moved outside vcur {}'.format(self.last_vertex_id))
            elif iou < self.iou_threshold:
                print('Low IoU {}'.format(iou))
            else:
                print('Too far from location center')
            #print(self.path[0], self.last_vertex_id)
            print('Changed:', changed)
            # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            if not changed:
                print('Localization dt:', self.current_stamp - self.localization_time)
                if self.current_stamp - self.localization_results['timestamp'] < 5:
                    #print('Localized stamp:', self.localizer.localized_stamp)
                    changed = self.reattach_by_localization(self.cur_iou, self.localization_results['timestamp'])
                    print('Changed from localization:', changed)
                    if changed:
                        decision = 'LOCALIZATION_SWITCH'
                    if not changed:
                        if self.mode == 'mapping':
                            print('No proper vertex to change. Add new vertex')
                            if localization_is_fresh:
                                vertex_ids = self.localization_results['vertex_ids_matched']
                                rel_poses = self.localization_results['rel_poses']
                            else:
                                vertex_ids = []
                                rel_poses = []
                            self.add_new_vertex(vertex_ids, rel_poses)
                            decision = 'NEW_VERTEX'
                else:
                    if self.mode == 'mapping':
                        print('No recent localization. Add new vertex')
                        self.add_new_vertex([], [])
                        decision = 'NEW_VERTEX'
                    else:
                        print('No recent localization')
                        decision = 'WAIT_LOCALIZATION'
                    #     self.reattach_by_edge(require_match=False)
            # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            if not changed and self.mode == 'localization':
                fallback_changed = self.reattach_by_edge(require_match=False)
                decision = 'LOCALIZATION_FALLBACK' if fallback_changed else 'WAIT_LOCALIZATION'
        elif was_unattached and decision == 'KEEP':
            decision = 'FIRST_VERTEX'
        self.trace_summary(vertex_before, decision, inside=inside_vcur, iou=iou,
                           rel_dist=dst, localization_stamp=localized_stamp)

    def save_graph(self):
        if self.path_to_save_graph is not None:
            self.graph.save_to_json(self.path_to_save_graph)
        print('N of localizer calls:', self.localizer.cnt)
        print('N of localization fails:', self.localizer.n_loc_fails)
