# marker_manager.py
import numpy as np
import logging
from typing import List, Dict, Tuple

class Marker: # marker object
    def __init__(
        self, 
        marker_id: int, 
        corners: Tuple, 
        center: Tuple,
        position_3d: np.ndarray,
    ):
        self.id = marker_id
        self.corners = corners # in pixel
        self.center = center
        self.position_3d = position_3d 

class MarkerManager: # maker mananagment    
    def __init__(
        self, 
        fixed_ids: Dict,
        move_id: int
    ):
  
        self.fixed_ids = fixed_ids
        self.move_id = move_id
        self.fixed_markers: Dict[int, Marker] = {}
        self.moving_marker: Marker = None
        self.reference_defined = False # check if having coordinate system
        self.origin_3d: List = None     
        self.x_axis_3d = None      
        self.y_axis_3d = None     

        self.relative_distances: Dict[int, float] = {}

    def update_markers(
        self, 
        markers: List[Marker]
    ) -> None:

        self.fixed_markers.clear()
        self.moving_marker = None
        for m in markers:
            if m.id in self.fixed_ids:
                self.fixed_markers[m.id] = m
            elif m.id == self.move_id:
                self.moving_marker = m


        self.__update_relative_distance()
        
    
    def __update_relative_distance(self):

        self.relative_distances.clear()
        for fd_id, fd in self.fixed_markers.items():
            if self.moving_marker is not None:
                dist_3d = np.linalg.norm(self.moving_marker.position_3d - fd.position_3d)

            else: dist_3d = 0
            self.relative_distances[fd_id] = dist_3d
    

