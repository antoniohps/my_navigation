# encode: utf-8
import os
import numpy as np

import yaml
import cv2
from PIL import Image as PilImage

class OccupancyGrid(object):
    """ Stores an occupancy grid map, which indicates the probability of occupancy.
        Attributes:
            map: the occupancy grid map to localize against (nav_msgs/OccupancyGrid)
    """
    def __init__(self, mapfilename):
        
        if isinstance(mapfilename, str):
            self._dir = os.path.dirname(os.path.abspath(mapfilename))
            mapfile = open(mapfilename, 'r')
            mapdata = yaml.load(mapfile)
            mapfile.close()
        else: raise TypeError("map must be a YAML filename")

        fname =  mapdata['image']
        self._resolution = mapdata['resolution']
        self._origin = mapdata['origin']
        self._occupied_thresh = mapdata['occupied_thresh']
        self._free_thresh = mapdata['free_thresh']

        # Open image file relatively to the YAML file if path is relative 
        fname = fname if os.path.isabs(fname) else os.path.join(self._dir, (fname))
        self._mapimage = fname
        map = PilImage.open(fname, 'r')
        map = np.asarray(map).astype(np.uint8)
        
        # Convert to grayscale if it is the case
        map = map if len(map.shape) == 2 else np.mean(map, axis=2)
        map = map if mapdata['negate'] > 0 else 255 - map

        self.map = map   # save this for later
    
    #TODO: from heere
    @property
    def width(self):
        return self.map.shape[1]

    @property
    def height(self):
        return self.map.shape[0]

    @property
    def color_image(self):
        return cv2.cvtColor(255 - self.map, cv2.COLOR_GRAY2RGB)

if __name__ == '__main__':
    import cv2

    print os.path.abspath(os.path.curdir)

    mapdir = os.path.dirname(os.path.abspath(__file__))
    mapfile = os.path.normpath(os.path.normpath(mapdir+'/../../maps/map.yaml'))
    occupancy_grid = OccupancyGrid(mapfile)
    occ_map = occupancy_grid.map
    cv2.imshow('Occupancy Grid', occ_map)
    cv2.waitKey(0)
