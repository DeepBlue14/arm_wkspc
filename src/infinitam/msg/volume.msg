float32 voxelSize # size of voxels in volume (in meters)

int32 nPoints     # number of xyz tuples points array
float32[] points  # points data, size 3*nPoints
float32[] normals # normals data, size 3*nPoints

int32 trackCount  # a counter that increments every time the tracker resets, starting at 0
float32[] pose    # sensor pose (4x4 matrix) relative to its pose when trackCount last incremented
