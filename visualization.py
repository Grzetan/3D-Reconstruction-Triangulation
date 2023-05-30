import bpy

path = "/home/grzetan/PJA/Quaternions/<2k_frames/dron1_scaled_label_<2k.ply"
points = []
after_header = False
for line in open(path, 'r'):
    if line.startswith("end_header"):
        after_header = True
        continue
    if not after_header:
        continue
    
    c = line.split(' ')
    points.append([float(c[0]), float(c[1]), float(c[2])])

# Delete all objects
#bpy.ops.object.mode_set(mode='OBJECT')
#bpy.ops.object.select_all(action='SELECT')
#bpy.ops.object.delete(use_global=False)

crv = bpy.data.curves.new('crv', 'CURVE')
crv.dimensions = '3D'

# make a new spline in that curve
spline = crv.splines.new(type='POLY')

# a spline point for each point
spline.points.add(len(points)-1) # theres already one point by default

# assign the point coordinates to the spline points
for p, new_co in zip(spline.points, points):
    p.co = (new_co + [1.0]) # (add nurbs weight)

# make a new object with the curve
obj = bpy.data.objects.new('object_name', crv)
bpy.context.collection.objects.link(obj)