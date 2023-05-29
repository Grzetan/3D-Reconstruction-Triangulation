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
bpy.ops.object.mode_set(mode='OBJECT')
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Add new path
cu = bpy.data.curves.new("Path", "CURVE")
cu.dimensions = '3D'
ob = bpy.data.objects.new("Path", cu)
polyline = cu.splines.new('BEZIER')  # 'POLY''BEZIER''BSPLINE''CARDINAL''NURBS'
bpy.context.collection.objects.link(ob)

bpy.context.view_layer.objects.active = ob

bpy.ops.object.mode_set(mode='EDIT')

for i, p in enumerate(points):
    
    bpy.ops.curve.vertex_add(location=p)
    

bpy.ops.object.mode_set(mode='OBJECT')