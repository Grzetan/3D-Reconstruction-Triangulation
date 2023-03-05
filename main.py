import c3d

with open('Dron T02.c3d', 'rb') as handle:
    reader = c3d.Reader(handle)
    for i, data in enumerate(reader.read_frames()):
        print(len(data[2][3]))
        input()
        # print('Frame {}: {}'.format(i, points.round(2)))


# Data[0] = frame_id
# Data[1] = 4x5 array with last number equal to 0
# Data[2] = 44x10 array with numbers??