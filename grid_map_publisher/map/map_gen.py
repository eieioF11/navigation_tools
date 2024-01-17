import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image
import numpy as np
import colorsys

# map.yaml config
image = "field.pgm"
# resolution = 0.005
# origin =  [-2.96, -2.56, 0]
negate =  0
occupied_thresh = 0.65
free_thresh =  0.25

# field size
field_size = [12000, 6000]
field_origin = [0, 0]

tick = 100

# fence (pos.x, pos.y, size.x, size.y)[mm]
sobstacle = [
    # fence
    # x
    [0, 0,12000 ,50],
    [1925, 1925,12000-1925*2,25],
    [0, 6000-25,1925,25],
    [12000-1925, 6000-25,1925,25],
    # y
    [0, 0 ,50,6000-25],
    [12000-50, 0 ,50,6000-25],
    [1925,1925,25,6000-1925],
    [12000-1925,1925,25,6000-1925],
    ]

fig, ax = plt.subplots(figsize=(5.0, 5.0))

ax.tick_params(labelbottom=False,
                labelleft=False,
                labelright=False,
                labeltop=False)
ax.tick_params(bottom=False,
                left=False,
                right=False,
                top=False)

bounding_box_line = [
    [0,0],
    [field_size[0],0],
    [field_size[0], field_size[1]],
    [0, field_size[1]]
]

ax.set_xlim(0, field_size[0] + 2*tick)
ax.set_ylim(0, field_size[1] + 2*tick)

base = patches.Rectangle(
    (0, 0),
    field_size[0] + 2*tick, 
    field_size[1] + 2*tick,
    linewidth = 0,
    edgecolor = 'gray',
    facecolor = 'gray',
    fill=True)
ax.add_patch(base)

field_base = patches.Rectangle(
    (tick, tick),
    field_size[0], 
    field_size[1],
    linewidth = 0,
    edgecolor = 'white',
    facecolor = 'white',
    fill=True)
ax.add_patch(field_base)

for obj in sobstacle:
    ax.add_patch(patches.Rectangle(
        (tick+obj[0], tick+obj[1]),
        obj[2], obj[3],
        linewidth = 0,
        edgecolor = 'black',
        facecolor = 'black',
        fill=True
    ))

ax.set_aspect('equal')
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

map_dir = os.path.dirname(os.path.abspath(__file__))

png_name = image.split('.')[0] + ".png"

png_abs_path = os.path.join(map_dir, png_name)
plt.savefig(png_abs_path)


# convert png to pgm
im = np.array(Image.open(png_abs_path))

print(len(im))
print(len(im[0]))
print(im[0][0])

res = field_size[0] + 2*tick

print(len(im))
print((res/1000) / len(im)) # res

pgm_f = open(os.path.join(map_dir, image.split('.')[0]+".pgm"), 'w')

pgm_f.write("P2\n")
pgm_f.write(str(len(im)) + " " + str(len(im[0])) + "\n")
pgm_f.write("255\n")
for line in im:
    for pixel in line:
        hsv_p = colorsys.rgb_to_hsv(pixel[0]/255, pixel[1]/255, pixel[2]/255)
        # print()
        pgm_f.write(str(int(255*hsv_p[2])) + "\n")
pgm_f.write("\n")    
pgm_f.close()

yaml_f = open(os.path.join(map_dir, image.split('.')[0]+".yaml"), 'w')

yaml_f.write("image: " + image.split('.')[0] + ".pgm\n")
yaml_f.write("resolution: {:.4f}\n".format((res/1000) / len(im)))
yaml_f.write("origin: [{:.3f}, {:.3f}, 0]\n".format(-(tick + field_origin[0])/1000, -(tick + field_origin[1])/1000))
yaml_f.write("negate: 0\n")
yaml_f.write("occupied_thresh: 0.65\n")
yaml_f.write("free_thresh: 0.25\n")
yaml_f.close()

plt.show()