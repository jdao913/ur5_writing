import matplotlib.pyplot as plt
import csv
import numpy as np

# with open('your_filename.csv') as csvfile:

def get_x_y(csv_file):
    x = []
    y = []
    with open(csv_file) as csvfile:
        plots = csv.reader(csvfile,delimiter=',')
        for row in plots:
            x.append(float(row[0]))
            y.append(float(row[1]))

    return x,y

ax,ay = get_x_y('a_cap.csv')
bx,by = get_x_y('b_cap.csv')
cx,cy = get_x_y('c_cap.csv')
dx,dy = get_x_y('d_cap.csv')
ex,ey = get_x_y('e_cap.csv')
fx,fy = get_x_y('f_cap.csv')

gx,gy = get_x_y('g_cap.csv')
hx,hy = get_x_y('h_cap.csv')
ix,iy = get_x_y('i_cap.csv')
jx,jy = get_x_y('j_cap.csv')
kx,ky = get_x_y('k_cap.csv')
lx,ly = get_x_y('l_cap.csv')

mx,my = get_x_y('m_cap.csv')
nx,ny = get_x_y('n_cap.csv')
ox,oy = get_x_y('o_cap.csv')
px,py = get_x_y('p_cap.csv')
qx,qy = get_x_y('q_cap.csv')
rx,ry = get_x_y('r_cap.csv')

sx,sy = get_x_y('s_cap.csv')
tx,ty = get_x_y('t_cap.csv')
ux,uy = get_x_y('u_cap.csv')
vx,vy = get_x_y('v_cap.csv')
wx,wy = get_x_y('w_cap.csv')
xx,xy = get_x_y('x_cap.csv')

yx,yy = get_x_y('y_cap.csv')
zx,zy = get_x_y('z_cap.csv')


fig, axis = plt.subplots(4, 6)
axis[0,0].plot(ax,ay)
axis[0,1].plot(bx,by)
axis[0,2].plot(cx,cy)
axis[0,3].plot(dx,dy)
axis[0,4].plot(ex,ey)
axis[0,5].plot(fx,fy)

axis[1,0].plot(gx,gy)
axis[1,1].plot(hx,hy)
axis[1,2].plot(ix,iy)
axis[1,3].plot(jx,jy)
axis[1,4].plot(kx,ky)
axis[1,5].plot(lx,ly)

axis[2,0].plot(mx,my)
axis[2,1].plot(nx,ny)
axis[2,2].plot(ox,oy)
axis[2,3].plot(px,py)
axis[2,4].plot(qx,qy)
axis[2,5].plot(rx,ry)

axis[3,0].plot(sx,sy)
axis[3,1].plot(tx,ty)
axis[3,2].plot(ux,uy)
axis[3,3].plot(vx,vy)
axis[3,4].plot(wx,wy)
axis[3,5].plot(xx,xy)

for i in range(4):
    for j in range(6):
        axis[i,j].set_yticklabels([])
        axis[i,j].set_xticklabels([])

plt.show()
