import matplotlib as ml, matplotlib.pyplot as plt
import parameters
import numpy as np
import simulator

crew_color = 'k'
plt.ion()

def draw_path(robot_path, mission, crew):
    plt.figure()

    plt.plot(robot_path[:,0],robot_path[:,1])
    plt.axis('scaled')


    for cx,cy,ct in crew:
        plt.gca().add_patch(
            plt.Circle(
                (cx,cy),
                radius=parameters.crew_radius,
                ec=crew_color,fill=False
            ))
        plt.gca().add_line(
            plt.Line2D(
                (cx,cx+np.cos(ct)*parameters.crew_radius), 
                (cy,cy+np.sin(ct)*parameters.crew_radius),
                color=crew_color
            ))

    last_x = -100
    last_y = -100
    for idx in range(robot_path.shape[0]):
        x,y = robot_path[idx,0], robot_path[idx,1]
        if (np.linalg.norm(np.array([x-last_x,y-last_y])) >= 2* simulator.spheres_width) or (idx == robot_path.shape[0]-1):
            plt.gca().add_patch(plt.Rectangle(
                (robot_path[idx,0]-simulator.spheres_width/2,y-simulator.spheres_height/2), 
                width=simulator.spheres_width,
                height=simulator.spheres_height,
                angle=robot_path[idx,2],
                ec='b',
                fill=False)) 
            last_x = x
            last_y = y    


    for wp in mission:
        plt.gca().add_patch(
            plt.Circle(
                wp,
                radius=simulator.spheres_width*1.5,
                ec='m',fill=False, lw=3
            ))

    plt.xlim((0,parameters.module_size[0]))
    plt.ylim((0,parameters.module_size[1]))
    plt.tight_layout()
    # plt.show()


