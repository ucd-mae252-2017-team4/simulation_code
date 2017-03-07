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

    for idx in range(0,robot_path.shape[0],10):
        plt.gca().add_patch(plt.Rectangle(
            (robot_path[idx,0]-simulator.spheres_width/2,robot_path[idx,1]-simulator.spheres_height/2), 
            width=simulator.spheres_width,
            height=simulator.spheres_height,
            angle=robot_path[idx,2],
            ec='b',
            fill=False))      


    for wp in mission:
        pass

    plt.xlim((0,parameters.module_size[0]))
    plt.ylim((0,parameters.module_size[1]))
    plt.tight_layout()
    # plt.show()


