import matplotlib as ml, matplotlib.pyplot as plt
import parameters
import numpy as np

crew_color = 'k'
plt.ion()

# np array columns: x,y,theta
def draw_path(robot_path, mission, crew):
    plt.figure()

    plt.plot(robot_path[:,0],robot_path[:,1])
    plt.axis('scaled')


    for cx,cy,ct in crew: #crew x, crew y, crew theta
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
        if (np.linalg.norm(np.array([x-last_x,y-last_y])) >= 2* parameters.robot_length) or (idx == robot_path.shape[0]-1):
            plt.gca().add_patch(plt.Rectangle(
                robot_path[idx,parameters.XY_POS] - parameters.robot_length/2,
                width=parameters.robot_length,
                height=parameters.robot_length,
                angle=robot_path[idx,2],
                ec='b',
                fill=False)) 
            last_x = x
            last_y = y    


    for wp in mission:
        plt.gca().add_patch(
            plt.Circle(
                wp,
                radius=parameters.robot_length*1.5,
                ec='m',fill=False, lw=3
            ))

    plt.xlim((0,parameters.module_size[0]))
    plt.ylim((0,parameters.module_size[1]))
    plt.tight_layout()
    # plt.show()



