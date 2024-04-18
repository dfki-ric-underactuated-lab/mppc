import numpy as np
import matplotlib.pyplot as plt
import copy
import matplotlib
matplotlib.use("TkAgg")

class Plotter:
    def __init__(self, Hopper):
        self.Hopper = Hopper
        self.fontsize = 37
        self.my_dpi = 80
        self.x_scope_add= 0.08
        self.y_scope_min = -0.02
        self.y_scope_max_add = 0.2
    

    def obstacle_course(self, parkour, ax=None, areas=None, obstacles=None, marker=True):
        self.Parkour = parkour
        show = False
        if areas is None:
            areas = []
            for i in range(len(self.Parkour["res_position"])):
                if self.Parkour["goal_x"] > self.Parkour["res_position"][i]:
                    areas.append(i)
        if obstacles is None:
            obstacles = []
            for i in range(len(self.Parkour["position"])):
                if self.Parkour["goal_x"] > self.Parkour["position"][i]:
                    obstacles.append(i)
        if ax is None:
            show = True
            fig, ax = plt.subplots(figsize=(1800 / self.my_dpi, 900 / self.my_dpi), dpi=self.my_dpi)

        ax.set_xlabel("x [m]", fontsize=self.fontsize)
        ax.set_ylabel("z [m]", fontsize=self.fontsize)

        plot_stepsize = 0.0001
        overshoot = 1
        parkour_plot_x = np.arange(self.Parkour["start_x"] - overshoot, 
                                   self.Parkour["goal_x"] + overshoot + plot_stepsize, plot_stepsize)
        parkour_plot_z = np.zeros(len(parkour_plot_x))

        # PARKOUR
        for i in obstacles:
            start_point = (overshoot - self.Parkour["start_x"] + self.Parkour["position"][i]
                        - self.Parkour["width"][i] / 2) / plot_stepsize
            end_point = (overshoot - self.Parkour["start_x"] + self.Parkour["position"][i] 
                        + self.Parkour["width"][i] / 2) / plot_stepsize
            parkour_plot_z[int(start_point):int(end_point)] = self.Parkour["height"][i]
            parkour_plot_z[[int(start_point),
                            int(end_point)]] = (self.Parkour["height"][i] 
                                                + self.Parkour["margin_obstacle_vertical"])
        ax.plot(parkour_plot_x, parkour_plot_z, 'g', linewidth=6, label='Contact Allowed')

        # HURDLE MARGINS
        for i in obstacles:
            front_a = (overshoot - self.Parkour["start_x"] + self.Parkour["position"][i] - self.Parkour["width"][i] / 2 
                       - self.Parkour["margin_obstacle_horizontal"]) / plot_stepsize
            front_b = (overshoot - self.Parkour["start_x"] + self.Parkour["position"][i] - self.Parkour["width"][i] / 2 
                       + self.Parkour["margin_obstacle_horizontal"]) / plot_stepsize
            back_a = (overshoot - self.Parkour["start_x"] + self.Parkour["position"][i] + self.Parkour["width"][i] / 2 
                      - self.Parkour["margin_obstacle_horizontal"]) / plot_stepsize
            back_b = (overshoot - self.Parkour["start_x"] + self.Parkour["position"][i] + self.Parkour["width"][i] / 2 
                      + self.Parkour["margin_obstacle_horizontal"]) / plot_stepsize
            ax.plot(parkour_plot_x[int(front_a):int(front_b)], 
                    parkour_plot_z[int(front_a):int(front_b)], 'r', linewidth=6)
            ax.plot(parkour_plot_x[int(back_a):int(back_b)], 
                    parkour_plot_z[int(back_a):int(back_b)],
                    'r', linewidth=6, label='Contact Prohibited' if i == 0 else '')

        # RESTRICTED AREAS
        for i in areas:
            front = (overshoot + self.Parkour["res_position"][i]
                     - self.Parkour["res_width"][i] / 2) / plot_stepsize
            back = (overshoot + self.Parkour["res_position"][i] 
                    + self.Parkour["res_width"][i] / 2) / plot_stepsize
            ax.plot(parkour_plot_x[int(front):int(back)], parkour_plot_z[int(front):int(back)], 
                    'r', linewidth=6)

        plt.grid(True)
        plt.rcParams.update({'font.size': self.fontsize-7})
        if show:
            plt.title("2D Parkour")
            ax.plot(self.Parkour["start_x"], 0, '^b', markersize=20, label='Start')
            ax.plot(self.Parkour["goal_x"], 0, 'vb', markersize=20, label='Goal')
            plt.xticks(fontsize=self.fontsize)
            plt.yticks(fontsize=self.fontsize)
            plt.xlim([-0.08, self.Parkour["goal_x"] + 0.08])
            plt.ylim([-0.02, max(parkour_plot_z) + 0.25])
            ax.legend(loc='upper left', fontsize=self.fontsize-9)
            plt.show()

        return ax


    def jump_history(self, parkour, res_dict, contact_x, contact_z, show=False):
        fig, ax = plt.subplots(figsize=(1700 / self.my_dpi, 800 / self.my_dpi), dpi=self.my_dpi)
        ax = self.obstacle_course(parkour, ax)
        ax.set_xlim([self.Parkour["start_x"]-self.x_scope_add, self.Parkour["goal_x"]+self.x_scope_add])
        y_max_obstacle = (max(np.append(self.Parkour["height"], 0)) 
                          + self.Parkour["margin_obstacle_vertical"] + self.y_scope_max_add)
        ax.set_ylim([self.y_scope_min, y_max_obstacle + self.y_scope_max_add])
        ax.set_title('Executed Jumps')
        ax.set_xticks(np.append(self.Parkour["start_x"], np.append(np.round(self.Parkour["position"], 2),
                       self.Parkour["goal_x"])))
        plt.setp(ax.get_xticklabels(), rotation=40, horizontalalignment='right')
        plt.grid(True)
        plt.rcParams.update({'font.size': self.fontsize})
        if res_dict is None:
            return ax

        N = len(res_dict["t"])
        t = res_dict["t"]
        v = res_dict["v"]
        theta = res_dict["theta"]
        
        y_max_jump = max((contact_z[0:-1] + self.Hopper.leg.r_takeoff * np.sin(theta) 
                          + self.Hopper.dz_hip_foot + self.Hopper.dz_foot_knee 
                          + ((v * np.sin(theta)) ** 2 / (2 * self.Hopper.gravity))))
        ax.set_ylim([self.y_scope_min, max(y_max_obstacle, y_max_jump) + self.y_scope_max_add])

        # DESIRED FOOT TRAJECTORY
        for i in range(N):
            T = t[i] * np.linspace(0, 1, 40)
            X = (contact_x[i] + self.Hopper.leg.r_takeoff * np.cos(theta[i]) 
                 + self.Hopper.dx_hip_foot) + (v[i] * np.cos(theta[i]) * T)
            Z = (contact_z[i] + self.Hopper.leg.r_takeoff * np.sin(theta[i]) 
                 + self.Hopper.dz_hip_foot) + (v[i] * np.sin(theta[i]) * T 
                                               - 0.5 * self.Hopper.gravity * T**2)
            ax.plot(X, Z, "b-", linewidth=2, label='Desired Foot Trajectory' if i == N-1 else '')
            ax.plot(X + self.Hopper.dx_foot_knee, Z + self.Hopper.dz_foot_knee, 
                    "y-", linewidth=2, label='Knee Trajectory' if i == N-1 else '')

        # CONTACT
        ax.plot(contact_x[1:], contact_z[1:], "bo", markersize=12, label='Contact Points')
        
        plt.xlim([-0.08, max(self.Parkour["goal_x"], contact_x[-1]) + 0.08])
        handles, labels = ax.get_legend_handles_labels()
        plt.rcParams.update({'font.size': self.fontsize})

        if show:
            plt.show()


    def current_plan(self, parkour, res_dict, contact_x, contact_z, jump, EE_x=None, EE_z=None, save=False):
        N = len(res_dict["t"])
        t = res_dict["t"]
        v = res_dict["v"]
        theta = res_dict["theta"]
        obstacles = res_dict["obstacles"]
        areas = res_dict["areas"]

        fig, ax = plt.subplots(figsize=(1700 / self.my_dpi, 800 / self.my_dpi), dpi=self.my_dpi)
        ax = self.obstacle_course(parkour, ax, areas, obstacles, marker=False)
        
        # DESIRED FOOT TRAJECTORY
        for i in range(N):
            T = t[i] * np.linspace(0, 1, 1000)
            X = (contact_x[i] + self.Hopper.leg.r_takeoff * np.cos(theta[i]) 
                 + self.Hopper.dx_hip_foot) + (v[i] * np.cos(theta[i]) * T)
            Z = (contact_z[i] + self.Hopper.leg.r_takeoff * np.sin(theta[i]) 
                 + self.Hopper.dz_hip_foot) + (v[i] * np.sin(theta[i]) * T 
                                               - 0.5 * self.Hopper.gravity * T**2)
            ax.plot(X, Z, "b--", linewidth=5, label='foot' if i == N-1 else '')
            ax.plot(X + self.Hopper.dx_foot_knee, Z + self.Hopper.dz_foot_knee, 
                    "C1--", linewidth=5, label='knee' if i == N-1 else '')
        
        if EE_x is not None and EE_z is not None:
            ax.plot(EE_x, EE_z, 'm', linewidth=5, label='measured foot')

        # START
        ax.plot(contact_x[0], contact_z[0], '^b', markersize=15, label='Start')

        # CONTACT
        ax.plot(contact_x[1:-1], contact_z[1:-1], "bo", markersize=13, label='Contact Points')

        # END
        ax.plot(contact_x[-1], contact_z[-1], 'vb', markersize=15, label='End')
        
        # FIRURE SETTINGS
        ax.axis('equal')
        ax.set_aspect('equal', 'box')
        plt.grid(True)
        ax.set_xlim([min(contact_x[0]-self.x_scope_add, contact_x[-1]-2-self.x_scope_add), contact_x[-1]+self.x_scope_add])
        y_max_obstacle = (max(np.append(self.Parkour["height"][obstacles], 0)) 
                          + self.Parkour["margin_obstacle_vertical"]) if obstacles.any() else 0
        y_max_jump = max((contact_z[0:-1] + self.Hopper.leg.r_takeoff * np.sin(theta) 
                          + self.Hopper.dz_hip_foot + self.Hopper.dz_foot_knee 
                          + ((v * np.sin(theta)) ** 2 / (2 * self.Hopper.gravity))))
        ax.set_ylim([self.y_scope_min, 0.85])
        handles, labels = ax.get_legend_handles_labels()
        for l in range(len(labels)):
            if labels[l] == 'foot':
                break
        ax.legend(handles[l:l+2], labels[l:l+2], loc='upper left', fancybox=True, ncol=2, fontsize=self.fontsize)
        ax.set_xticks(np.round(contact_x, 2))
        plt.title("Planned Trajectory")
        if save:
            plt.savefig(f'utils/plots/easy_cplan_jump{jump}.png', dpi=300)
        plt.show()

    def real_foot(self, parkour, X, Z, ax=None, show=True):
        if ax is None:
            fig, ax = plt.subplots(figsize=(1800 / self.my_dpi, 900 / self.my_dpi), dpi=self.my_dpi)
            ax = self.obstacle_course(parkour, ax)
        ax.plot(0, 0, '^b', markersize=20, label='Start')
        ax.plot(7.2, 0, 'vb', markersize=20, label='Goal')
        ax.plot(X, Z, 'm', linewidth=5, label='Measured Foot Trajectory')
        if show:
            plt.xticks(fontsize=self.fontsize)
            plt.yticks(fontsize=self.fontsize)
            plt.xlim([-0.2, self.Parkour["goal_x"] + 0.2])
            plt.ylim([-0.02, max(Z) + 0.1])
            handles, labels = ax.get_legend_handles_labels()
            ax.legend([handles[0], handles[1], handles[4], handles[2], handles[3]], [labels[0], labels[1], labels[4], labels[2], labels[3]], loc='upper left', fancybox=True, ncol=2, fontsize=self.fontsize-0.5)
            plt.savefig(f'utils/plots/easy_foot_traj.png', dpi=300)
            plt.show()
        return ax