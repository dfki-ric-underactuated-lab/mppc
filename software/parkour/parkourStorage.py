import numpy as np

def choose_parkour(selection):
    if selection == 1:
        Parkour = {"start_x": 0.0,
                    "goal_x": 7.2,
                    "position": np.array([1.3, 1.9, 2.8, 4.8, 6.5]),
                    "height":   np.array([0.2, 0.2, 0.2, 0.2, 0.3]),
                    "width":    np.array([0.2, 0.2, 0.1, 0.2, 0.2]),
                    "vicon": np.array([]),
                    "margin_obstacle_horizontal": 0.03,
                    "margin_obstacle_vertical": 0.05,
                    "margin_knee": 0.03,
                    "res_position": np.array([1.6, 4.5, 5.1]),  # np.array([1.3, 2.1, 2.6, 4.5]),
                    "res_width":    np.array([0.4, 0.4, 0.4])  # np.array([0.5, 0.2, 0.4, 0.4])
                    }
        
    elif selection == 2:
        Parkour = {"start_x": 0.0,
                   "goal_x": 7.2,
                   "position": np.array([1.3, 3.3, 4.8, 5.3, 6.6]),
                   "height":   np.array([0.1, 0.2, 0.2, 0.2, 0.1]),
                   "width":    np.array([0.2, 0.1, 0.2, 0.1, 0.3]),
                   "vicon": np.array([]),
                   "margin_obstacle_horizontal": 0.03,
                   "margin_obstacle_vertical": 0.05,
                   "margin_knee": 0.03,
                   "res_position": np.array([1.0, 1.6, 3.3, 5.3]),
                   "res_width":    np.array([0.4, 0.4, 0.4, 0.4])
                   }
    
    return Parkour