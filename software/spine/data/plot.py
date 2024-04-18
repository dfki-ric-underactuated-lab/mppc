"""
Record Plot
===========
"""

import numpy as np

from mpl_plotter import figure
from mpl_plotter.color.schemes import colorscheme_one
from mpl_plotter.two_d import line, panes

    
def plot_record(record,
                show=True,
                save=False,
                filename=None):
    """
    Matrix plot containing the desired and measured state variables of each actuator
    in your system. 
    
    The plot will be shaped as an ``m`` by 3 matrix, where ``m`` is the number
    of actuators. That is, it will have three columns, for position, velocity
    and force, and an equal number of rows to the number of actuators in the system.
    
    **Arguments**
    
    ``record`` [``software.spine.record.record`` instance]
      experiment ``record`` object, containing the time vector of the
      experiment as well as the desired and measured state vectors
      of all motors in the experiment.

    ``show`` [bool]
      whether to call ``plt.plot()`` at the end of this function. You may not
      want this to happen if you intend to further customize the plot after
      calling this function

    ``save`` [bool]
      whether to save the plot

    ``filename`` [str]
      name of the file to which the plot will be saved (if ``save`` is ``True``),
      **without** extension: the plot will be saved **both** in high quality
      PNG (200 dpi) and EPS formats, as such::

         <filename>.png
         <filename>.eps

    """

    # number of actuators
    m = len(record.motor_names)

    # slice
    s = slice(1, -1)
    
    x        = record.t
    y        = []
    titles   = []
    y_labels = []
    
    for i in range(m):
        motor_id    = record.motor_names[i]
        pd, vd, td  = record.matrix_des(i)
        pm, vm, tm  = record.matrix_msr(i)
        titles     += [f'{motor_id} :: {p}' for p in ['Pos', 'Vel', 'Tor']]
        y_labels   += ['rad', 'rad/s', 'Nm']
        y          += [[pd[s], pm[s]],
                       [vd[s], vm[s]],
                       [td[s], tm[s]]]
        
    colorscheme    = colorscheme_one()
    colorscheme[0] = '#d95959'
        
    panes(x[s], y,

          tick_label_decimals_x=1,
          tick_label_decimals_y=3,
          
          titles=titles,
          label_x='t [s]',
          labels_y=y_labels,
          label_rotation_y=90,
          label_pad_y=10,

          # curves
          zorders=[2, 3],
          line_widths=[3, 1.5],
          colors=['black', '#d95959'],

          # legend
          plot_labels=['$des$', '$msr$'],
          legend_loc=[0.91, 0.45],
          
          # layout
          fig=figure((12, 3.5*m)),
          rows=m,
          top=0.96,
          bottom=0.05,
          left=0.075,
          right=0.885,
          hspace=0.35,
          wspace=0.6)

    from matplotlib import pyplot as plt

    if save:

        assert filename is not None, 'you must provide a filename *without extension* to save the record plot of your experiment'
        
        plt.savefig(filename + '.eps', format='eps')
        plt.savefig(filename, dpi=200)

    if show:
        plt.show()

    plt.close()
