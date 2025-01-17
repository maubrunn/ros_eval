import matplotlib.pyplot as plt
import pandas as pd
from plot_helpers.colors import get_colors, get_style
from typing import List

colors = get_colors()
plt.rcParams.update(get_style())

LINEWIDTH = 3


def plot_data(
        data: List[pd.DataFrame], 
        path: str, 
        title: str, 
        x_label: str,
        y_label: str, 
        col_bag: str,
        labels: List[str],
        x_col: str = 'time',
        second_linestyle: str = 'solid',
        pad = False
        ) -> None:
    figure= plt.figure( figsize=[10, 10])
    plt1 = figure.add_subplot(1, 1, 1)
    plt1.margins(x=0, y=0)

    for i in range(len(data)):
        color = colors[i] if i != 1 or second_linestyle == 'solid' else 'black'
        linestyle = 'solid' if i != 1 or second_linestyle == 'solid' else second_linestyle
        x_data = data[i][x_col] if x_col != "pd_indices" else data[i].index
        plt1.plot(x_data, data[i][col_bag], label=labels[i], color=color, linewidth=LINEWIDTH, linestyle=linestyle)
  
    # plt1.title.set_text(title)
    plt1.set_xlabel(x_label)
    plt1.set_ylabel(y_label)
    plt1.set_title(title, pad=20)
    max_y = max([d[col_bag].max() for d in data])
    if pad:
        plt1.set_xmargin(0.2)
        plt1.set_ymargin(0.3)
    else:
        plt1.set_ylim(top=max_y + 0.3 * max_y)
    plt1.grid()
    plt1.legend(loc='upper right')
    # plt1.set_xticklabels([])
    plt.savefig(path, bbox_inches='tight', pad_inches=0.1)