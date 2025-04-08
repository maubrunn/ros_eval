import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Polygon
from matplotlib_scalebar.scalebar import ScaleBar
import pandas as pd
import numpy as np
from plot_helpers.colors import get_colors, get_style, get_config_by_key
from typing import List, Tuple


LINEWIDTH = 4
MARKERSIZE = 6
TEXTSIZE = 40

colors = get_colors()
plt.rcParams.update(get_style(TEXTSIZE))

def add_polygon(ax, polygons, scale=False, rotate=False):
    offset = 0.5
    x_min = pd.concat(polygons).x.min() - offset
    x_max = pd.concat(polygons).x.max() + offset
    y_min = pd.concat(polygons).y.min() -  offset
    y_max = pd.concat(polygons).y.max() + offset

    if rotate:
        x_min, x_max, y_min, y_max = y_min, y_max, x_min, x_max
    
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    grid_n = 7
    ax.grid(color='gray', linestyle='-', linewidth=0.5, alpha=0.2)
    # ax.set_aspect('equal', adjustable='box')
    xticks = [x_min + j*(x_max - x_min) /grid_n for j in range(grid_n)]
    yticks = [y_min + j*(y_max - y_min) /grid_n for j in range(grid_n)]
    ax.set_xticks(xticks)
    ax.set_xticklabels([''] * len(xticks))
    ax.set_yticks(yticks)
    ax.set_yticklabels([''] * len(yticks))
    ax.tick_params(axis='both', which='both', length=0)
    ax.add_patch(plt.Rectangle((x_min, y_min), x_max-x_min, y_max-y_min, facecolor='lightgrey', zorder=0, alpha=0.25))
    if scale:
        scalebar = ScaleBar(1, units='m', location='upper right',  border_pad=0.2)
        ax.add_artist(scalebar)

    for i in range(len(polygons)):
        c = "white" if i % 2 == 0 else "lightgrey"
        poly = pd.DataFrame()
        poly["x"] = polygons[i].x.copy()
        poly["y"] = polygons[i].y.copy()
        if rotate:
            poly["x"], poly["y"] = poly["y"], poly["x"]
        p = Polygon(poly, closed=True, facecolor=c, edgecolor='black', zorder=1)
        ax.add_patch(p)

    return ax

def setup_grid(num, polygons=[], rotate=True, width=10, height=10, config=[], vert=False) -> Tuple[plt.figure, List[plt.Axes]]:
    fig, gs = None, None
    def setup_ratios(key):
        ratios = []
        for i in range(num):
            ratios.append(get_config_by_key(i, key, config))

        return ratios


    if vert:
        fig = plt.figure(figsize=(width, num * height))
        gs = gridspec.GridSpec(num, 1, figure=fig, height_ratios=setup_ratios("height_ratio"))
    else:
        fig = plt.figure(figsize=(width * num, height))
        gs = gridspec.GridSpec(1, num, figure=fig, width_ratios=setup_ratios("width_ratio"))
    
    axes = []

    for i in range(num):
        if vert:
            ax = fig.add_subplot(gs[i, 0])
        else:
            ax = fig.add_subplot(gs[0, i])

        ax.set_xlabel(get_config_by_key(i, "x_label", config))
        ax.set_ylabel(get_config_by_key(i, "y_label", config))
        ax.text(0.04, 0.96, get_config_by_key(i, "text", config), fontsize=TEXTSIZE, transform=ax.transAxes,  verticalalignment='top', horizontalalignment='left', bbox=dict(facecolor='white', alpha=0.75, edgecolor='black', boxstyle='round,pad=0.25'))
        if len(polygons) > 0 and get_config_by_key(i, "polygon", config):
            ax = add_polygon(ax, polygons, get_config_by_key(i, "scale", config), rotate)
        axes.append(ax)
    return fig, axes


def finish_fig(figure, path, handles=[], cols=1, config={}, legend_loc="lower right", bbox_to_anchor=None) -> None:
    for i, ax in enumerate(figure.get_axes()):
        h, _ = ax.get_legend_handles_labels()
        h += handles
        if get_config_by_key(i, "legend", config):
            if bbox_to_anchor is None:
                legend = ax.legend(handles=h, loc=legend_loc, ncols=cols)
            else:
                legend = ax.legend(handles=h, bbox_to_anchor=bbox_to_anchor , ncols=cols)

            legend.get_frame().set_alpha(0.95)  # Set alpha to 0.9 for less transparency

        if not get_config_by_key(i, "polygon", config):
            ax.set_xmargin(0)
            x_min, x_max = ax.get_xlim()
            y_min, y_max = ax.get_ylim()
            xticks = np.linspace(x_min, x_max, 10)
            yticks = np.linspace(y_min, y_max, 5)
            xticks = [round(x, 0) for x in xticks]
            yticks = [round(y, 0) for y in yticks]
            ax.set_xticks(xticks)
            ax.set_yticks(yticks)
            ax.grid(color='gray', linestyle='-', linewidth=0.5, alpha=0.2)  
    plt.subplots_adjust(wspace=0.0, hspace=0.05)
    figure.savefig(path , dpi=300, bbox_inches='tight', pad_inches=0)
    
def add_legend_entry(label: str, marker: str = 'o', color="gray") -> plt.Line2D:
    if marker != "-":
        return plt.Line2D([0], [0], marker=marker, color=color, linestyle='', markersize=MARKERSIZE * 2, label=label)
    else:
        return plt.Line2D([0], [0], color=color,  linewidth=LINEWIDTH, label=label)

def plot_arrow(ax: plt.Axes, x: float, y: float, yaw: float, color: str, alpha: float = 1.0, size_multiplier=1.0) -> None:
    ax.arrow(
        x=x,
        y=y,
        dx=np.cos(yaw) * (0.5 + (size_multiplier - 1) * 0.25),
        dy=np.sin(yaw) * (0.5 + (size_multiplier - 1) * 0.25),
        color=color,
        alpha=alpha,
        width=0.04 + (size_multiplier - 1) * 0.1,
        length_includes_head=True,
        head_width=0.2 * size_multiplier,
        head_length=0.2 * size_multiplier,
    )
    return ax

def plot_points(
        ax: plt.Axes,
        data: pd.DataFrame,
        x_col: str = 'x',
        y_col: str = 'y',
        style: str = 'o',
        alpha: float = 0.6,
        color: str = 'black',
):
    for i in range(len(data[x_col])):
        x = data[x_col].iloc[i]
        y = data[y_col].iloc[i]
        ax.plot(x, y, style, alpha=alpha, color=color, markersize=MARKERSIZE, label='_nolegend_')
    return ax

def plot_line(
        ax: plt.Axes,
        data: pd.DataFrame,
        color,
        label,
        rotate=False,
        fill_around=False,
        var=False,
        ):
    col_x, col_y, col_x_low, col_x_high, col_y_low, col_y_high = "x", "y", "x_low", "x_up", "y_low", "y_up"
    if rotate:
        col_x, col_y = col_y, col_x
        col_x_low, col_x_high, col_y_low, col_y_high = col_y_low, col_y_high, col_x_low, col_x_high
    
    ax.plot(data[col_x], data[col_y], '-', alpha=0.9, color=color, markersize=MARKERSIZE, label=label)
    
    if fill_around:
        x_lower = data[col_x_low]
        x_upper = data[col_x_high]
        y_lower = data[col_y_low]
        y_upper = data[col_y_high]
        
        ax.fill(np.concatenate([x_lower, x_upper[::-1]]),  # X values: lower to upper (reversed)
            np.concatenate([y_lower, y_upper[::-1]]),  # Y values: lower to upper (reversed)
            color=color, alpha=0.3)
    if var:
        ax.fill_between(data[col_x], data[col_y] - np.sqrt(data["var"]), data[col_y] + np.sqrt(data["var"]), color=color, alpha=0.3)
    return ax

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
    # plt1.set_xticklabels([])
    plt.savefig(path, bbox_inches='tight', pad_inches=0.1)

def plot_bars(
        data: List[pd.DataFrame], 
        path: str,
        title:str,
        x_label:str,
        y_label:str, 
        Labels: List[str]
        ) -> None:

    figure= plt.figure( figsize=[15, 3 * len(data)])
    plt1 = figure.add_subplot(1, 1, 1)
    color_index = 0
    used_colors = {}
    # plt1.margins(x=0, y=0)
    for i in range(len(data)):
        left_offset = 0
        for column, value in data[i].items():
            color = used_colors.get(column, None)
            if color is None:
                color = colors[color_index % len(colors)]
                color_index += 1
                used_colors[column] = color
            plt1.barh([Labels[i]], [value], left=left_offset, color=color, label=column)
            left_offset += value
    
    
    plt1.set_xlabel(xlabel=x_label)
    plt1.set_ylabel(ylabel=y_label)
    # plt1.grid()
    plt1.legend(loc="upper center", bbox_to_anchor=(0.5, 1.6), ncol=3)
    # plt1.set_title(title)
    plt1.set_xlim(left=0, right=left_offset + 0.6* left_offset)
    # Save and show the plot
    plt.tight_layout()
    plt.savefig(path, bbox_inches='tight', pad_inches=0.1)