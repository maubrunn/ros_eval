#!/usr/bin/env python3

def get_colors():
    return [
        "#B7352D", # eth red
        "#215CAF", # eth blue
        "#627313", # eth green
        "#007894", #eth petrol
        "#8E6713", # eth bronze
        "#A7117A", # eth purple
        "#6F6F6F", # eth grey
    ]

def transform_to_cv2(hex_color):
    """
    Converts a hexadecimal color string to OpenCV BGR color format.

    Args:
        hex_color (str): Hexadecimal color string (e.g., "#RRGGBB" or "RRGGBB").

    Returns:
        tuple: BGR color tuple for OpenCV (Blue, Green, Red).
    """
    hex_color = hex_color.lstrip('#')
    r = int(hex_color[0:2], 16)
    g = int(hex_color[2:4], 16)
    b = int(hex_color[4:6], 16)
    return (b, g, r)

def get_style(font_size=30):
    return {
        'font.size': font_size,
        "font.family": "DejaVu Serif",
        "font.serif": "Times New Roman"
        }

def get_config_by_key(i, key, config):
        default = {
            "x_label": "",
            "y_label": "",
            "polygon": False,
            "legend": True,
            "height_ratio": 1,
            "width_ratio": 1,
            "text": "",
            "scale": False,
        }

        if i < len(config) and key in config[i]:
            return config[i][key]
        elif key in default:
            return default[key]

        raise ValueError(f"Unknown key {key}")   