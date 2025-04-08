import cv2
import numpy as np
import pandas as pd
from typing import List, Tuple

class Video:
    def __init__(self, video_path: str, width: int, height: int, fps: int=20):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.outpath = video_path
        self.width = width
        self.height = height
        self.transform_coordinates = None
        self.writer = cv2.VideoWriter(self.outpath, fourcc, fps, (width, height))

    def get_empty_frame(self) -> np.ndarray:
        return np.ones((self.height, self.width, 3), dtype=np.uint8) * 255

    def add_frame(self, img: cv2.UMat, flip=True):
        if flip:
            img = cv2.flip(img, 1)
        self.writer.write(img)


    def set_transform_function(self, x_min, x_max, y_min, y_max):
        def transform_coordinates(x, y):
            px = int(((x - x_min) / (x_max - x_min)) * self.width)
            py = int(((y - y_min) / (y_max - y_min)) * self.height)
            return px, py
        self.transform_coordinates =  transform_coordinates

    def add_polygons(self, img: np.ndarray, polygons: List[pd.DataFrame], colors, alphas=None):
        if len(polygons) != len(colors):
            raise ValueError("Number of polygons and colors must be the same")

        if alphas is None:
            alphas = [1.0] * len(polygons)
        elif len(alphas) != len(polygons):
            raise ValueError("Number of polygons and alphas must be the same")


        if self.transform_coordinates is None:
            offset = 0.5
            x_min = pd.concat(polygons).x.min() - offset
            x_max = pd.concat(polygons).x.max() + offset
            y_min = pd.concat(polygons).y.min() - offset
            y_max = pd.concat(polygons).y.max() + offset
            self.set_transform_function(x_min, x_max, y_min, y_max)

        overlay = img.copy()

        for i, poly in enumerate(polygons):
            points = np.array([self.transform_coordinates(x, y) for x, y in zip(poly["x"], poly["y"])], np.int32)
            overlay = np.zeros_like(img, dtype=np.uint8)
            mask = np.zeros_like(img, dtype=np.uint8)
            color_bgr = tuple(int(c) for c in colors[i])
            cv2.fillPoly(overlay, [points], color_bgr)
            cv2.fillPoly(mask, [points], (255, 255, 255))
            alpha = alphas[i]
            img[mask > 0] = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)[mask > 0]
            cv2.polylines(img, [points], isClosed=True, color=(0, 0, 0), thickness=2)

        return img
    

    def add_point(self,  img: np.ndarray, x: float, y: float, color=(0, 0, 0), radius=3):
        if self.transform_coordinates is None:
            raise ValueError("Please set the transform function before adding points")
        
        px, py = self.transform_coordinates(x, y)
        cv2.circle(img, (px, py), radius, color, -1) 
        return img
    
    def add_line(self, img: np.ndarray, x: List[float], y: List[float], color: Tuple[int, int, int], thickness=2):
        """
        Draws a line on the image connecting the given x and y coordinates.

        Args:
            img (np.ndarray): The image to draw on.
            x (List[float]): List of x coordinates.
            y (List[float]): List of y coordinates.
            color (Tuple[int, int, int]): BGR color of the line.
            thickness (int): Line thickness.
        """
        if len(x) != len(y):
            raise ValueError("x and y must have the same length")

        points = [self.transform_coordinates(x_, y_) for x_, y_ in zip(x, y)]
        for i in range(len(points) - 1):
            cv2.line(img, points[i], points[i + 1], color, thickness)

        return img

    def add_points(self, img: np.ndarray, x: List[float], y: List[float], color=(0, 0, 0), radius=3, alpha=1):
        if len(x) == 0  or len(y) == 0:
            return img
        
        if self.transform_coordinates is None:
            raise ValueError("Please set the transform function before adding points")
        
        if len(x) != len(y):
            raise ValueError("x and y must be of the same length")
    
        overlay = np.zeros_like(img, dtype=np.uint8)
        b, g, r = color
        circle_color = (b, g, r, int(alpha * 255))
        for x_, y_ in zip(x, y):
            px, py = self.transform_coordinates(x_, y_)
            cv2.circle(overlay, (px, py), radius, circle_color, -1) 
        
        mask = np.zeros_like(img, dtype=np.uint8)
        for x_, y_ in zip(x, y):
            px, py = self.transform_coordinates(x_, y_)
            cv2.circle(mask, (px, py), radius, (255, 255, 255), -1)

        img[mask > 0] = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)[mask > 0]

        return img

    def finish(self):
        print(f"Writing video to {self.outpath}")
        self.writer.release()
    
    def __del__(self):
        self.finish()


   


def filter_msgs_to_fps(df: pd.DataFrame, fps: int, time_col="time"):
    """
    Filters messages to match the desired FPS.

    Args:
        data (pd.DataFrame): The DataFrame containing the messages.
        fps (int): The desired frames per second.
        time_col (str): The name of the column containing the timestamps.

    Returns:
        pd.DataFrame: The filtered DataFrame.
    """
    interval = 1 / fps
    selected_indices = np.searchsorted(df[time_col], np.arange(df[time_col].iloc[0], df[time_col].iloc[-1], interval))
    return df.iloc[selected_indices]