import requests
import base64
import json
import time
import os
import pytest

SERVER_URL = "http://localhost:5000"

UPLOAD_FOLDER = "../uploaded_maps"

def get_latest_map_file(extension):
    files = [f for f in os.listdir(UPLOAD_FOLDER) if f.endswith(extension)]
    if not files:
        return None
    return max(files, key=lambda f: int(f.split("_")[-1].split(".")[0]))

def test_navigation_path():
    latest_map = get_latest_map_file(".png")
    assert latest_map is not None, "No map image file found in uploaded_maps/"
    
    r = requests.get(f"{SERVER_URL}/navigation/path")
    assert r.status_code == 200, f"Status: {r.status_code}, Body: {r.text}"
    data = r.json()
    assert "smooth_path" in data
    assert isinstance(data["smooth_path"], list)

def test_navigation_path_with_plot():
    latest_map = get_latest_map_file(".png")
    assert latest_map is not None, "No map image file found in uploaded_maps/"

    r = requests.get(f"{SERVER_URL}/navigation/path_with_plot")
    assert r.status_code == 200, f"Status: {r.status_code}, Body: {r.text}"
    data = r.json()
    assert "figure" in data
    assert data["figure_format"] == "png"


def test_robot_position_post_and_get():
    data = {"x": 1.5, "y": 2.5, "theta": 1.57}
    r = requests.post(f"{SERVER_URL}/robot/position", json=data)
    assert r.status_code == 200
    assert r.json()["message"] == "Robot position updated successfully"

    r = requests.get(f"{SERVER_URL}/robot/position")
    assert r.status_code == 200
    pos = r.json()
    assert pos["x"] == data["x"]
    assert pos["y"] == data["y"]
    assert pos["theta"] == data["theta"]


def test_get_latest_map_file():
    r = requests.get(f"{SERVER_URL}/map/latest")
    assert r.status_code == 200
    assert r.headers["Content-Type"] == "application/octet-stream"


