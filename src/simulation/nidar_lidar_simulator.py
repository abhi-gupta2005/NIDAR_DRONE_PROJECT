#!/usr/bin/env python3

import math
import random

import numpy as np


SCAN_HZ = 10.0
MEASUREMENTS_PER_SECOND = 5000
SAMPLES_PER_SCAN = int(MEASUREMENTS_PER_SECOND / SCAN_HZ)
MAX_RANGE_M = 12.0

ROOM_W = 8.0
ROOM_H = 6.0

SENSOR_X = ROOM_W / 2
SENSOR_Y = ROOM_H / 2

PERSON_RADIUS = 0.28
PERSON_START = np.array([4.8, 2.0])
PERSON_END = np.array([3.2, 4.2])
PERSON_PERIOD = 12.0


class CircleObstacle:
    def __init__(self, name, x, y, radius):
        self.name = name
        self.x = x
        self.y = y
        self.radius = radius


STATIC_OBSTACLES = [
    CircleObstacle("Chair", 2.0, 1.7, 0.35),
    CircleObstacle("Box", 6.1, 1.4, 0.45),
    CircleObstacle("Pillar", 5.8, 4.7, 0.30),
]


def moving_person(t):
    phase = (t % PERSON_PERIOD) / PERSON_PERIOD
    s = 0.5 - 0.5 * math.cos(2 * math.pi * phase)

    x = PERSON_START[0] + (PERSON_END[0] - PERSON_START[0]) * s
    y = PERSON_START[1] + (PERSON_END[1] - PERSON_START[1]) * s

    return CircleObstacle("Person", float(x), float(y), PERSON_RADIUS)


def ray_to_wall(angle):
    dx = math.cos(angle)
    dy = math.sin(angle)
    distances = []

    if dx > 1e-9:
        distances.append((ROOM_W - SENSOR_X) / dx)
    elif dx < -1e-9:
        distances.append((0 - SENSOR_X) / dx)

    if dy > 1e-9:
        distances.append((ROOM_H - SENSOR_Y) / dy)
    elif dy < -1e-9:
        distances.append((0 - SENSOR_Y) / dy)

    positive = [d for d in distances if d > 0]
    return min(positive) if positive else MAX_RANGE_M


def ray_circle(angle, obstacle):
    ox = SENSOR_X - obstacle.x
    oy = SENSOR_Y - obstacle.y
    dx = math.cos(angle)
    dy = math.sin(angle)

    b = 2 * (ox * dx + oy * dy)
    c = ox * ox + oy * oy - obstacle.radius * obstacle.radius
    disc = b * b - 4 * c

    if disc < 0:
        return None

    root = math.sqrt(disc)
    t1 = (-b - root) / 2
    t2 = (-b + root) / 2

    valid = [t for t in (t1, t2) if t > 0]
    return min(valid) if valid else None


def make_scan(t, noise_scale=1.0):
    angles = np.linspace(0, 360, SAMPLES_PER_SCAN, endpoint=False)

    person = moving_person(t)
    obstacles = STATIC_OBSTACLES + [person]

    distances = np.empty(len(angles), dtype=float)
    hits = []

    for i, deg in enumerate(angles):
        angle = math.radians(float(deg))

        d = ray_to_wall(angle)
        hit = "Wall"

        for obstacle in obstacles:
            h = ray_circle(angle, obstacle)

            if h is not None and h < d:
                d = h
                hit = obstacle.name

        sigma = (0.006 + 0.0015 * d) * noise_scale
        d += random.gauss(0, sigma)
        d = max(0.03, min(MAX_RANGE_M, d))

        if random.random() < 0.008:
            d = np.nan
            hit = "Invalid"

        distances[i] = d
        hits.append(hit)

    return angles, distances, hits, person
