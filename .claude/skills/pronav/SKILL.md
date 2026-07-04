---
name: pronav
description: Reference for implementing and tuning the guidance laws - pursuit and proportional navigation (pro-nav), the core of this project. Use when building or tuning M3 (static intercept) and M4 (moving target, pursuit-vs-pro-nav comparison).
---
# Guidance: pursuit and proportional navigation

The portfolio headline is proportional navigation - implement it carefully and measure it.

## Pursuit (baseline)
Aim the interceptor's velocity straight at the target's current position. Simple, but lags a moving target because it always chases where the target *was*.

## Proportional navigation (pro-nav)
Command lateral acceleration proportional to the line-of-sight (LOS) rotation rate:
  a_cmd = N * Vc * lambda_dot
- `lambda_dot` = how fast the bearing to the target is rotating (LOS rate).
- `Vc` = closing speed.
- `N` = navigation constant, typically 3-5 (start N=3-4).
Intuition: if the bearing is not rotating you are already on a collision course; if it is, turn to null the rotation. This anticipates the target, so miss distance is far smaller than pursuit against a mover.

## In this sim
- Get bearing/LOS from the AprilTag detection (tag pose + camera intrinsics -> relative position -> LOS angle).
- Feed the command into PX4 OFFBOARD via MAVSDK (velocity or attitude setpoints).
- Run pursuit and pro-nav on the SAME target paths, log both miss distances, and compare. That comparison is the resume evidence.
- Mind units and frames (ENU / OpenCV / FRD - see GOALS.md).