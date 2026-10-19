# Robot Localization Project Writeup
Prem Sethumadhavan and Trevor McDonald\
October 20th, 2026

Your write-up should touch on the following topics.

    What was the goal of your project?
    How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
    Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
    Describe the tests that you implemented and conducted to verify your code, and their results.
    What if any challenges did you face along the way?
    What would you do to improve your project if you had more time?
    Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

## 1. Project Overview
The Robot Localization project answers the question "where is the robot in space?" To answer that question, there are a couple of steps to be accomplished. Given a known map, we initialize hundreds of particles and run a particle filtering algorithm. The algorithm takes input from the robots lidar sensor data and completes transformations using its odometry frame to generate another set of particles. Overtime, the guesses get better and better and we know the rough location of our robot in the given space.

## 2. Code Architecture


## 3. Design Decisions

## 4. Testing & Verification

## 5. Challenges, Improvements, & Lessons





