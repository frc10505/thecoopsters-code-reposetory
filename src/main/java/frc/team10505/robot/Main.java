/*
 * Copyright (C) 2025 Team 10505 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team10505.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {

  public Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }

}