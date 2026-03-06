// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static final class ShooterConstants {
        
        public static final int kHoodID = 20;
        public static final int kShooterTopLeftID = 21;
        public static final int kShooterTopRightID = 22;
        public static final int kShooterBottomRightID = 23;

        public static final double kDefaultShooterSpeed = 7.0;
        public static final double kBottomMotorRatio = -0.8; // run the bottom hood slower for backspin (stabler shot)

        // PositionDutyCycle takes inputs in rotations.
        // 8.88 rotations for 30 degrees (~40-70deg from the horizontal)
        // 0.296 rot ~~ 1deg
        public static final double kMaxRot = 8.8;
        public static final double kHoodDegsToRot = 0.296;
        public static final double kHoodP = 1.5;
        public static final double kHoodI = 0.0;
        public static final double kHoodD = 0.0;
        public static final double kHoodFeedForward = 0.229; // final later
        public static final double kHoodLowDegFromHorizontal = 40;

        public static final double kGravity = 9.807;
        
        // Linear Regression (from testing): 
        // y = a + bx, where y = velocity and x = motor speed
        // should test again since these were prototype numbers
        public static final double kA = -0.4498;
        public static final double kB = 8.682;

        // in meters
        public static final double shooterHeight = 0.5842; // 23 in to m
        public static final double drivebaseHeight = 0.1778; // 7 in
        public static final double startHeight = shooterHeight + drivebaseHeight;
        public static final double hubHeight = 1.8288; // 72 in (per game manual)
        public static final double deltaHeight = hubHeight - startHeight; 

        
    }
}