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

        public static final double kHoodInitialPosition = 0.0;

        public static final double kGravity = 9.807;
        
        // Linear Regression: y = a + bx, where y = velocity and x = motor speed
        public static final double kA = -0.4498;
        public static final double kB = 8.682;

        // in meters
        public static final double shooterHeight = 0.5842; // 23 in to m
        public static final double drivebaseHeight = 0.1778; // 7 in
        public static final double startHeight = shooterHeight + drivebaseHeight;
        public static final double hubHeight = 1.8288; // 72in (per game manual)
        public static final double deltaHeight = hubHeight - startHeight; 

        // add the new gear ratios and stuff

        // public static final double hoodMobility = 3.465;
        // public static final double hoodMobilityRatio = hoodMobility / 13.772;
        
    }
}