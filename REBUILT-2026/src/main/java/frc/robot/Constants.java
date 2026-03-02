// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class Constants {
    public static final class ShooterConstants {
        public static final int kLeaderId = 12;
        public static final int kFollowerId = 11;

        public static final double kHoodInitialPosition = 0.0;
        public static final double kDefaultShooterSpeed = 7.0;

        public static final double kGravity = 9.807;
        
        // Linear Regression: y = a + bx, where y = velocity and x = motor speed
        public static final double kA = -0.4498;
        public static final double kB = 8.682;

        // in meters
        public static final double ballHeight = 0.5842; // without robot
        public static final double hubHeight = 1.8288; // 72in (per game manual)
        public static final double deltaHeight = hubHeight - ballHeight; 

        // in rotations
        public static final double hoodMobility = 3.465; // ratio from motor to all range of motion on hood 
        public static final double hoodMobilityRatio = hoodMobility / 13.772; // IMPORTANT: RANGE OF MOTION IS NOT 60, WE ARE WORKING TO UPDATE THAT
    }
}