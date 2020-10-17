/* (C)2020 */
package org.frc5687.diffswerve.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import javax.swing.*;

public class Constants {
    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;

    public static class DriveTrain {
        public static final double WIDTH = 20.0; // TODO: Figure out which units to use.
        public static final double LENGTH = 20.0; // TODO: Figure out which units to use.
        public static final Translation2d FRONT_RIGHT_POSITION =
                new Translation2d(WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d FRONT_LEFT_POSITION =
                new Translation2d(-WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d BOTTOM_RIGHT_POSITION =
                new Translation2d(WIDTH / 2.0, -LENGTH / 2.0);
        public static final Translation2d BOTTOM_LEFT_POSITION =
                new Translation2d(-WIDTH / 2.0, -LENGTH / 2.0);
        public static final boolean RIGHT_INVERTED = false;
        public static final boolean LEFT_INVERTED = false;
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double GEAR_RATIO = 1.0;

        public static final double VELOCITY_KP = 0.25;
        public static final double VELOCITY_KI = 0.0001;
        public static final double VELOCITY_KD = 0.3;
        public static final double VELOCITY_KF = 0.000;
    }

    public static class DifferentialSwerveModule {

        public static final double GEAR_RATIO_WHEEL = 1.0; // 6.48;
        public static final double GEAR_RATIO_STEER = 1.0; // 4.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.007;
        public static final double INERTIA_STEER = 0.007;
        public static final double Q_AZIMUTH_ANG_VELOCITY = 1;
        public static final double Q_AZIMUTH = 0.1;
        public static final double Q_WHEEL_ANG_VELOCITY = 1;
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = 2; // degrees
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 200; // RPM
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 200; // RPM
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // degrees
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 10; // degrees
    }
}
