package org.frc5687.diffswerve.robot;

import javax.swing.*;

public class Constants {
    public static final int  TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static class DriveTrain {
        public static final double WIDTH = 20.0; //TODO: Figure out which units to use.
        public static final double LENGTH = 20.0; //TODO: Figure out which units to use.
        public static final boolean RIGHT_INVERTED = false ;
        public static final boolean LEFT_INVERTED = false ;
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double GEAR_RATIO = 1.0;

    }
}
