package org.frc5687.diffswerve.robot.util;

public interface ILoggingSource {
    void error(String message);
    void warn(String message);
    void info(String message);
    void debug(String message);
}
