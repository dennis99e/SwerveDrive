/* (C)2020 */
package org.frc5687.diffswerve.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5687.diffswerve.robot.util.ILoggingSource;
import org.frc5687.diffswerve.robot.util.MetricTracker;
import org.frc5687.diffswerve.robot.util.OutliersContainer;
import org.frc5687.diffswerve.robot.util.RioLogger;

public abstract class OutliersSubsystem extends SubsystemBase implements ILoggingSource {
    private MetricTracker _metricTracker;
    private final Notifier _controlLoop = new Notifier(this::update);

    public OutliersSubsystem(OutliersContainer container) {
        container.registerSubSystem(this);
    }

    public void startNotifier(double kDt) {
        _controlLoop.startPeriodic(kDt);
    }

    public void update() {}

    @Override
    public void error(String message) {
        RioLogger.error(this, message);
    }

    @Override
    public void warn(String message) {
        RioLogger.warn(this, message);
    }

    @Override
    public void info(String message) {
        RioLogger.info(this, message);
    }

    @Override
    public void debug(String message) {
        RioLogger.debug(this, message);
    }

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    // Example of metrics collection. In a child class's constructor:
    // getMetricTracker().registerReportableMetricName("foo");
    // getMetricTracker().registerReportableMetricName("bar");
    // getMetricTracker().registerReportableMetricName("baz");
    //
    // Later on in the child class...
    // metric("foo", 123);
    // metric("bar", "elvis");
    // metric("baz", 42.42);
    // metric("pants", 99);    <~ This metric won't get written to USB storage because it wasn't
    // registered.

    protected void logMetrics(String... metrics) {
        _metricTracker = MetricTracker.createMetricTracker(getClass().getSimpleName(), metrics);
    }

    public abstract void updateDashboard();

    protected void enableMetrics() {
        if (_metricTracker != null) {
            _metricTracker.enable();
        }
    }

    protected void disableMetrics() {
        if (_metricTracker != null) {
            _metricTracker.disable();
        }
    }
}
