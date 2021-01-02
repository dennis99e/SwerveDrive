/* (C)2020-2021 */
package org.frc5687.diffswerve.robot.util;

public class Vector2d {

    private final double _x;
    private final double _y;

    public Vector2d(double x, double y) {
        _x = x;
        _y = y;
    }

    public double getX() {
        return _x;
    }

    public double getY() {
        return _y;
    }

    public double getMagnitude() {
        return Math.sqrt(_x * _x + _y * _y);
    }

    public double getAngle() {
        return Math.atan2(_y, _x);
    }

    public Vector2d add(Vector2d other) {
        return new Vector2d(_x + other.getX(), _y + other.getY());
    }

    public Vector2d scale(double scale) {
        return new Vector2d(getX() * scale, getY() * scale);
    }

    public Vector2d normalize() {
        return scale(1.0 / getMagnitude());
    }

    public Vector2d normalize(double target) {
        return scale(target / getMagnitude());
    }

    public Vector2d rotate(double ang) {
        return new Vector2d(
                _x * Math.cos(ang) - _y * Math.sin(ang), _x * Math.sin(ang) + _y * Math.cos(ang));
    }

    public double dot(Vector2d other) {
        return getX() * other.getX() + getY() * other.getY();
    }

    public Vector2d abs() {
        return new Vector2d(Math.abs(_x), Math.abs(_y));
    }

    public Vector2d clone() {
        return new Vector2d(_x, _y);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vector2d)) {
            return false;
        }
        Vector2d other = (Vector2d) obj;
        if (Double.doubleToLongBits(_x) != Double.doubleToLongBits(other._x)) {
            return false;
        }
        if (Double.doubleToLongBits(_y) != Double.doubleToLongBits(other._y)) {
            return false;
        }
        return true;
    }
}
