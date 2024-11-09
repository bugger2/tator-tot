package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private SparkMax driveLeft;
    private SparkMax driveRight;
    private DifferentialDrive diffDrive;
    private final Pigeon2 gyro;
    private Angle initialAngle = Degrees.of(0);

    private DifferentialDriveOdometry odometry;

    private PIDController pidController;

    private final double ROTATIONS_PER_METER = 1;

    public Drivetrain() {
        driveLeft = new SparkMax(33, SparkLowLevel.MotorType.kBrushless);
        driveRight = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);
        diffDrive = new DifferentialDrive(driveLeft, driveRight);
        driveRight.setInverted(true);
        gyro = new Pigeon2(0);

        driveLeft.getEncoder().setPosition(0);
        driveRight.getEncoder().setPosition(0);

        pidController = new PIDController(0, 0, 0);

        odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    }

    @Override
    public void periodic() {
        odometry.update(
            new Rotation2d(getYaw()),
            rotationsToMeters(driveLeft.getEncoder().getPosition()),
            rotationsToMeters(driveRight.getEncoder().getPosition())
        );
    }

    private double rotationsToMeters(double rotations) {
        return rotations / ROTATIONS_PER_METER;
    }
    
    private void setSpeeds(double forward, double rotation) {
        // negatives to reverse forward direction so that forward is the RSL
        diffDrive.arcadeDrive(-forward, -rotation);
    }

    public Command drive(DoubleSupplier forward, DoubleSupplier rotation) {
        return this.run(() ->
            setSpeeds(
                forward.getAsDouble(),
                rotation.getAsDouble()
            )
        );
    }

    public Command turn(Angle setpoint) {
        return this.run(() -> {
            double speed = pidController.calculate(
                getYaw().in(Degrees), initialAngle.in(Degrees));
            setSpeeds(-speed, speed);
        }).until(() -> Math.abs(getYaw().minus(initialAngle).in(Degrees)) >= setpoint.in(Degrees))
            .beforeStarting(() -> {
                initialAngle = getYaw();
                double setpointDeg = setpoint.in(Degrees);
                pidController = new PIDController(.5/setpointDeg, .2/setpointDeg, 0);
            });
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    public Angle getYaw() {
        return gyro.getYaw().getValue();
    }

    public void resetGyro() {
        gyro.setYaw(0);
    }
}    
