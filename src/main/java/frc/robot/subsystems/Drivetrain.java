package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private SparkMax driveOne;
    private SparkMax driveTwo;
    private DifferentialDrive diffDrive;
    private final Pigeon2 gyro;
    private Angle initialAngle = Degrees.of(0);

    private PIDController pidController;

    public Drivetrain() {
        driveOne = new SparkMax(33, SparkLowLevel.MotorType.kBrushless);
        driveTwo = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);
        diffDrive = new DifferentialDrive(driveOne, driveTwo);
        driveTwo.setInverted(true);
        gyro = new Pigeon2(0);

        pidController = new PIDController(0, 0, 0);
    }
    
    private void setSpeeds(double motor1Speed, double motor2Speed) {
        // negatives to reverse forward direction so that forward is the RSL
        diffDrive.tankDrive(-motor1Speed -motor2Speed);
    }

    public Command drive(DoubleSupplier motor1Speed, DoubleSupplier motor2Speed) {
        return this.run(() ->
            setSpeeds(
                motor1Speed.getAsDouble(),
                motor2Speed.getAsDouble()
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

    public Angle getYaw() {
        return gyro.getYaw().getValue();
    }
}    
nn
