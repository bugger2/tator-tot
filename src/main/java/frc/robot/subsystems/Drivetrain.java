// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Drivetrain extends SubsystemBase {
    SparkMax driveOne;
    SparkMax driveTwo;
    DifferentialDrive diffDrive;

    private final Pigeon2 gyro;


    public Drivetrain() {
        driveOne = new SparkMax(33, SparkLowLevel.MotorType.kBrushless);
        driveTwo = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);
        diffDrive = new DifferentialDrive(driveOne, driveTwo);
        driveTwo.setInverted(true);
        gyro = new Pigeon2(0);
    }
    


    public Command drive(DoubleSupplier motor1Speed, DoubleSupplier motor2Speed) {
        // negatives to reverse forward direction so that forward is the RSL
        return this.run(() ->
            diffDrive.tankDrive(
                -motor1Speed.getAsDouble(),
                -motor2Speed.getAsDouble()
            )
        );
    }

    public Command resetEncoders() {
        return this.runOnce(() -> {
            driveOne.getEncoder().setPosition(0.0);
            driveTwo.getEncoder().setPosition(0.0);
        });
    }

    public Angle getYaw() {
        return gyro.getYaw().getValue();
    }

    @Override
    public void periodic() {

    }
}
