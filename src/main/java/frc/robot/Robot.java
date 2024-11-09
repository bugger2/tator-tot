// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private CommandXboxController controller;

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;

    public Robot() {
        controller = new CommandXboxController(0);

        drivetrain = new Drivetrain();
        intake = new Intake();
        shooter = new Shooter();

        // if no other command is running on the drivetrain, make it drive
        drivetrain.setDefaultCommand(
            drivetrain.drive(controller::getLeftX, controller::getRightX)
        );

        // when the left trigger is held, and the sensor is NOT hit, run the picker motor
        // `whileTrue` kills that command if either condition becomes false
        controller.leftTrigger().and(intake.gamePieceDetected.negate())
            .whileTrue(intake.feed());

        // when the right trigger is held, and the sensor is hit, shoot the game piece
        controller.rightTrigger().and(intake.gamePieceDetected)
            .onTrue(shooter.shoot());

        // once the gamepiece has left the sensor for 0.5 seconds, stop shooting
        intake.gamePieceDetected.negate()
            .debounce(0.5)
            .onTrue(shooter.stop());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = drivetrain.drive(() -> 0.5, () -> 0) // drive at half power straight forward
            .until(() -> drivetrain.getOdometryPose().getTranslation().getX() >= 1) // until you've passed 1 meter
            .andThen(drivetrain.turn(Degrees.of(180))) // after that, turn 180 degrees
            .andThen(drivetrain.drive(() -> 0.5, () -> 0.0) // and after that, drive half power straight forward
                // ("forward" is robot relative in this case, meaning drive towards the bots front, which is now turned 180 degrees)
                .until(() -> drivetrain.getOdometryPose().getTranslation().getX() <= 0))
                // ^-- stop driving once you've reached the starting position of 0 meters            
            .beforeStarting(drivetrain::resetGyro); // and be sure to reset the gyro before driving

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
