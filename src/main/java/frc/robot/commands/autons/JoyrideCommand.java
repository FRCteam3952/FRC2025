package frc.robot.commands.autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;

public class JoyrideCommand extends Command {
    // Don't go over these speeds or I won't run your code.
    public static final double MAX_ROT_SPEED_ANGULAR = 3;
    public static final double MAX_SPEED_METERS_PER_SEC = 3;
    public static final double DEFAULT_SPEED = MAX_ROT_SPEED_ANGULAR * 0.6;

    private final DriveTrainSubsystem driveTrain;
    private final Timer timer;
    private final double duration;

    public JoyrideCommand(DriveTrainSubsystem driveTrain, double duration) {
        this.driveTrain = driveTrain;
        this.timer = new Timer();
        this.duration = duration;

        addRequirements(driveTrain);
    }

    // This code is executed once, when the command starts executing. 
    @Override
    public void initialize() {
        this.timer.restart();
        /*
         * Overview of the driveTrain.drive(forwardSpeed, sidewaysSpeed, rotSpeed, fieldRelative) command:
         * Method to drive the robot (set the wheel speeds) using joystick info.
         * Parameters:
         *  forwardSpeed Speed of the robot in the x direction (forward).
         *  sidewaysSpeed Speed of the robot in the y direction (sideways).
         *  rotSpeed Angular rate of the robot.
         *  fieldRelative Whether the provided x and y speeds are relative to the field.
         *      false means that forward is towards the front of the robot; true means that forward means towards the front of the field.
         */

        // Example: just drive forwards slowly
        this.driveTrain.drive(1.8, 0, 0, false);
    }

    // This code is executed many times, once per frame while the command is executing.
    @Override
    public void execute() {
        // If nothing is executed here, the robot will keep driving in the same direction specified by `initialize()`

        // Example code: drive in a circle.
        // Begins going straight forwards.
        double timeElapsed = timer.get();
        double percentComplete = timeElapsed / this.duration;
        if(percentComplete<=0.33){
            this.driveTrain.drive(0, -1.8, 0, false);
        } else if (percentComplete<=0.66) {
            this.driveTrain.drive(1.8, 0, 0, false);
        } else if (percentComplete <=1) {
            this.driveTrain.drive(-1.8, 1.8, 0, false);
        } 
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Without this line of code, the robot would keep driving forwards.
        this.driveTrain.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(this.duration);
    }
}