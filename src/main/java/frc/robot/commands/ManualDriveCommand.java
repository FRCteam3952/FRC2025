package frc.robot.commands;

import frc.robot.controllers.Controller;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Flags;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.Util;

public class ManualDriveCommand extends Command {
    public static final double MAX_SPEED_METERS_PER_SEC = Flags.DriveTrain.LOWER_MAX_SPEED ? 1.5 : 3;
    public static final double MAX_ROT_SPEED_ANGULAR = 3;
    private final DriveTrainSubsystem driveTrain;
    private final Controller joystick;
    // private final AprilTagHandler aprilTagHandler;
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1);
    // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);
    // private final Trigger autoAimSubwoofer;
    // private final LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    // private boolean wasAutomaticallyDrivingLastFrame = false;

    public ManualDriveCommand(DriveTrainSubsystem driveTrain, Controller joystick) { //AprilTagHandler aprilTagHandler) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;
        // this.aprilTagHandler = aprilTagHandler;
        // this.autoAimSubwoofer = ControlHandler.get(joystick, ControllerConstants.AUTO_AIM_FOR_SHOOT);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // this.driveTrain.setPose(new Pose2d(2, 7, RobotGyro.getRotation2d()));
    }

    private double flipFactor() {
        if (Util.onBlueTeam()) {
            return 1;
        } else {
            return -1;
        }
    }

    @Override
    public void execute() {
        // System.out.println("vert: " + this.joystick.getRightVerticalMovement() + ", hor: " + this.joystick.getRightHorizontalMovement());
        // this.driveTrain.drive(this.joystick.getVerticalMovement());
        double flip = flipFactor();
        double ySpeed = Util.squareKeepSign(this.ySpeedLimiter.calculate(this.joystick.getLeftVerticalMovement()  * flip)) * MAX_SPEED_METERS_PER_SEC;
        double xSpeed = Util.squareKeepSign(this.xSpeedLimiter.calculate(this.joystick.getLeftHorizontalMovement() * flip)) * MAX_SPEED_METERS_PER_SEC;
        // System.out.println("xSpeed = " + xSpeed);
        // System.out.println("ySpeed = " + ySpeed);
        double rotSpeed = -this.joystick.getRightHorizontalMovement() * MAX_ROT_SPEED_ANGULAR;


        // System.out.println("forward speed: " + ySpeed + ", x speed: " + xSpeed);
        // System.out.println("y: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftVerticalMovement()) + ", x: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftHorizontalMovement()));

        this.driveTrain.drive(xSpeed, ySpeed, rotSpeed, true);
    }

    /**
     * Calculate the angle the gyroscope should be at in order to look at the speaker
     *
     * @return A Rotation2d representing the angle to the speaker. The gyroscope value should equal this value when the robot is facing the speaker.
     * //
     */
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
