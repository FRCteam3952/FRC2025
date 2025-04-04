package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.Util;

public class ManualDriveCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final AbstractController joystick;
    // private final AprilTagHandler aprilTagHandler;
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1);

    private final Trigger leftStickButton, rightStickButton;
    private boolean toggleMicroAdjustment = false;
    // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);
    // private final Trigger autoAimSubwoofer;
    // private final LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    // private boolean wasAutomaticallyDrivingLastFrame = false;

    public ManualDriveCommand(DriveTrainSubsystem driveTrain, AbstractController joystick) { //AprilTagHandler aprilTagHandler) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;

        this.leftStickButton = joystick.leftJoystickButtonTrigger();
        this.rightStickButton = joystick.rightJoystickButtonTrigger();

        this.rightStickButton.onTrue(new InstantCommand(() -> toggleMicroAdjustment = !toggleMicroAdjustment));
        // this.aprilTagHandler 7= aprilTagHandler;
        // this.autoAimSubwoofer = ControlHandler.get(joystick, ControllerConstants.AUTO_AIM_FOR_SHOOT);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // this.driveTrain.setPose(new Pose2d(2, 7, RobotGyro.getRotation2d()));
    }
    
    @Override
    public void execute() {
        // System.out.println(NetworkTablesUtil.getLimeyJson());
        // System.out.println("vert: " + this.joystick.getRightVerticalMovement() + ", hor: " + this.joystick.getRightHorizontalMovement());
        // this.driveTrain.drive(this.joystick.getVerticalMovement());
        double flip = DriveTrainSubsystem.flipFactor();
        // We need a negative sign here because the robot starts facing in the other direction
        
        double speedAdjust = 1;
        if(toggleMicroAdjustment) {
            speedAdjust = 0.25;
        }
        double ySpeed = Util.squareKeepSign(this.ySpeedLimiter.calculate(this.joystick.getLeftVerticalMovement() * flip)) * DriveTrainSubsystem.MAX_SPEED_METERS_PER_SEC * speedAdjust;
        double xSpeed = -Util.squareKeepSign(this.xSpeedLimiter.calculate(this.joystick.getLeftHorizontalMovement() * flip)) * DriveTrainSubsystem.MAX_SPEED_METERS_PER_SEC * speedAdjust;
        // System.out.println("xSpeed = " + xSpeed);
        // System.out.println("ySpeed = " + ySpeed);

        double rotSpeed = -this.joystick.getRightHorizontalMovement() * DriveTrainSubsystem.MAX_ROT_SPEED_ANGULAR * speedAdjust;
        // System.out.println("i am in manual drive normal");

        // System.out.println("forward speed: " + ySpeed + ", x speed: " + xSpeed);
        // System.out.println("y: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftVerticalMovement()) + ", x: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftHorizontalMovement()));

        this.driveTrain.drive(ySpeed, xSpeed, rotSpeed, true);
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
