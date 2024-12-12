package org.firstinspires.ftc.teamcode.commands.armcommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmCorrected;

@Config
public class ProfiledArmCommand extends CommandBase {
    PIDFController armController;
    private MotionProfile profile;
    ElapsedTime timer = new ElapsedTime();

    PIDCoefficients coefficients = new PIDCoefficients(0.006, 0.0, 0.0003); // Adjust PID coefficients as needed

    // Feedforward Coefficients
    double kV = 0.0, kA = 0.0, kStatic = 0.00;

    // The tolerance for getting to a certain position. Strict tries to get just a bit closer.
    private double ARM_POSITION_TOLERANCE = 15,
            ARM_POSITION_TOLERANCE_STRICT = 10;

    private double armPosition = 0, armVelocity = 0, controllerOutput = 0;

    final double MOTION_PROFILE_MAX_VELOCITY = 300,
            MOTION_PROFILE_MAX_ACCEL = 325,
            MOTION_PROFILE_MAX_JERK = 0;

    boolean holdAtEnd;
    final ArmCorrected arm;
    final double targetPosition;

    double setPointPos;
    boolean isInside;

    public double gravity = 0.05;

    public ProfiledArmCommand(ArmCorrected arm, int targetPosition, boolean holdAtEnd) {
        this(arm, targetPosition, holdAtEnd, false, true, 0);
    }

    public ProfiledArmCommand(ArmCorrected arm, int targetPosition, boolean holdAtEnd, boolean strict) {
        this(arm, targetPosition, holdAtEnd, strict, true, 0);
    }

    public ProfiledArmCommand(ArmCorrected arm, int targetPosition, boolean holdAtEnd, boolean strict, boolean isInside) {
        this(arm, targetPosition, holdAtEnd, strict, isInside, 0);
    }

    public ProfiledArmCommand(ArmCorrected arm, int targetPosition, boolean holdAtEnd, boolean strict, boolean isInside, int delay) {
        // Add a delay before proceeding with initialization
        try {
            Thread.sleep(delay); // ms delay before proceeding
        } catch (InterruptedException e) {
            e.printStackTrace(); // Handle interruption
        }

        addRequirements(arm);

        if (strict) this.ARM_POSITION_TOLERANCE = ARM_POSITION_TOLERANCE_STRICT;

        this.holdAtEnd = holdAtEnd;
        this.arm = arm;
        this.targetPosition = targetPosition;

        setPointPos = targetPosition;

        // Gravity feedforward term to counteract gravity
        armController = new PIDFController(coefficients, kV, kA, kStatic, (x, v) -> {
            // Feedforward Gravitational Below
            double kG = gravity;

            return kG * arm.getVoltageComp();
        });

        // Prevent it from going down TOO fast
        // This is the same as the maximum amount of kG compensation subtracted from max negative value
        armController.setOutputBounds(-1.0, 1.0);
    }

    @Override
    public void initialize(){
        armController.reset();

        // Generate the motion profile
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(arm.getCurrentPosition(), arm.getArmVelocity()),
                new MotionState(targetPosition, 0),
                MOTION_PROFILE_MAX_VELOCITY,
                MOTION_PROFILE_MAX_ACCEL,
                MOTION_PROFILE_MAX_JERK
        );

        timer.reset();
    }

    @Override
    public void execute(){
        armPosition = arm.getCurrentPosition();
        armVelocity = arm.getArmVelocity();

        double currentTime = timer.seconds();

        MotionState state = profile.get(currentTime);

        // Update the PIDFController with the profile's state
        armController.setTargetPosition(state.getX());
        armController.setTargetVelocity(state.getV());
        armController.setTargetAcceleration(state.getA());

        controllerOutput = armController.update(armPosition, armVelocity);

        // Update the lift power with the controller
        arm.manual(controllerOutput);

        // Additional SetPoint variables can be set here if needed
        setPointPos = state.getX();
    }

    @Override
    public boolean isFinished() {
        // End if the lift position is within the tolerance
        return Math.abs(targetPosition - armPosition) <= ARM_POSITION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted){
        if (holdAtEnd) arm.manual(0); //TODO: CHECK FOR ISSUES ARM_POWER_INSIDE
        else arm.brake(); // Assuming brake_power() is a method to stop the lift
    }

    public double getGravity() {
        return gravity;
    }
}
