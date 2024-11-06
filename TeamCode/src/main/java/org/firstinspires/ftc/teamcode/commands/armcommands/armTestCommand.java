package org.firstinspires.ftc.teamcode.commands.armcommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandBase;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@Config
public class armTestCommand extends CommandBase {
    PIDFController armController;

    private MotionProfile profile;

    ElapsedTime timer = new ElapsedTime();

    public static double kP_Proportional = 0.022, kI_Integral = 0, kD_Derivative = 0.0015;
    PIDCoefficients coefficients = new PIDCoefficients(kP_Proportional, kI_Integral, kD_Derivative);

    public static double kV_FeedForwardsVelocity = 0.0, kA_FeedForwardsAcceleration = 0.0, kStatic_FeedForwardsStaticFriction = 0.0;

    public static int ARM_POS_TOLERANCE = 20,
            ARM_POS_TOLERANCE_STRICT = 15;


    private double armPosition = 0, armVelocity = 0, controllerOutput = 0;

    final double MOTION_PROFILE_MAX_VELOCITY = 300,
            MOTION_PROFILE_MAX_ACCEL = 325,
            MOTION_PROFILE_MAX_JERK = 0;

    boolean holdAtEnd;
    final Arm arm;
    final double targetPosition;

    double setPointPos;

    public static double gravity = 0.15;
    private static double holdPower = 0.0;

    public armTestCommand(Arm arm, int targetPosition, boolean holdAtEnd) {
        this(arm, targetPosition, holdAtEnd, false);
    }

    public armTestCommand(Arm arm, int targetPosition, boolean holdAtEnd, boolean strict) {
        addRequirements(arm);

        if (strict) this.ARM_POS_TOLERANCE = ARM_POS_TOLERANCE_STRICT;

        this.holdAtEnd = holdAtEnd;
        this.arm = arm;
        this.targetPosition = targetPosition;

        setPointPos = targetPosition;

        armController = new PIDFController(coefficients, kV_FeedForwardsVelocity, kA_FeedForwardsAcceleration, kStatic_FeedForwardsStaticFriction, (x, v) -> {
            double kG = gravity;

            return kG * arm.getVoltageComp();
        });

        armController.setOutputBounds(-1, 1);
    }

    @Override
    public void initialize() {
        armController.reset();

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(arm.getArmPosition(), arm.getArmVelocity()),
                new MotionState(targetPosition, 0),
                MOTION_PROFILE_MAX_VELOCITY,
                MOTION_PROFILE_MAX_ACCEL,
                MOTION_PROFILE_MAX_JERK
        );

        timer.reset();
    }

    @Override
    public void execute() {
        armPosition = arm.getArmPosition();
        armVelocity = arm.getArmVelocity();

        double currentTime = timer.seconds();

        MotionState state = profile.get(currentTime);

        armController.setTargetPosition(state.getX());
        armController.setTargetVelocity(state.getV());
        armController.setTargetAcceleration(state.getA());

        arm.setPowerActual(controllerOutput);

        setPointPos = state.getX();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetPosition - armPosition) <= ARM_POS_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        if (holdAtEnd) arm.setPowerActual(holdPower);
        else arm.brake_power();
    }
}
