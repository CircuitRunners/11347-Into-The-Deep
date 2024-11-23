package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.support.RunAction;

@Config
public class Diffy extends SubsystemBase {
    private Servo leftDiffyServo, rightDiffyServo;
    private ServoStates currentState;
    private final ElapsedTime switchTimer = new ElapsedTime();
    public enum ServoStates {
        CENTER(0.05);

        private final double position;
        ServoStates(double position) {
            this.position = position;
        }


        private double getPosition() {
            return this.position;
        }
    }

    public RunAction CenterDiffy;

    public Diffy(HardwareMap h) {
        leftDiffyServo = h.get(Servo.class, "leftDiffyServo");
        rightDiffyServo = h.get(Servo.class, "rightDiffyServo");
        currentState = ServoStates.CENTER;
        leftDiffyServo.setPosition(currentState.getPosition());
        rightDiffyServo.setPosition(currentState.getPosition());

        switchTimer.reset();

        CenterDiffy = new RunAction(this::CenterDiffy);

    }

    public void CenterDiffy() {
        setPosition(ServoStates.CENTER);
    }

    public void setPosition(ServoStates state) {
        currentState = state;

        leftDiffyServo.setPosition(state.getPosition());
        rightDiffyServo.setPosition(state.getPosition());
    }

    public double currentPosition() {
        return leftDiffyServo.getPosition();
    }
}
