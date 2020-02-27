package com.millburnrobotics.skystone.subsystems;

import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.SideClawConstants.*;

public class SideClaw extends Subsystem {
    public enum SideClawSide {
        LEFT,
        RIGHT
    }
    private SideClawSide side;
    ElapsedTime changeSideArm = new ElapsedTime();
    ElapsedTime changeSideClaw = new ElapsedTime();

    ElapsedTime changeSide = new ElapsedTime();
    @Override
    public void init(boolean auto) {
        if (auto) {
            if (Robot.getInstance().side == Constants.Side.BLUE) {
                this.side = SideClawSide.LEFT;
            } else {
                this.side = SideClawSide.RIGHT;
            }
            armInit();
            clawOpen();
        } else {
            armUp();
            clawClose();
            changeSideArm.reset();
            changeSideClaw.reset();
            changeSide.reset();
            side = SideClawSide.LEFT;
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("CurrentSideArmPosition", getArmPosition());
        telemetry.addData("CurrentSideClawPosition", getArmPosition());
        telemetry.addData("SideClawSide", side.toString());
    }

    @Override
    public void update() {

    }

    public void setSide(SideClawSide side) {
        this.side = side;
    }

    public void toggleSide() {
        if (changeSide.milliseconds() > 50) {
            if (this.side == SideClawSide.LEFT) {
                side = SideClawSide.RIGHT;
            } else {
                side = SideClawSide.LEFT;
            }
            changeSide.reset();
        }
    }

    public void armUp() {
        if (this.side == SideClawSide.LEFT) {
            setArmPosition(SIDE_ARM_L_UP_POS);
        } else {
            setArmPosition(SIDE_ARM_R_UP_POS);
        }
    }
    public void armMid() {
        if (this.side == SideClawSide.LEFT) {
            setArmPosition(SIDE_ARM_L_MID_POS);
        } else {
            setArmPosition(SIDE_ARM_R_MID_POS);
        }
    }
    public void armDown() {
        if (this.side == SideClawSide.LEFT) {
            setArmPosition(SIDE_ARM_L_DOWN_POS);
        } else {
            setArmPosition(SIDE_ARM_R_DOWN_POS);
        }
    }
    public void armInit() {
        if (this.side == SideClawSide.LEFT) {
            setArmPosition(SIDE_ARM_L_INIT_POS);
        } else {
            setArmPosition(SIDE_ARM_R_INIT_POS);
        }
    }
    public void setArmPosition(double pos) {
        changeSideArm.reset();
        if (this.side == SideClawSide.LEFT) {
            Robot.getInstance().sideClawArmL.setPosition(pos); // yes bc i reversed it and fuck this
        } else {
            Robot.getInstance().sideClawArmR.setPosition(pos);
        }
    }
    public double getArmPosition() {
        if (this.side == SideClawSide.LEFT) {
            return Robot.getInstance().sideClawArmL.getPosition();
        } else {
            return Robot.getInstance().sideClawArmR.getPosition();
        }
    }
    public boolean canToggleSideArm() {
        return changeSideArm.milliseconds() > 25;
    }

    public void clawClose() {
        if (this.side == SideClawSide.LEFT) {
            setClawPosition(SIDE_CLAW_L_CLOSE_POS);
        } else {
            setClawPosition(SIDE_CLAW_R_CLOSE_POS);
        }
    }
    public void clawOpen() {
        if (this.side == SideClawSide.LEFT) {
            setClawPosition(SIDE_CLAW_L_OPEN_POS);
        } else {
            setClawPosition(SIDE_CLAW_R_OPEN_POS);
        }
    }
    public void setClawPosition(double pos) {
        changeSideClaw.reset();
        if (this.side == SideClawSide.LEFT) {
            Robot.getInstance().sideClawClawL.setPosition(pos);
        } else {
            Robot.getInstance().sideClawClawR.setPosition(pos);
        }
    }
    public double getClawPosition() {
        return (this.side == SideClawSide.LEFT) ? Robot.getInstance().sideClawClawL.getPosition() : Robot.getInstance().sideClawClawR.getPosition();
    }
    public boolean canToggleSideClaw() {
        return changeSideClaw.milliseconds() > 25;
    }

    public SideClawSide getSide() {
        return side;
    }
}
