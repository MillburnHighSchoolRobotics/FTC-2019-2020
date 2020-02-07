package com.millburnrobotics.skystone.auto.modes.twoblock;

import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.WaitAction;
import com.millburnrobotics.skystone.auto.actions.chainbar.ChainBarUpAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawArmDownAction;
import com.millburnrobotics.skystone.auto.actions.sideclaw.SideClawArmUpAction;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Blue")
public class TwoBlockBlueMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.RED;
        runAction(new SideClawArmUpAction());
        runAction(new WaitAction(2000));
        runAction(new SideClawArmDownAction());
        runAction(new WaitAction(2000));
    }
}
