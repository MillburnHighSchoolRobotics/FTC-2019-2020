package com.millburnrobotics.skystone.auto.actions;

import java.util.ArrayList;
import java.util.List;

public class SeriesAction implements Action {

    private Action current;
    private final ArrayList<Action> actions;

    public SeriesAction(List<Action> actions) {
        this.actions = new ArrayList<>(actions);
        current = null;
    }

    @Override
    public boolean isFinished() {
        return actions.isEmpty() && current == null;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        if (current == null) {
            if (actions.isEmpty()) {
                return;
            }

            current = actions.remove(0);
            current.start();
        }

        current.update();

        if (current.isFinished()) {
            current.done();
            current = null;
        }
    }

    @Override
    public void done() {
        if (current != null) {
            current.done();
        }
    }
}