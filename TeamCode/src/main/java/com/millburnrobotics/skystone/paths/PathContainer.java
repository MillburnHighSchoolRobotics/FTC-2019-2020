package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;

public interface PathContainer {
    Path buildPath(boolean blueSide);
}