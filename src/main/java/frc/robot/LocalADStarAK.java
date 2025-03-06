package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// public class LocalADStarAK implements Pathfinder {
//   private final ADStarIO io = new ADStarIO();

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
