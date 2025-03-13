package frc.robot.commands.drive;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;
import java.util.Map;

public class ReefAutoDriveManager {
	// General idea:
	// pre-generate enough PathPlanner paths from various start positions -> each reef location
	// During gameplay, driver will set target reef
	// Robot will find the closest path start location, dynamic-path towards that point, then drive to the reef with pre-gen path
	// Ideally, pick a path with a start point that's closer to the reef rather than farther
	// We want to avoid high changes in velocity (both magnitude and direction)
	// We also want to avoid being overly inefficient (trade-off between efficiency and # of pregens)
	
	/**
	 * A map of reef indices (1 -> 6) to a list of paths
	 */
	private final Map<Integer, List<PathPlannerPath>> reefIndexToPathList;
	
	public ReefAutoDriveManager(Map<Integer, List<PathPlannerPath>> reefIndexToPathList) {
		this.reefIndexToPathList = reefIndexToPathList;
	}
	
	public List<PathPlannerPath> getPathsForReef(int reefIdx) {
		return reefIndexToPathList.get(reefIdx);
	}
	
	public PathPlannerPath getPathForReef(int reefIdx, Pose2d currentPose) {
		List<PathPlannerPath> paths = getPathsForReef(reefIdx);
		for(PathPlannerPath path : paths) {
			path.getStartingHolonomicPose();
		}
		return null;
	}
}
