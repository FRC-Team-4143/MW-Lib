package frc.mw_lib.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public abstract class GitLogger {

  private static StringPublisher project_name =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/PROJECT_NAME").publish();
  private static StringPublisher git_sha =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_SHA").publish();
  private static StringPublisher git_date =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_DATE").publish();
  private static StringPublisher git_branch =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_BRANCH").publish();
  private static StringPublisher build_date =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/BUILD_DATE").publish();
  private static StringPublisher dirty =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/DIRTY").publish();

  public static void logGitData(String mavenName, String gitSha, String gitDate, 
                                 String gitBranch, String buildDate, int dirtyFlag) {
    project_name.set(mavenName);
    git_sha.set(gitSha);
    git_date.set(gitDate);
    git_branch.set(gitBranch);
    build_date.set(buildDate);
    switch (dirtyFlag) {
      case 0:
        dirty.set("All changes committed");
        break;
      case 1:
        dirty.set("Uncommitted changes");
        break;
      default:
        dirty.set("Unknown");
        break;
    }
  }
}
