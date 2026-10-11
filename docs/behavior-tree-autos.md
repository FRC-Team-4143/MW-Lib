# Behavior-tree autos

`com.marswars.bt` runs autos written as [BehaviorTree.CPP v4](https://www.behaviortree.dev/) XML
files, which open in [Arbor](https://github.com/coalman321/Arbor) (our tree editor and live viewer) and Groot2. Each `<Name>.xml` file in a robot project's autos
folder becomes an auto in the chooser, except library files (several trees and no
`main_tree_to_execute`, such as a shared `common.xml`), which autos pull in with `<include>`. mainbot's `src/main/deploy/autos/README.md` is
the guide to writing them.

**Setting it up in a robot project:**

```java
BehaviorTreeFactory factory = new BehaviorTreeFactory();      // BT.CPP built-ins + MW-Lib nodes
SwerveNodes.register(factory, SwerveSubsystem::getInstance);  // MW-Lib swerve nodes
RobotStateNodes.register(factory, RobotContainer::getInstance, RobotStates.class); // SetWantedState

// deploy/autos is an Arbor project: bt_project.json, nodes.xml and the trees/ folder
AutoManager.getInstance().registerAutos(BehaviorTreeAuto.loadAll(
        factory, Filesystem.getDeployDirectory().toPath().resolve("autos/trees")));
BtLiveServer.start(BtLiveServer.Options.defaults());     // live view for Arbor (port 1670)
BehaviorTreeFileLogger.enableDefault();                  // one .btlog (FileLogger2 format) per run
```

**What's included:**

| Piece | What it is |
|---|---|
| `bt.core`, `bt.control`, `bt.decorator`, `bt.action` | The engine, with BT.CPP v4 node semantics and all BT.CPP built-ins except scripting |
| `bt.xml` | BT.CPP v4 XML parser and writer: both node forms, `SubTree` port passing, `<include>`, `<TreeNodesModel>` |
| `ParallelDeadline` | MW-Lib control node: runs children together until the first one finishes, then halts the rest |
| `bt.RobotStateNodes` | `SetWantedState` and `IsSystemState` for the robot's super state machine. Autos use the same robot states and sequences as teleop |
| `bt.swerve.SwerveNodes` | `FollowTrajectory`, `WaitForChoreoEvent`, `SetSwerveState` and Choreo/chassis conditions for any `MwSwerveSubsystem` |
| `auto.BehaviorTreeAuto` | An `Auto` loaded from XML. It pre-loads and alliance-flips every trajectory the XML names, and re-reads the file when the auto is re-selected |
| Tree parameters | Ports declared for a tree in `<TreeNodesModel>` become dashboard values at `/Tuning/Autos/<auto>/<port>`, read into the blackboard each run |
| `bt.monitor` | Logs `BehaviorTree/{Active}` and `BehaviorTree/<auto>/{Transitions,Result,Error}` through `MwLog` (`Transitions` is each node's status change since the last tick as `"<node>: <STATE>"`; only what AdvantageScope can show, the tree's shape is Arbor's job). `BehaviorTreeFileLogger` writes a `.btlog` per run in BT.CPP's `FileLogger2` format, with every status transition, for replay in Arbor or Groot2 |
| `bt.debug.BtLiveServer` | Streams the running tree over WebSocket using the btlive v1 protocol (default port 1670) |

**Node palette.** MW-Lib's shared nodes are generated from their registrations
(`MwLibNodes.nodeSpecXml()`), so there is no file in this repo to keep in sync. A robot's palette (the
editor's `nodes.xml`) is MW-Lib's section with the robot's own nodes appended, from
`factory.writeTreeNodesModelXml(false)`; the robot regenerates it with `./gradlew test -PupdateNodeSpec`
in its own project.

Generic nodes any robot could use belong in MW-Lib, registered with `registerLibraryNode`. Robot
projects register only their own.
