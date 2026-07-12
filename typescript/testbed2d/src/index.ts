import {Testbed} from "./Testbed";
import * as CollisionGroups from "./demos/collisionGroups";
import * as Cubes from "./demos/cubes";
import * as Keva from "./demos/keva";
import * as Heightfield from "./demos/heightfield";
import * as Polyline from "./demos/polyline";
import * as RevoluteJoints from "./demos/revoluteJoints";
import * as LockedRotations from "./demos/lockedRotations";
import * as ConvexPolygons from "./demos/convexPolygons";
import * as CharacterController from "./demos/characterController";
import * as PidController from "./demos/pidController";
import * as Voxels from "./demos/voxels";

import("@dimforge/rapier2d").then((RAPIER) => {
    let builders = new Map([
        ["collision groups", CollisionGroups.initWorld],
        ["character controller", CharacterController.initWorld],
        ["convex polygons", ConvexPolygons.initWorld],
        ["cubes", Cubes.initWorld],
        ["heightfield", Heightfield.initWorld],
        ["joints: revolute", RevoluteJoints.initWorld],
        ["keva tower", Keva.initWorld],
        ["locked rotations", LockedRotations.initWorld],
        ["pid controller", PidController.initWorld],
        ["polyline", Polyline.initWorld],
        ["voxels", Voxels.initWorld],
    ]);
    let testbed = new Testbed(RAPIER, builders);
    testbed.run();
});
