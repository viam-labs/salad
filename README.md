# Module salad

A Viam module for automated salad assembly.

## build-coordinator

Orchestrates salad assembly by grabbing ingredients to target weights and delivering the finished bowl.
Ingredients are added in category order: base, protein, topping, then dressing.
For each ingredient, reads the scale before and after each grab to verify weight was added.
If 3 consecutive grabs produce no weight change (< 0.5g), errors out (possible empty bin).
Once all ingredients are added, calls deliver_bowl.

When a grab adds no weight, the next attempt probes deeper into the bin by `depth-step-mm`
(capped at `max-depth-offset-mm`) to reach lower food levels. If motion planning rejects
a depth as unreachable, the offset is halved and retried — a binary search for the
deepest reachable Z. The same grab is retried once at the original depth before the
deeper probe begins.

### config
```
{
    // required - name of the grabber-controls generic service
    "grabber-controls" : "<grabber-controls service>",

    // required - name of the bowl-controls generic service
    "bowl-controls" : "<bowl-controls service>",

    // required - name of the sensor used as a scale (must return a numeric reading)
    "scale-sensor" : "<sensor>",

    // optional - name of a viam:beanjamin:text-to-speech service
    // if set, announces completion after each build
    "text-to-speech" : "<text-to-speech service>",

    // required - list of available ingredients
    // ingredient name must match a bin name in the grabber-controls config
    // category must be one of: "base", "protein", "topping", "dressing"
    // ingredients are added to the bowl in category order
    "ingredients" : [
        {
            "name" : "lettuce",
            "grams-per-serving" : 30.0,
            "category" : "base"
        },
        {
            "name" : "chicken",
            "grams-per-serving" : 40.0,
            "category" : "protein"
        },
        {
            "name" : "tomato",
            "grams-per-serving" : 25.0,
            "category" : "topping"
        },
        {
            "name" : "croutons",
            "grams-per-serving" : 15.0,
            "category" : "topping"
        },
        {
            "name" : "ranch",
            "grams-per-serving" : 20.0,
            "category" : "dressing"
        }
    ],

    // optional - voxel-based filter applied to the merged PCD before
    // meshification, to remove "ghost" points (depth-camera artifacts that
    // float inside otherwise-empty bins). Two passes run in sequence:
    //   1. neighbor-count: drop a voxel if its (2r+1)^3 cube has fewer than
    //      "min-neighbors" occupied voxels — strips thin chains/wisps of
    //      noise that bridge unrelated clusters.
    //   2. connected-components: build 26-connected components over the
    //      survivors and drop any component smaller than
    //      "min-component-voxels" — kills locally dense but globally
    //      isolated ghost blobs.
    // Set min-neighbors or min-component-voxels to 0 to disable a pass.
    // Defaults shown below were tuned against typical fridge captures.
    "filter" : {
        "voxel-mm"             : 10,
        "neighbor-radius"      : 1,
        "min-neighbors"        : 8,
        "min-component-voxels" : 1000
    },

    // optional - decimate the Poisson-reconstructed mesh down to roughly
    // this many triangles using quadric error metrics. The mesh is loaded
    // as a world obstacle for every motion plan, so a smaller mesh means
    // faster planning. Set to 0 to disable decimation. Default: 5000.
    "mesh-target-triangles" : 5000,

    // optional - if true, skip ALL lil-arm motion during build_salad:
    // grab_bowl/grab_lid are skipped, and bowl-controls.reset will not
    // send the lil-arm home. Useful when the lil-arm is unreliable but
    // you still want it configured in bowl-controls. Default: false.
    "skip-lil-arm" : false,

    // optional - mm to descend deeper into the bin on each empty-handed
    // grab retry. Set <= 0 to disable depth probing. Default: 20.
    "depth-step-mm" : 20,

    // optional - cap on how many mm below the configured grab Z the
    // depth probe is allowed to reach. Set <= 0 to disable depth
    // probing. Default: 80.
    "max-depth-offset-mm" : 80
}
```

#### setup_station

Captures a point cloud from `imaging-camera`, filters it, runs meshification, and segments the mesh into bins. Writes the following stable assets to `/home/viam/assets/`:

- `merged.pcd` — raw merged point cloud captured from the camera.
- `filtered_merged.pcd` — point cloud after the filter pass; this is what gets meshed.
- `mesh.ply` — surface mesh (Poisson reconstruction).
- `zones.json` — per-bin segmentation of the mesh.

Tune the filter against any captured PCD via the CLI before changing config:

```
./bin/salad-cli filter --input <path>.pcd --viz \
  --voxel 10 --neighbor-radius 1 --min-neighbors 8 --min-component-voxels 1000
```

### DoCommand

#### build_salad
Map of ingredient name to number of servings. Grabs each ingredient until the scale shows the target weight has been reached, then delivers the bowl. Optionally include a customer name, which is shown in the UI during the build and used in the completion announcement if text-to-speech is configured.
```
{
    "build_salad" : {
        "lettuce" : 2,
        "tomato" : 1,
        "croutons" : 1
    },
    "customer_name" : "Alice"  // optional
}
```

#### status
Returns the current status and progress percentage of the salad build.
Progress is based on total servings completed + bowl delivery as steps.
For the example above: 2 + 1 + 1 + 1 (delivery) = 5 steps.
```
{ "status" : true }
```
Response:
```
{
    "status" : "adding lettuce",  // "idle", "adding <ingredient>", "delivering salad", or "complete"
    "progress" : 40.0,            // percentage 0-100
    "customer_name" : "Alice"     // name of the current order, empty string if not provided
}
```

## grabber-controls

Controls left/right grippers and bin switches for grabbing ingredients and delivering bowls.

### config
```
{
    // required - list of ingredient bins
    "bins" : [
        {
            "name" : "lettuce",
            "above-bin" : "<switch>",  // switch to position arm above this bin
            "in-bin" : "<switch>"      // switch to lower arm into this bin
        }
    ],

    // required - hover height above the bin's mean Z (mm), used for both approach and post-grab return
    "bin-hover-height-mm" : 100,
    "bin-hover-orientation" : { "x": 0, "y": 0, "z": 1, "th": 0 },

    // required - after returning to hover, the arm ascends a further bin-clearance-height-mm straight up
    // using a tight line tolerance and loose orientation tolerance to guarantee fridge clearance
    "bin-clearance-height-mm" : 50,

    // required - switches and grippers
    "high-above-bowl" : "<switch>",       // switch to position arm high above the bowl
    "left-gripper" : "<gripper>",         // left gripper for grabbing ingredients
    "left-home" : "<switch>",             // switch to send left arm home
    "right-gripper" : "<gripper>",        // right gripper for grabbing/delivering bowl
    "right-above-bowl" : "<switch>",      // switch to position right arm above bowl
    "right-grab-bowl" : "<switch>",       // switch to lower right arm to grab bowl
    "right-above-delivery" : "<switch>",  // switch to position right arm above delivery
    "right-bowl-delivery" : "<switch>",   // switch to lower right arm for delivery
    "right-home" : "<switch>",            // switch to send right arm home

    // optional - linear constraint tolerances for bin grab descent and ascent (default 1.0)
    "grab-line-tolerance-mm" : 1.0,          // max deviation from straight line during descent/ascent
    "grab-orientation-tolerance-degs" : 1.0, // max orientation deviation during descent/ascent

    // optional - linear constraint tolerances for the post-grab clearance ascent
    // tight line keeps the arm straight up (guarantees fridge clearance)
    // loose orientation lets the arm reach a natural configuration at the higher Z
    "clearance-line-tolerance-mm" : 1.0,          // default 1.0
    "clearance-orientation-tolerance-degs" : 45.0 // default 45.0
}
```

### DoCommand

#### get_from_bin
Grabs from the bin with the given zone ID and drops into the bowl.
Optionally descend `depth-offset-mm` below the configured grab Z to
reach lower food levels.
```
{
    "get_from_bin" : 2,
    "depth-offset-mm" : 0  // optional, default 0
}
```
Response includes the depth offset that was used:
```
{
    "success" : true,
    "bin" : "croutons",
    "depth-offset-mm" : 0,
    "message" : "..."
}
```

#### deliver_bowl
Picks up the bowl and delivers it.
```
{
    "deliver_bowl" : true
}
```

## bowl-controls

Controls the right gripper and switches for preparing and delivering bowls, plus an optional "lil-arm" for grabbing the lid and bowl from a stack.

### config
```
{
    // required - right gripper for grabbing/delivering bowl
    "right-gripper" : "<gripper>",

    // required - switch to position right arm above bowl
    "right-above-bowl" : "<switch>",

    // required - switch to lower right arm to grab bowl
    "right-grab-bowl" : "<switch>",

    // required - switch to position right arm above delivery
    "right-above-delivery" : "<switch>",

    // required - switch to lower right arm for delivery
    "right-bowl-delivery" : "<switch>",

    // required - switch to send right arm home
    "right-home" : "<switch>",

    // optional - gripper for the lil-arm used to grab the lid and bowl
    // if set, "lil-arm-home" is required, and "lil-arm-poses" should include
    // entries named "lid" and "bowl" to support grab_lid and grab_bowl
    "lil-arm-gripper" : "<gripper>",

    // required if "lil-arm-gripper" is set - switch to send lil-arm home
    "lil-arm-home" : "<switch>",

    // optional - list of named poses used by grab_lid and grab_bowl
    // pose name "lid" is used by grab_lid; "bowl" is used by grab_bowl
    "lil-arm-poses" : [
        {
            "name" : "lid",                 // required - pose name
            "above" : "<switch>",           // required - switch to position above the target
            "at" : "<switch>",              // required - switch to lower onto the target
            "center-above" : "<switch>",    // required - switch to position above the drop point
            "center-at" : "<switch>"        // required - switch to lower to the drop point
        },
        {
            "name" : "bowl",
            "above" : "<switch>",
            "at" : "<switch>",
            "center-above" : "<switch>",
            "center-at" : "<switch>"
        }
    ]
}
```

### DoCommand

#### deliver_bowl
Picks up the bowl from under the ingredient area and delivers it, then opens the gripper.
```
{
    "deliver_bowl" : true
}
```

#### prepare_bowl
Fetches a bowl from delivery and places it under the ingredient area.
```
{
    "prepare_bowl" : true
}
```

#### grab_lid
Uses the lil-arm to grab the lid from its stack and move it to the center drop point. Requires `lil-arm-gripper` to be configured and a `lil-arm-poses` entry named `lid`.
```
{
    "grab_lid" : true
}
```

#### grab_bowl
Uses the lil-arm to grab a bowl from its stack and move it to the center drop point. Requires `lil-arm-gripper` to be configured and a `lil-arm-poses` entry named `bowl`.
```
{
    "grab_bowl" : true
}
```

#### lil_arm_home
Sends the lil-arm to its home position. Requires `lil-arm-home` to be configured.
```
{
    "lil_arm_home" : true
}
```

#### reset
Sends the right arm home. If `lil-arm-gripper` is configured, also sends the lil-arm home unless `skip_lil_arm` is true.
```
{
    "reset" : true,
    "skip_lil_arm" : false
}
```

## dressing-controls

Controls the gripper and switches for pouring dressing onto the salad. Supports
multiple dressing bottles — the prepare/pour/post-pour/home poses are shared
across all dressings, while each dressing has its own `approach-grab` and
`grab` poses so different bottles can be picked up from and returned to
different slots.

### config
```
{
    // required - gripper for grabbing the dressing container
    "gripper" : "<gripper>",

    // required - switch to prepare dressing position
    "prepare-dressing" : "<switch>",

    // required - switch to pour dressing
    "pour-dressing" : "<switch>",

    // required - switch for second pour position
    "pour-dressing2" : "<switch>",

    // required - switch for post-pour position
    "post-pour-dressing" : "<switch>",

    // required - switch to send arm home
    "home" : "<switch>",

    // optional - generic service to shake the arm while pouring
    "shake-arm-service" : "<generic service>",

    // required - map of dressing options, keyed by ingredient name.
    // the name must match an ingredient with category "dressing" in the
    // build-coordinator config. each dressing's bottle is picked up and
    // returned to the same slot, sandwiched between approach-grab moves.
    "dressings" : {
        "<name>" : {
            // required - safe approach pose above this bottle slot.
            // visited before descending to grab and after lifting away,
            // on both pickup and put-back.
            "approach-grab" : "<switch>",

            // required - descent pose at the bottle slot, used for both
            // grabbing the bottle and setting it back down.
            "grab" : "<switch>"
        }
    }
}
```

### DoCommand

#### pour_dressing
Picks up the named dressing's bottle, pours over the bowl, returns the bottle
to its slot, and sends the arm home. The value must be the name of an entry in
the `dressings` map (and match an ingredient with category `dressing` in the
build-coordinator config).
```
{
    "pour_dressing" : "<dressing name>"
}
```

#### reset
Sends the arm home.
```
{
    "reset" : true
}
```

## chefs-kiss-controls

Controls the gripper and switches for performing a chef's kiss seasoning action over the bowl.

### config
```
{
    // required - switch to position the arm over the bowl
    "position" : "<switch>",

    // required - gripper for grabbing seasoning
    "gripper" : "<gripper>",

    // required - switch to send arm home
    "home" : "<switch>",

    // required - generic service to shake the arm
    "arm-shaker-service" : "<generic service>"
}
```

### DoCommand

#### chefs_kiss
Grabs seasoning, positions over the bowl, opens gripper, shakes arm, then returns home.
```
{
    "chefs_kiss" : true
}
```

## passthrough-to-camera

Vision service that passes through point clouds from a camera.

### config
```
{
    // required
    "camera" : "<camera>"
}
```

## file-vision

Vision service that returns a 3D mesh loaded from a PLY file.

### config
```
{
    // required - path to PLY file
    "file" : "<path to .ply file>"
}
```

## supply-detector

Detects ingredient supply level from overhead camera ROIs.

### config
```
{
    // required - camera to capture images from
    "camera": "<camera name>",

    // required - list of bins to monitor
    "bins": [
        {
            "name": "<ingredient name>",
            "x1": 0,
            "y1": 0,
            "x2": 100,
            "y2": 100
        }
    ],

    // optional - fraction of non-steel pixels below which a bin is considered low (default: 0.20)
    "low_threshold": 0.20
}
```

### DoCommand

#### check_supply

Returns the supply level for each configured bin.

```
{
    "check_supply": true
}
```

Response:
```
{
    "<bin name>": { "ratio": <float>, "low": <bool> },
    ...
}
```
