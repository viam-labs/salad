# Module salad

A Viam module for automated salad assembly.

## build-coordinator

Orchestrates salad assembly by grabbing ingredients to target weights and delivering the finished bowl.
Ingredients are added in category order: base, protein, topping, then dressing.
For each ingredient, reads the scale before and after each grab to verify weight was added.
If 3 consecutive grabs produce no weight change (< 0.5g), errors out (possible empty bin).
Once all ingredients are added, calls deliver_bowl.

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
    ]
}
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

    // required - switches and grippers
    "high-above-bowl" : "<switch>",       // switch to position arm high above the bowl
    "left-gripper" : "<gripper>",         // left gripper for grabbing ingredients
    "left-home" : "<switch>",             // switch to send left arm home
    "right-gripper" : "<gripper>",        // right gripper for grabbing/delivering bowl
    "right-above-bowl" : "<switch>",      // switch to position right arm above bowl
    "right-grab-bowl" : "<switch>",       // switch to lower right arm to grab bowl
    "right-above-delivery" : "<switch>",  // switch to position right arm above delivery
    "right-bowl-delivery" : "<switch>",   // switch to lower right arm for delivery
    "right-home" : "<switch>"             // switch to send right arm home
}
```

### DoCommand

#### get_from_bin
Grabs from the named bin and drops into the bowl.
```
{
    "get_from_bin" : "lettuce"
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

#### reset
Sends the right arm home. If `lil-arm-gripper` is configured, also sends the lil-arm home.
```
{
    "reset" : true
}
```

## dressing-controls

Controls the gripper and switches for pouring dressing onto the salad.

### config
```
{
    // required - gripper for grabbing the dressing container
    "gripper" : "<gripper>",

    // required - switch to prepare dressing position
    "prepare-dressing" : "<switch>",

    // required - switch to grab the dressing container
    "grab-dressing" : "<switch>",

    // required - switch to pour dressing
    "pour-dressing" : "<switch>",

    // required - switch for second pour position
    "pour-dressing2" : "<switch>",

    // required - switch for post-pour position
    "post-pour-dressing" : "<switch>",

    // required - switch to send arm home
    "home" : "<switch>",

    // optional - generic service to shake the arm while pouring
    "shake-arm-service" : "<generic service>"
}
```

### DoCommand

#### pour_dressing
Grabs the dressing container, pours dressing over the bowl, then returns the container and sends the arm home.
```
{
    "pour_dressing" : true
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
