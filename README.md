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
Map of ingredient name to number of servings. Grabs each ingredient until the scale shows the target weight has been reached, then delivers the bowl.
```
{
    "build_salad" : {
        "lettuce" : 2,
        "tomato" : 1,
        "croutons" : 1
    }
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
    "progress" : 40.0             // percentage 0-100
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
