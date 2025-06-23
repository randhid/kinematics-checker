# Module kinematics-utils 

Provide a description of the purpose of the module and any relevant information.

## Model rand:kinematics-utils:kinematics-checker

Provide a description of the model and any relevant information.

### Configuration
The following attribute template can be used to configure this model:

```json
{
"kinematics_file": <float>,
"ply-cad-file": <string>
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `kinematics-file` | string  | Required  | A kinematics file describing the arm's joints and geometries. Can be an SVA ,.json or a .urdf |
| `ply-cad-file` | string | Optional  | a .ply file of the CAD model of the arm being modeled. |
| `cad-transform` | []float | Optional  | Optional transform for the input CAD file to manipulate the mesh in the world state. |

#### Example Configuration

```json
{
  "kinematics_file": "ur20.json",
  "ply-cad-file": "foo"
}
```

### DoCommand

If your model implements DoCommand, provide an example payload of each command that is supported and the arguments that can be used. If your model does not implement DoCommand, remove this section.

#### Example DoCommand

```json
{
  "command_name": {
    "arg1": "foo",
    "arg2": 1
  }
}
```
