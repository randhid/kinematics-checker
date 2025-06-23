# Kinematics Utils Module

A Viam module providing utilities for kinematics checking, mesh visualization, and URDF conversion.

## Models

This module provides three main components:

###Kinematics Checker

A modular arm component that checks your SVA or URDF kinematics against CAD models.
#### Example Configuration

```json
{
  "kinematics_file": "ur20.json",
  "ply-cad-file": "ur20_cad.ply",
  "cad-transform": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
}
```

#### Configuration

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `kinematics-file` | string | Yes | Path to the kinematics file (SVA or URDF format) |
| `ply-cad-file` | string | No | Path to the CAD mesh file in PLY format |
| `cad-transform` | []float | No | Optional transform for the input CAD file to manipulate the mesh in the world state |


### Mesh Visualizer

A gripper component that loads and visualizes 3D mesh files for debugging and visualization purposes.

#### Configuration

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `mesh-file` | string | Yes | Path to the mesh file in PLY format |
| `mesh-transform` | []float | No | Optional transform to apply to the mesh (pose) |

#### Example Configuration

```json
{
  "mesh-file": "robot_arm.ply",
  "mesh-transform": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "o_x": 0.0,
    "o_y": 0.0,
    "o_z": 1.0,
    "theta": 0.0
  }
}
```


### URDF Converter

A generic service that provides URDF conversion utilities.
#### Example Configuration

```json
{
  "urdf-file": "robot.urdf"
}
```

#### Configuration

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `urdf-file` | string | Yes | Path to the URDF file to be processed |


#### Supported Commands

The URDF converter supports the following commands via `DoCommand`:

##### URDF Command
```json
{
  "command": "urdf",
  "urdf_file": "robot.urdf"
}
```

##### URDF to SVA Command
```json
{
  "command": "urdf2sva",
  "urdf_file": "robot.urdf"
}
```

## Installation

1. Add the module to your robot configuration:

```json
{
  "modules": [
    {
      "name": "kinematics-utils",
      "executable_path": "path/to/kinematics-utils"
    }
  ]
}
```


## File Formats

### Supported Kinematics Formats

- **SVA**: JSON format with kinematic parameters
- **URDF**: XML format for robot descriptions

### Supported Mesh Formats

- **PLY**: Polygon file format for 3D meshes

### Example SVA File Structure

```json
{
  "name": "UR20",
  "kinematic_param_type": "SVA",
  "links": [
    {
      "id": "base_link",
      "parent": "world"
    }
  ],
  "joints": [
    {
      "id": "shoulder_pan_joint",
      "type": "revolute",
      "parent": "base_link",
      "axis": {
        "x": 0,
        "y": 0,
        "z": 1
      },
      "max": 360,
      "min": -360
    }
  ]
}
```

### Example URDF File Structure

```xml
<?xml version="1.0"?>
<robot name="ur20-test">
  <link name="base_link"/>
  <link name="shoulder_link"/>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```
