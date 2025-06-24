package main

import (
	kinematicsutils "github.com/randhid/kinematics-utils"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	// Register all models in a single ModularMain call
	module.ModularMain(
		resource.APIModel{arm.API, kinematicsutils.KinematicsChecker},
		resource.APIModel{generic.API, kinematicsutils.URDFConverter},
		resource.APIModel{gripper.API, kinematicsutils.MeshViz},
		resource.APIModel{camera.API, kinematicsutils.PointCloudViz},
	)
}
