package main

import (
	kinematicsutils "github.com/randhid/kinematics-utils"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(resource.APIModel{arm.API, kinematicsutils.KinematicsChecker})
	module.ModularMain(resource.APIModel{arm.API, kinematicsutils.URDFConverter})
	module.ModularMain(resource.APIModel{arm.API, kinematicsutils.MeshViz})
	module.ModularMain(resource.APIModel{camera.API, kinematicsutils.PointCloudViz})
}
