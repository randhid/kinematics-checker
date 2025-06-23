package kinematicsutils

import (
	"errors"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

var (
	family            = resource.NewModelFamily("rand", "kinematics-utils")
	KinematicsChecker = family.WithModel("kinematics-checker")
	URDFConverter     = family.WithModel("urdf-converter")
	MeshViz           = family.WithModel("mesh-viz")

	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterComponent(arm.API, KinematicsChecker,
		resource.Registration[arm.Arm, *KinConfig]{
			Constructor: newKinChecker,
		},
	)

	resource.RegisterService(generic.API, URDFConverter,
		resource.Registration[resource.Resource, *URDFConverterConfig]{
			Constructor: newURDFConverter,
		},
	)

	resource.RegisterComponent(gripper.API, MeshViz,
		resource.Registration[gripper.Gripper, *MeshVizConfig]{
			Constructor: newMeshViz,
		},
	)

	resource.RegisterComponent(camera.API, PointCloudViz,
		resource.Registration[camera.Camera, *PointCloudVizConfig]{
			Constructor: newPointCloudViz,
		},
	)
}
