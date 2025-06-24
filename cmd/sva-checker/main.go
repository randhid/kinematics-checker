package main

import (
	"context"
	"flag"

	vizclient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/components/arm/fake"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

const (
	plyFile = "ur3e.ply"
)

var (
	svaConfig = resource.Config{ConvertedAttributes: &fake.Config{ModelFilePath: "ur3e.json"}}
)

func main() {
	logger := logging.NewLogger("svachekcer")

	vizualizeAssy := flag.Bool("with-assembly", false, "visualize the arm assembly mesh")
	flag.Parse()

	armee, err := fake.NewArm(context.Background(), resource.Dependencies{}, svaConfig, nil)
	if err != nil {
		logger.Error(err)
		return
	}
	inputs := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})

	// inputs := referenceframe.FloatsToInputs([]float64{1.5708, -1.5708, 0, 0, 0, 0})
	m, err := armee.Kinematics(context.Background())
	if err != nil {
		logger.Error(err)
	}
	ugif, err := m.Geometries(inputs)
	if err != nil {
		logger.Error(err)
	}
	if err != nil {
		logger.Error(err)
		return
	}
	geoms := ugif.Geometries()

	for _, g := range geoms {
		vizclient.DrawGeometry(g, "red")
	}

	if *vizualizeAssy {
		logger.Info("Visualizing  assembly mesh")
		mesh, err := spatialmath.NewMeshFromPLYFile(plyFile)
		if err != nil {
			logger.Error(err)
			return
		}
		tfMesh := mesh.Transform(
			spatialmath.NewPoseFromOrientation(&spatialmath.OrientationVectorDegrees{Theta: 90, OX: 1, OY: 0, OZ: 0}))

		if err := vizclient.DrawGeometry(tfMesh, "blue"); err != nil {
			logger.Error(err)
			return
		}
		logger.Info("Visualized  assembly mesh")
	}

}
