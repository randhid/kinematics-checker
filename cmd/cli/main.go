package main

import (
	"context"

	kinematicsutils "github.com/randhid/kinematics-utils"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}

func realMain() error {
	ctx := context.Background()
	logger := logging.NewDebugLogger("cli")

	deps := resource.Dependencies{}
	// can load these from a remote machine if you need

	cfg := kinematicsutils.KinConfig{
		KinematicsFile: "ur20.json",
		CADFile:        "ur20.ply",
	}

	thing, err := kinematicsutils.NewKinematicsChecker(
		ctx, deps, arm.Named("foo"), &cfg, logger)
	if err != nil {
		return err
	}
	defer thing.Close(ctx)

	thing3, err := kinematicsutils.NewMeshViz(
		ctx, deps, gripper.Named("foo3"), &kinematicsutils.MeshVizConfig{
			MeshFile: "ur3e.ply",
		}, logger)
	if err != nil {
		return err
	}
	defer thing3.Close(ctx)

	thing4, err := kinematicsutils.NewPointCloudViz(
		ctx, deps, camera.Named("foo4"), &kinematicsutils.PointCloudVizConfig{
			PointCloudFile: "example.pcd",
		}, logger)
	if err != nil {
		return err
	}
	defer thing4.Close(ctx)

	thing2, err := kinematicsutils.NewURDFConverter(
		ctx, deps, generic.Named("foo2"), &kinematicsutils.URDFConverterConfig{
			URDFFile: "ur20.urdf",
		}, logger)
	if err != nil {
		return err
	}
	defer thing2.Close(ctx)

	return nil
}
